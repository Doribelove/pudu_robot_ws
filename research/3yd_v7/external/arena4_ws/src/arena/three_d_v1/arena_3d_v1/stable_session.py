"""Bounded readback retries and orderly ownership-scoped ROS shutdown.

The full-content ACK remains the acceptance barrier. A lost first service
reply must not consume the entire ACK deadline or leave an orphaned future.
"""
import os
import signal
import time
import gc
import hashlib
from pathlib import Path
import numpy as np
import psutil
from .semantic_pipeline import ReferenceSmacSession
from .semantic_world import json_write

_DEFERRED_OWNED_SHM = set()


def mapped_shm(pids):
    names = set()
    for pid in pids:
        try:
            for line in Path(f'/proc/{pid}/maps').read_text().splitlines():
                if '/dev/shm/fastrtps_' in line:
                    name = line.split('/dev/shm/', 1)[1].split(' ')[0]
                    names.add(name)
        except (OSError, ProcessLookupError):
            pass
    return names


def orderly_stop(stack):
    """Signal launch once; let it stop its children before group escalation."""
    process = stack.process
    events = []
    if process is not None:
        def members():
            result = []
            for p in psutil.process_iter(['pid', 'status']):
                try:
                    if os.getpgid(p.pid) == process.pid and p.info['status'] != psutil.STATUS_ZOMBIE:
                        result.append(p.pid)
                except (OSError, psutil.Error):
                    pass
            return result
        for sig, grace in [(signal.SIGINT, 10.), (signal.SIGTERM, 3.), (signal.SIGKILL, 2.)]:
            living = members()
            if not living:
                break
            # Launch propagates SIGINT to children. Sending to the entire
            # group as well gives lifecycle nodes duplicate interrupts.
            try:
                if sig == signal.SIGINT and process.poll() is None:
                    process.send_signal(sig)
                else:
                    os.killpg(process.pid, sig)
            except ProcessLookupError:
                pass
            events.append({'signal':int(sig), 'members':living})
            until = time.monotonic()+grace
            while time.monotonic() < until and members():
                process.poll()
                time.sleep(.05)
        try:
            process.wait(timeout=.1)
        except Exception:
            pass
        survivors = members()
        if survivors:
            raise RuntimeError(f'OWNED_STACK_PROCESSES_REMAIN: {survivors}')
    if stack.log_stream is not None:
        stack.log_stream.close()
    return events


class StableSmacSession(ReferenceSmacSession):
    def __init__(self, ctx, output, **kwargs):
        self.lifecycle_root = Path(output)/'lifecycle_evidence'
        self.lifecycle_root.mkdir(parents=True, exist_ok=True)
        self.readbacks = []
        self.ack_events = []
        self.shutdown_events = []
        self.owned_shm = set()
        self.closed = False
        super().__init__(ctx, output, **kwargs)
        self.stack.stop = self._stop_stack

    def _wait_for_costmap_ack(self, expected, changed, **kwargs):
        if self._semantic_costmap is None:
            result = self._mask_content_ack(expected, changed, **kwargs)
        else:
            result = super()._wait_for_costmap_ack(expected, changed, **kwargs)
        self.ack_events.append(dict(result))
        return result

    def _mask_content_ack(self, expected, changed, timeout_s=None):
        """Confirm every affected mask cell; repair repeated observed gaps.

        Retain the inherited mask contract: closed cells must be 254 and
        opened cells must be below 254 (inflation is allowed). Cold startup
        checks the entire all-lethal map. Semantic cost maps still use the
        parent's exact effective-master comparison. Two reassertions are
        bounded by the original deadline, without a gap-size exclusion.
        """
        expected=np.asarray(expected,dtype=np.int8)
        cold=bool(np.all(expected==100))
        checked=np.ones(expected.shape,bool) if cold else np.asarray(changed,dtype=bool)
        started=time.monotonic();deadline=started+float(timeout_s or self.costmap_ack_timeout_s)
        attempts=repairs=messages=cells=stable=0;stamp=-1;last_error='';last_hash=''
        mismatch=checked.copy();last_repair=started;last_bad_hash=None;bad_observations=0
        serialization_ms=publication_ms=0.
        while time.monotonic()<deadline:
            attempts+=1
            try:server,stamp=self._server_costmap_snapshot(deadline)
            except RuntimeError as error:
                last_error=str(error);stable=0;continue
            last_error=''
            mismatch=checked & np.where(expected==100,server!=254,server>=254)
            last_hash=hashlib.sha256(np.ascontiguousarray(server[checked]).tobytes()).hexdigest()
            if not np.any(mismatch):
                stable+=1
                if stable>=2:
                    self._last_server_update_time_ns=stamp;self._costmap_ack_sequence+=1
                    result={'costmap_update_acknowledged':True,'costmap_ack_status':'cold_exact_content_verified' if cold else 'mask_content_verified',
                            'costmap_ack_stable_observations':stable,'costmap_ack_mismatch_cells':0}
                    break
            else:
                stable=0;bad_observations=bad_observations+1 if last_bad_hash==last_hash else 1;last_bad_hash=last_hash
                if repairs<2 and bad_observations>=2 and time.monotonic()-last_repair>=.05:
                    _,info=self._publish_dirty_roi(expected,mismatch)
                    repairs+=1;messages+=int(info.get('roi_message_count',0));cells+=int(info.get('roi_published_cells',0))
                    serialization_ms+=float(info.get('local_map_serialization_ms',0));publication_ms+=float(info.get('local_map_publication_ms',0))
                    last_repair=time.monotonic();bad_observations=0
            self.client.executor.spin_once(timeout_sec=min(.01,max(0.,deadline-time.monotonic())))
        else:
            result={'costmap_update_acknowledged':False,'costmap_ack_status':'readback_error' if last_error else 'server_content_mismatch',
                    'costmap_ack_mismatch_cells':int(np.count_nonzero(mismatch)),'costmap_ack_error':last_error}
        return {**result,'costmap_ack_wait_ms':(time.monotonic()-started)*1000,'costmap_ack_attempts':attempts,
                'costmap_ack_checked_cells':int(np.count_nonzero(checked)),'costmap_ack_sequence':self._costmap_ack_sequence,
                'server_costmap_update_time_ns':int(stamp),'server_costmap_content_hash':last_hash,
                'costmap_ack_repair_count':repairs,'costmap_ack_repair_messages':messages,'costmap_ack_repair_cells':cells,
                'costmap_ack_repair_serialization_ms':serialization_ms,'costmap_ack_repair_publication_ms':publication_ms}

    def _stop_stack(self):
        self.shutdown_events.extend(orderly_stop(self.stack))

    def _server_costmap_snapshot(self, deadline):
        if self._get_costmap_client is None or self.client is None:
            raise RuntimeError('global costmap readback service is unavailable')
        started = time.monotonic()
        # Share the existing overall deadline; do not extend it on retry.
        attempt_deadline = min(deadline, started+.75)
        future = None
        record = {'started':started, 'attempt_deadline':attempt_deadline, 'overall_deadline':deadline}
        try:
            if not self._get_costmap_client.wait_for_service(timeout_sec=min(.2,max(0.,attempt_deadline-time.monotonic()))):
                raise RuntimeError('global costmap readback service unavailable')
            future = self._get_costmap_client.call_async(self.GetCostmap.Request())
            while not future.done() and time.monotonic() < attempt_deadline:
                self.client.executor.spin_once(timeout_sec=min(.01,max(0.,attempt_deadline-time.monotonic())))
            if not future.done():
                raise RuntimeError('global costmap readback attempt timed out')
            response = future.result()
            message = getattr(response, 'map', None)
            meta = getattr(message, 'metadata', None)
            width, height = int(getattr(meta,'size_x',0)),int(getattr(meta,'size_y',0))
            if (width,height) != (self.ctx.hospital_map.width,self.ctx.hospital_map.height):
                raise RuntimeError(f'global costmap readback shape mismatch: {width}x{height}')
            data = np.frombuffer(bytes(getattr(message,'data',b'')),dtype=np.uint8)
            if data.size != width*height:
                raise RuntimeError('global costmap readback data length mismatch')
            stamp = getattr(meta,'update_time',None)
            stamp_ns = int(getattr(stamp,'sec',0))*1_000_000_000+int(getattr(stamp,'nanosec',0))
            record.update(success=True, stamp_ns=stamp_ns, bytes=int(data.size))
            return data.reshape(height,width),stamp_ns
        except Exception as error:
            record.update(success=False,error=str(error))
            raise
        finally:
            if future is not None and not future.done():
                # A timed-out request cannot accumulate or accept a late reply.
                self._get_costmap_client.remove_pending_request(future)
                future.cancel()
                record['pending_request_removed'] = True
            record['elapsed_s'] = time.monotonic()-started
            self.readbacks.append(record)

    def close(self):
        if self.closed:
            return
        owned = [os.getpid()]
        if self.stack.process is not None:
            try:
                parent = psutil.Process(self.stack.process.pid)
                owned += [parent.pid]+[p.pid for p in parent.children(recursive=True)]
            except psutil.Error:
                pass
        self.owned_shm |= mapped_shm(owned)
        try:
            if self.client is not None:
                # ActionClient is a Waitable, not one of Node._clients.
                # Humble Node.destroy_node() does not destroy waitables.
                # Release it before destroying the Node/Context so its native
                # handle cannot retain the DDS participant until Python GC.
                self.client.client.destroy()
            super().close()
        finally:
            # shutdown() stops ROS, but leaves the native Context allocated
            # until Python collection. Explicit destruction releases this
            # session's DDS participant even when reference cycles survive.
            self.context.destroy()
            # Executor/future callback cycles can retain destroyed native
            # handles. Collect them at the explicit session boundary.
            gc.collect()
            # Delete only transport files observed mapped by this session,
            # after every accessible process has stopped using them. Fail
            # conservatively on unreadable same-user maps.
            protected = set()
            unreadable = []
            unrelated_helpers = []
            for p in psutil.process_iter(['pid','uids','status','name','cmdline']):
                try:
                    maps = Path(f'/proc/{p.pid}/maps').read_text()
                    for line in maps.splitlines():
                        if '/dev/shm/' in line:
                            protected.add(line.split('/dev/shm/',1)[1].split(' ')[0])
                    for fd in Path(f'/proc/{p.pid}/fd').iterdir():
                        try:
                            target=os.readlink(fd)
                            if target.startswith('/dev/shm/'):
                                protected.add(target[len('/dev/shm/'):].split(' ')[0])
                        except OSError:
                            pass
                except PermissionError:
                    if p.info.get('uids') and p.info['uids'].real == os.getuid() and p.info['status'] != psutil.STATUS_ZOMBIE:
                        # These system credential/mount helpers are not ROS
                        # participants. Other unreadable processes still block
                        # cleanup. Record every exemption for review.
                        if p.info['name'] in {'(sd-pam)','ssh-agent','fusermount3'}:
                            unrelated_helpers.append({'pid':p.pid,'name':p.info['name'],'cmdline':p.info['cmdline']})
                        else:
                            unreadable.append(p.pid)
                except OSError:
                    pass
            removed = []
            _DEFERRED_OWNED_SHM.update(self.owned_shm)
            if not unreadable:
                candidates=_DEFERRED_OWNED_SHM|{name+'_el' for name in _DEFERRED_OWNED_SHM}
                for name in sorted(candidates-protected):
                    path = Path('/dev/shm')/name
                    try:
                        if path.is_file() and path.stat().st_uid == os.getuid():
                            path.unlink();removed.append(name)
                    except FileNotFoundError:
                        pass
            deferred_before=sorted(_DEFERRED_OWNED_SHM)
            _DEFERRED_OWNED_SHM.intersection_update({name for name in _DEFERRED_OWNED_SHM if (Path('/dev/shm')/name).exists()})
            json_write(self.lifecycle_root/'shutdown.json', {'signals':self.shutdown_events,
                       'owned_shm':sorted(self.owned_shm),'removed':removed,'protected':sorted(protected),
                       'cleanup_skipped_unreadable_pids':unreadable,'unrelated_unreadable_helpers':unrelated_helpers,
                       'deferred_ownership_considered':deferred_before,'deferred_remaining':sorted(_DEFERRED_OWNED_SHM)})
            json_write(self.lifecycle_root/'readbacks.json',self.readbacks)
            json_write(self.lifecycle_root/'acks.json',self.ack_events)
            self.closed = True
