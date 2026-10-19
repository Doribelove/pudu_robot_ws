"""Capture the same Smac action before/after smoothing without changing search."""
import time,math
import numpy as np
from .diagnostics import DiagnosticSession

class RawDiagnosticSession(DiagnosticSession):
    def start(self):
        super().start()
        from nav_msgs.msg import Path
        self._raw_messages=[];self._raw_counter=0
        self._raw_subscription=self.client.node.create_subscription(Path,'/unsmoothed_plan',self._capture,10)

    def _capture(self,msg):
        rows=[]
        for p in msg.poses:
            v=p.pose;q=v.orientation
            rows.append([v.position.x,v.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
        self._raw_messages.append((msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,np.asarray(rows,float)))

    def plan(self,query,*args,**kwargs):
        self._raw_messages=[];started=time.time_ns()
        result=super().plan(query,*args,**kwargs)
        # Flush already-published diagnostics; no planner-state settling delay.
        deadline=time.monotonic()+.5
        while result.planner_success and not any(stamp>=started-1000000 for stamp,_ in self._raw_messages) and time.monotonic()<deadline:
            self.client.executor.spin_once(timeout_sec=.01)
        messages=[p for stamp,p in self._raw_messages if stamp>=started-1000000]
        self._raw_counter+=1
        root=self.params_file.parent/'raw_paths';root.mkdir(exist_ok=True)
        tag=f'{query.query_id}_{self._raw_counter:03d}'
        record={'same_action_raw_path_received':bool(messages),'tag':tag,'action_start_ns':started,'observed_stamps':[s for s,_ in self._raw_messages]}
        if messages:
            p=root/(tag+'_raw.npy');np.save(p,messages[-1]);record['raw_path_file']=str(p)
        if result.points:
            p=root/(tag+'_returned.npy');np.save(p,np.array([[v['x'],v['y'],v['yaw']] for v in result.points]));record['returned_path_file']=str(p)
        result.diagnostics['raw_comparison']=record
        return result
