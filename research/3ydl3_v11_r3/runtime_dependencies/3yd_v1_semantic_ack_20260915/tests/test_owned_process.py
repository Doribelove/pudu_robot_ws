import json,mmap,os,signal,subprocess,sys,time,uuid
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from owned_process import supervise,cleanup,fingerprint,save,recover_pending


def test_normal_exit_and_killed_worker_release_owned_segments(tmp_path):
    for kill in [False,True]:
        name='fastrtps_3ydtest_'+uuid.uuid4().hex
        command=[sys.executable,'-c',f"import mmap,os,time,signal;f=open('/dev/shm/{name}','w+b');f.truncate(4096);m=mmap.mmap(f.fileno(),4096);m[0]=1;time.sleep(.5);"+("os.kill(os.getpid(),signal.SIGKILL)" if kill else "")]
        journal=tmp_path/(name+'.supervisor.json');code=supervise(command,journal)
        d=json.loads(journal.read_text())
        assert code==(-signal.SIGKILL if kill else 0)
        assert d['cleanup']['complete'] and not Path('/dev/shm',name).exists()
        assert name in d['resources']


def test_existing_and_live_mapped_resources_are_never_deleted(tmp_path):
    name='fastrtps_3ydtest_'+uuid.uuid4().hex;p=Path('/dev/shm')/name
    try:
        p.write_bytes(b'x'*4096)
        journal=tmp_path/'existing.supervisor.json'
        code=supervise([sys.executable,'-c',f"import mmap,time;f=open('{p}','r+b');m=mmap.mmap(f.fileno(),0);time.sleep(.4)"],journal)
        assert code==0 and p.exists() and name not in json.loads(journal.read_text())['resources']
        with p.open('r+b') as f:
            mapping=mmap.mmap(f.fileno(),0)
            result=cleanup({'processes':{},'resources':{name:fingerprint(p)}})
            assert not result['complete'] and name in result['retained'] and p.exists()
            mapping.close()
    finally:p.unlink(missing_ok=True)


def test_changed_inode_is_left_untouched_and_stale_journal_recovers(tmp_path):
    name='fastrtps_3ydtest_'+uuid.uuid4().hex;p=Path('/dev/shm')/name
    try:
        p.write_bytes(b'a');old=fingerprint(p);other=p.with_name(p.name+'replacement');other.write_bytes(b'b');other.replace(p)
        d={'schema':'3yd-owned-process-v1','state':'running','supervisor':{'pid':999999999,'created':0},'processes':{},'resources':{name:old}}
        result=cleanup(d);assert p.read_bytes()==b'b' and result['replaced_names_left_untouched']==[name]
        d['resources'][name]=fingerprint(p);journal=tmp_path/'orphan.supervisor.json';save(journal,d)
        recovered=recover_pending(tmp_path)
        assert recovered and not p.exists() and json.loads(journal.read_text())['state']=='complete'
    finally:p.unlink(missing_ok=True)
