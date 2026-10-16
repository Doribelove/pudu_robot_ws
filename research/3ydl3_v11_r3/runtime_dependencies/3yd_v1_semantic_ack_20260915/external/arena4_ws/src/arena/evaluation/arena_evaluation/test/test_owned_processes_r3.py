from arena_evaluation import owned_processes_r3 as module


def row(pid,parent,born,state='S'):
    return {'pid':pid,'ppid':parent,'start_ticks':born,'state':state,'rss_bytes':100}


def test_only_verified_descendants_are_owned(monkeypatch):
    table={10:row(10,1,5),11:row(11,10,6),12:row(12,11,7),20:row(20,1,9)}
    monkeypatch.setattr(module,'process_stat',lambda pid:table.get(pid))
    monkeypatch.setattr(module,'process_table',lambda:table)
    tree=module.OwnedProcessTree(10)
    assert tree.capture()['rss_bytes']==300
    assert set(tree.identities)=={10,11,12}
    # Reparenting retains only the previously verified child identity.
    table[11]['ppid']=1
    assert tree.capture()['rss_bytes']==300


def test_reused_pid_and_its_children_are_never_signalled(monkeypatch):
    table={10:row(10,1,5),11:row(11,10,6)};sent=[]
    monkeypatch.setattr(module,'process_stat',lambda pid:table.get(pid))
    monkeypatch.setattr(module,'process_table',lambda:table)
    monkeypatch.setattr(module,'signal_identity',lambda pid,born,sig:sent.append(pid) or True)
    tree=module.OwnedProcessTree(10);tree.capture()
    table[10]=row(10,1,50);table[12]=row(12,10,60)
    tree.capture()
    assert 12 not in tree.identities
    assert not tree._signal(10,9) and sent==[]
    assert tree._signal(11,9) and sent==[11]


def test_retirement_cannot_claim_success_for_a_survivor(monkeypatch):
    table={10:row(10,1,5)}
    monkeypatch.setattr(module,'process_stat',lambda pid:table.get(pid))
    monkeypatch.setattr(module,'process_table',lambda:table)
    monkeypatch.setattr(module,'signal_identity',lambda *a:True)
    tree=module.OwnedProcessTree(10)
    result=tree.retire(timeout_s=.02)
    assert not result['confirmed'] and result['remaining'][0]['pid']==10


def test_pidfd_is_opened_before_identity_validation_and_always_closed(monkeypatch):
    events=[]
    monkeypatch.setattr(module.os,'pidfd_open',lambda pid:events.append('open') or 99)
    monkeypatch.setattr(module,'process_stat',lambda pid:events.append('validate') or row(pid,1,7))
    monkeypatch.setattr(module.signal,'pidfd_send_signal',lambda fd,sig:events.append('signal'))
    monkeypatch.setattr(module.os,'close',lambda fd:events.append('close'))
    assert module.signal_identity(10,7,9)
    assert events==['open','validate','signal','close']
    events.clear()
    assert not module.signal_identity(10,6,9)
    assert events==['open','validate','close']
