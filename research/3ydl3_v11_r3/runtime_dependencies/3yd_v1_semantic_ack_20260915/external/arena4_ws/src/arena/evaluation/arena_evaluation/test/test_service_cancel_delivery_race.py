import threading

from test_static_planner_service_r3 import service,req,valid,until


def test_acknowledged_cancel_cannot_race_with_valid_path_delivery(service,monkeypatch):
    s,workers=service;entered=threading.Event();release=threading.Event()
    original=s._finish
    def paused_finish(task,result):
        entered.set();assert release.wait(timeout=1.)
        return original(task,result)
    monkeypatch.setattr(s,'_finish',paused_finish)
    f=s.submit(req('a'));until(lambda:bool(workers[0].sent))
    workers[0].events.append({'event':'result','value':valid('a')})
    assert entered.wait(timeout=1.)
    answer=[];attempted=threading.Event()
    def cancel():attempted.set();answer.append(s.cancel('a'))
    thread=threading.Thread(target=cancel);thread.start();assert attempted.wait(timeout=1.)
    # A cancellation may linearize before delivery or after it. It cannot
    # report success and then deliver the same request's valid path.
    thread.join(timeout=.05);release.set();thread.join(timeout=1.)
    result=f.result(timeout=1.)
    assert answer and not (answer[0] and result['final_valid_success'])
