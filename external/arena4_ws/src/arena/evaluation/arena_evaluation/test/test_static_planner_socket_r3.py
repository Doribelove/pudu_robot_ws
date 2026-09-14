import json
from pathlib import Path
import socket
import threading
from concurrent.futures import Future

import pytest

from arena_evaluation.static_planner_service_cli_r3 import LocalServiceSocket


class Supervisor:
    def status(self):return {'state':'READY'}
    def cancel(self,rid):return rid=='active'
    def submit(self,r):
        f=Future();f.set_result({'request_id':r['request_id'],'final_valid_success':False,'points':[]})
        return f


@pytest.fixture
def server(tmp_path):
    # Linux sockaddr_un has a strict length limit independent of filesystem.
    import tempfile
    with tempfile.TemporaryDirectory(prefix='pln-api-',dir='/tmp') as d:
        s=LocalServiceSocket(Supervisor(),Path(d)/'api.sock');s.start()
        t=threading.Thread(target=s.serve);t.start()
        yield s
        s.stopping.set();t.join(timeout=1);s.close()
        assert not s.path.exists() and not t.is_alive()


def call(s,value):
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
        client.settimeout(2.);client.connect(str(s.path));client.sendall(json.dumps(value).encode()+b'\n')
        return json.loads(client.makefile('rb').readline())


def test_local_status_and_cancel(server):
    assert call(server,{'command':'status'})=={'ok':True,'result':{'state':'READY'}}
    assert call(server,{'command':'cancel','request_id':'active'})['result']['cancel_requested']
    assert server.path.stat().st_mode & 0o777==0o600


def test_unknown_command_and_nonobject_are_rejected(server):
    assert not call(server,{'command':'execute'})['ok']
    assert not call(server,[])['ok']


def test_path_output_keeps_rejection_semantics(server):
    result=call(server,{'command':'plan','request_id':'a'})['result']
    assert result['request_id']=='a' and not result['points'] and not result['final_valid_success']


def test_preexisting_socket_path_is_never_overwritten(tmp_path):
    path=tmp_path/'existing';path.write_text('user data')
    s=LocalServiceSocket(Supervisor(),path)
    try:
        with pytest.raises(FileExistsError):s.start()
    finally:s.close()
    assert path.read_text()=='user data'
