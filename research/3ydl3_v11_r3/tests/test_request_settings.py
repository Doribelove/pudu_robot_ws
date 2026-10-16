import json
import pytest
from three_yd_v5 import curve_backend as b


def test_identity_reads_once_and_checks_again_at_exit(monkeypatch):
    original=b._read_settings;calls=[]
    def read():calls.append(1);return original()
    monkeypatch.setattr(b,'_read_settings',read)
    with b.settings_scope():
        identities=[b.identity() for _ in range(40)]
        assert len(set(identities))==1 and len(calls)==1
        cfg=b.settings()[1];cfg['endpoint_tolerance']['yaw_deg']=123
        cfg['total_request_budget_s']=999
        assert b.settings()[1]['endpoint_tolerance']['yaw_deg']==30
        assert b.settings()[1]['total_request_budget_s']==30
    assert len(calls)==2
    b.settings();assert len(calls)==3


def test_environment_change_is_rejected_and_context_does_not_leak(monkeypatch):
    with pytest.raises(ValueError,match='CONFIG_VERSION_CHANGED'):
        with b.settings_scope():
            before=b.identity()
            monkeypatch.setenv('THREEYD_ENDPOINT_TOLERANCE','0')
            assert b.identity()==before
    assert b._REQUEST_SETTINGS.get() is None
    assert b.settings()[1]['endpoint_tolerance']['enabled'] is False


def test_config_file_change_rejected_at_delivery(monkeypatch,tmp_path):
    p=tmp_path/'l3.json';p.write_bytes(b.CONFIG_PATH.read_bytes())
    monkeypatch.setattr(b,'CONFIG_PATH',p)
    with pytest.raises(ValueError,match='CONFIG_VERSION_CHANGED'):
        with b.settings_scope():
            cfg=json.loads(p.read_text());cfg['total_request_budget_s']=29
            p.write_text(json.dumps(cfg))
    assert b._REQUEST_SETTINGS.get() is None


def test_exception_and_nested_scopes_restore_context():
    with pytest.raises(RuntimeError,match='fail'):
        with b.settings_scope():
            with b.settings_scope():assert b._REQUEST_SETTINGS.get() is not None
            raise RuntimeError('fail')
    assert b._REQUEST_SETTINGS.get() is None


def test_plan_withdraws_path_if_configuration_changes_before_delivery(monkeypatch):
    from test_task_progress import fake_service
    s,q,p=fake_service();original=s._plan
    def change(query):
        result=original(query)
        monkeypatch.setenv('THREEYD_ENDPOINT_TOLERANCE','0')
        return result
    s._plan=change
    with pytest.raises(ValueError,match='CONFIG_VERSION_CHANGED'):s.plan(q)
    assert s.current_trajectory() is None


def test_rejected_clear_preserves_untrusted_history_for_later_recertification():
    from test_task_progress import fake_service
    s,q,p=fake_service();retained=s.record;s.retained_record=retained;s.retained_path=p.copy()
    def rejected(*a,**k):raise ValueError('CLEAR_REQUIRES_FREE_OBSERVATION')
    s._guarded_update=rejected
    with pytest.raises(ValueError,match='CLEAR_REQUIRES_FREE_OBSERVATION'):s.update(4,clear=['past'])
    assert s.current_trajectory() is None and s.record is None
    assert s.retained_record is retained
