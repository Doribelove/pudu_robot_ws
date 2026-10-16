from three_d_v1_nav2.mission_state import Acceptance, MissionMachine, MissionState


PASS = Acceptance(True, True, True, True, True)


def drive_to_active(machine: MissionMachine, uuid: str = "uuid-1") -> None:
    machine.stack_active()
    machine.initial_pose_set()
    machine.localization_ready()
    machine.goal_sent(uuid)


def test_success_is_the_only_route_to_index_increment():
    machine = MissionMachine(("q1", "q2"))
    drive_to_active(machine)
    machine.action_result(True)
    machine.verify(PASS)
    assert machine.query_index == 0
    assert machine.state == MissionState.STOPPED_AND_AUDITED
    machine.advance()
    assert machine.query_index == 1
    assert machine.state == MissionState.SET_INITIAL_POSE


def test_aborted_action_halts_on_current_query():
    machine = MissionMachine(("q1", "q2"))
    drive_to_active(machine)
    machine.action_result(False, "NAV2_ABORTED")
    assert machine.state == MissionState.HALT_CURRENT_QUERY
    assert machine.query_index == 0


def test_any_failed_acceptance_gate_halts_without_advancing():
    for failed_field in range(5):
        machine = MissionMachine(("q1", "q2"))
        drive_to_active(machine, f"uuid-{failed_field}")
        machine.action_result(True)
        values = [True] * 5
        values[failed_field] = False
        machine.verify(Acceptance(*values))
        assert machine.state == MissionState.HALT_CURRENT_QUERY
        assert machine.query_index == 0


def test_duplicate_goal_uuid_fails_closed():
    machine = MissionMachine(("q1", "q2"))
    drive_to_active(machine, "same")
    machine.action_result(True)
    machine.verify(PASS)
    machine.advance()
    machine.initial_pose_set()
    machine.localization_ready()
    machine.goal_sent("same")
    assert machine.state == MissionState.HALT_CURRENT_QUERY
    assert machine.query_index == 1


def test_eight_queries_complete_in_frozen_order():
    ids = tuple(f"q{i}" for i in range(8))
    machine = MissionMachine(ids)
    for index, query_id in enumerate(ids):
        if index == 0:
            machine.stack_active()
        machine.initial_pose_set()
        machine.localization_ready()
        assert machine.query_id == query_id
        machine.goal_sent(f"uuid-{index}")
        machine.action_result(True)
        machine.verify(PASS)
        machine.advance()
    assert machine.state == MissionState.COMPLETE
    assert machine.query_index == 7
