"""Run original event replay; read source5 feature counters after its original clocks."""
if __name__ == '__main__':
    from owned_process import supervised_entry
    supervised_entry()
import json
import sys
from pathlib import Path
from compare_v9_review import save
from three_yd_v5.v9_r9_scope import scope_stats
from run_progress_events import main as original_main

def main():
    assert '--output' in sys.argv
    out = Path(sys.argv[sys.argv.index('--output') + 1])
    before = scope_stats()
    original_main()
    after = scope_stats()
    summary = json.loads((out/'summary.json').read_text())
    want = 'X' in summary['r9_mode']
    assert summary['assertions_passed'] and after['mode_enabled'] == want
    assert not after['current_input_or_scope_result_cached'] and not after['audit_or_ACK_results_cached']
    assert after['logical_padding_byte_counter_is_not_RSS']
    if want:
        assert after['exact_scope_calls'] > before['exact_scope_calls']
        assert after['disabled_calls'] == 0
    else:
        assert after['exact_scope_calls'] == 0
    save(out/'source5_event_features.json', {
        'source_cycle':5, 'original_run_progress_events_main_called_unchanged':True,
        'inspection_after_all_original_event_clocks':True,
        'logical_counters_not_RSS':True, 'before':before, 'after':after,
        'actual_X_exercised':want and after['exact_scope_calls']>before['exact_scope_calls']})
    print('SOURCE5_EVENT_ORIGINAL_ASSERTIONS_AND_ACTUAL_X_OUTSIDE_CLOCKS', summary['map'], after['exact_scope_calls'], flush=True)

if __name__ == '__main__':main()
