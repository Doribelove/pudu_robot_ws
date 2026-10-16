import pytest
from run_task_progress import select_queries


def test_unknown_seed_cannot_silently_run_only_shared_new_tasks():
    original = [{'id': v} for v in ('Q03', 'SHARED-01', 'SHARED-02', 'SHARED-03')]
    with pytest.raises(ValueError, match='UNKNOWN_QUERY_IDS:Q01'):
        select_queries(original, ['Q01', 'SHARED-01', 'SHARED-02', 'SHARED-03'])
    assert select_queries(original, [q['id'] for q in original]) == original


def test_one_requested_progress_id_preserves_all_four_original_steps():
    original = [{'id': 'Q09', 'step': step} for step in ('initial', 'p20', 'p50', 'p80')]
    assert select_queries(original, ['Q09']) == original
    with pytest.raises(ValueError, match='DUPLICATE_QUERY_ID_SELECTION'):
        select_queries(original, ['Q09', 'Q09'])
