from two_a_v1_nav2.contracts import EXPECTED_ARCHIVE_SHA256, verify_frozen_2a_sources


def test_frozen_2a_source_snapshot_matches_runtime():
    assert EXPECTED_ARCHIVE_SHA256.startswith("156818afd8f7")
    assert len(verify_frozen_2a_sources()) == 15
