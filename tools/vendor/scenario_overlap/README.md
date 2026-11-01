# Feature 002 Scenario Overlap Snapshot

`feature002_overlap.py` is copied byte-for-byte from Feature 002 commit
`3a0a835179c7366f2921d181436a019b28ba7dca`, source path
`tools/rc_scenarios/overlap.py`. Its pinned SHA-256 is
`d01ede3dbdfc73606cc52cd8e03c3f86da07d80c690ed48c4d860837b7849dd3`.

Feature 007 imports this snapshot only to compare read-only normalized scenario
projections against the frozen Feature 001/002 sets. Do not edit or copy it back
to a historical Feature 001/002 path. `tools/tests/test_scenario_overlap_snapshot.py`
checks the exact digest and a representative parameter projection.
