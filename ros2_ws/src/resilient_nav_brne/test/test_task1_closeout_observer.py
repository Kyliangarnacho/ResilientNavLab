"""Unit contract for closeout stale evidence when BRNE chooses a safe stop."""

import time

from geometry_msgs.msg import Twist

from resilient_nav_brne.brne_task1_closeout_observer import BrneTask1CloseoutObserver


def test_zero_raw_after_goal_silence_is_stale_evidence_even_without_prior_motion():
    observer = object.__new__(BrneTask1CloseoutObserver)
    observer.facts = {
        'raw_count': 0, 'invalid_raw_count': 0, 'raw_linear_min': None,
        'raw_linear_max': None, 'raw_angular_min': None, 'raw_angular_max': None,
        'active_raw_count': 0,
    }
    observer.active_seen = False
    observer.last_goal_received = time.monotonic() - 0.6
    observer.stale_stop = None

    observer._raw(Twist())

    assert observer.stale_stop['raw_command'] == [0.0, 0.0]
    assert observer.stale_stop['active_raw_seen_before_stop'] is False
    assert observer.stale_stop['goal_silence_sec'] >= 0.5
