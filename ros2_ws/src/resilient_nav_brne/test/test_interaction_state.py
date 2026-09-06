"""Regression contracts for sensor Scene 1 global-path retention."""

from resilient_nav_brne.interaction_state import InteractionState


def test_confirmed_track_acquires_and_release_requires_gain_and_trend():
    state = InteractionState(separation_margin=0.20, history_size=4)

    entered = state.update((0.0, 0.0), {7: (1.0, 0.0)})
    assert entered.entered_pedestrian_id == 7
    assert state.active

    # Neither a new closest point nor separation gain without a full trend
    # window is enough to release the retained path.
    for distance in (0.80, 0.70, 0.82):
        transition = state.update((0.0, 0.0), {7: (distance, 0.0)})
        assert transition.released_pedestrian_id is None
        assert state.active

    released = state.update((0.0, 0.0), {7: (0.92, 0.0)})
    assert released.released_pedestrian_id == 7
    assert released.reason == 'separation gain and distance trend clear'
    assert not state.active


def test_released_track_is_not_immediately_reacquired_while_still_visible():
    state = InteractionState(separation_margin=0.20, history_size=4)
    state.update((0.0, 0.0), {3: (1.0, 0.0)})
    for distance in (0.70, 0.75, 0.82, 0.95):
        transition = state.update((0.0, 0.0), {3: (distance, 0.0)})
    assert transition.released_pedestrian_id == 3

    assert state.update((0.0, 0.0), {3: (1.0, 0.0)}).entered_pedestrian_id is None
    assert not state.active

    # Track loss ends that interaction lifecycle. A later sensor track can be
    # acquired normally (the V1 tracker itself assigns a fresh ID).
    state.update((0.0, 0.0), {})
    entered = state.update((0.0, 0.0), {4: (1.0, 0.0)})
    assert entered.entered_pedestrian_id == 4


def test_missing_owner_releases_and_nearest_confirmed_track_is_selected():
    state = InteractionState()
    entered = state.update((0.0, 0.0), {5: (1.2, 0.0), 6: (0.8, 0.0)})
    assert entered.entered_pedestrian_id == 6

    released = state.update((0.0, 0.0), {5: (1.1, 0.0)})
    assert released.released_pedestrian_id == 6
    assert released.reason == 'tracked pedestrian disappeared'
    assert not state.active


def test_interaction_does_not_acquire_outside_entry_distance():
    state = InteractionState(
        separation_margin=0.20,
        history_size=4,
        entry_distance=2.20,
    )

    assert state.update((0.0, 0.0), {7: (2.21, 0.0)}).entered_pedestrian_id is None
    assert not state.active
    entered = state.update((0.0, 0.0), {7: (2.19, 0.0)})
    assert entered.entered_pedestrian_id == 7
    assert state.active
