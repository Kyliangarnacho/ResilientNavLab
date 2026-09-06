"""Focused contracts for the unified interaction and event policy."""

from types import SimpleNamespace

import numpy as np

from resilient_nav_brne.brne_shadow_node import BrneShadowNode
from resilient_nav_brne.interaction_state import InteractionState


class _Logger:
    def info(self, _message):
        pass


def _node_for_interaction_events():
    node = object.__new__(BrneShadowNode)
    node.proposal_protection_window_outputs = 5
    node.proposal_protection_output_deadband = 0.05
    node.proposal_opposite_scale = 0.25
    node.proposal_opposite_threshold = 0.35
    node._interaction = InteractionState(
        separation_margin=0.20,
        history_size=4,
        entry_distance=3.20,
    )
    node._passing_side = 0
    node._passing_side_output_count = 0
    node._crossing_event_side = 0
    node._crossing_event_complete = False
    node._head_on_event_active = False
    node._head_on_event_complete = False
    node._head_on_path_origin = None
    node._head_on_path_direction = None
    node._head_on_path_clearance = None
    node.head_on_lateral_direction_deadband_rad = 0.17
    node.head_on_release_path_clearance = 0.58
    node.crossing_initial_direction_window_outputs = 5
    node.planner = SimpleNamespace(config=SimpleNamespace(
        max_angular_velocity=0.8,
        crossing_lateral_speed_threshold=0.08,
        crossing_minimum_lateral_alignment=0.50,
        head_on_minimum_approach_speed=0.12,
        head_on_maximum_direction_angle_rad=1.05,
        crossing_time_max=4.0,
        crossing_forward_min=0.20,
        crossing_forward_max=2.0,
    ))
    node.get_logger = lambda: _Logger()
    return node


def test_first_left_output_protects_four_reverse_nominals_then_releases_sixth():
    node = _node_for_interaction_events()
    pedestrian = [(8, [1.0, 0.0, 0.0, 0.0])]

    # Output 1 selects left; it is part of the five-output window.
    assert node._proposal_nominal_override(
        [0.0, 0.0, 0.0], [2.0, 0.0], pedestrian
    ) is None
    node._record_passing_side_direction(0.6)

    # Outputs 2--5 suppress a strong right nominal without forcing command sign.
    for _ in range(4):
        assert node._proposal_nominal_override(
            [0.0, 0.0, 0.0], [0.0, -2.0], pedestrian
        ) == -0.2
        node._record_passing_side_direction(-0.4)

    # Output 6 passes the same raw -0.8 nominal through normally.
    assert node._passing_side_output_count == 5
    assert node._proposal_nominal_override(
        [0.0, 0.0, 0.0], [0.0, -2.0], pedestrian
    ) is None


def test_first_right_passing_side_protection_is_symmetric():
    node = _node_for_interaction_events()
    pedestrian = [(8, [1.0, 0.0, 0.0, 0.0])]
    node._proposal_nominal_override(
        [0.0, 0.0, 0.0], [2.0, 0.0], pedestrian
    )
    node._record_passing_side_direction(-0.6)

    assert node._proposal_nominal_override(
        [0.0, 0.0, 0.0], [0.0, 2.0], pedestrian
    ) == 0.2


def test_proposal_protection_ignores_reverse_nominal_below_point_35():
    node = _node_for_interaction_events()
    pedestrian = [(8, [1.0, 0.0, 0.0, 0.0])]
    node._proposal_nominal_override(
        [0.0, 0.0, 0.0], [2.0, 0.0], pedestrian
    )
    node._record_passing_side_direction(0.6)

    # This goal produces about -0.30 rad/s, below the 0.35 trigger.
    assert node._proposal_nominal_override(
        [0.0, 0.0, 0.0], [1.0, -0.15], pedestrian
    ) is None


def test_crossing_event_sets_opposite_side_and_clears_at_centerline():
    node = _node_for_interaction_events()
    crossing = [(8, [1.0, -0.5, 0.0, 0.2])]

    node._proposal_nominal_override(
        [0.0, 0.0, 0.0], [2.0, 0.0], crossing
    )

    assert node._crossing_event_side == -1
    assert node._passing_side == -1
    assert node._initial_crossing_preferred_side() == -1

    node._proposal_nominal_override(
        [0.0, 0.0, 0.0],
        [2.0, 0.0],
        [(8, [1.0, 0.01, 0.0, 0.2])],
    )
    assert node._crossing_event_complete
    assert node._initial_crossing_preferred_side() == 0


def test_crossing_event_hard_direction_mask_is_limited_to_five_outputs():
    node = _node_for_interaction_events()
    crossing = [(8, [1.0, -0.5, 0.0, 0.2])]

    node._proposal_nominal_override(
        [0.0, 0.0, 0.0], [2.0, 0.0], crossing
    )
    for _ in range(5):
        assert node._initial_crossing_preferred_side() == -1
        node._record_passing_side_direction(-0.2)

    assert node._passing_side_output_count == 5
    assert node._initial_crossing_preferred_side() == 0
    assert node._passing_side == -1


def test_head_on_interaction_is_not_misclassified_as_crossing():
    node = _node_for_interaction_events()
    head_on = [(8, [1.0, 0.1, -0.25, 0.02])]

    node._proposal_nominal_override(
        [0.0, 0.0, 0.0], [2.0, 0.0], head_on
    )

    assert node._interaction.active
    assert node._crossing_event_side == 0
    assert node._initial_crossing_preferred_side() == 0
    assert node._head_on_event_active
    assert node._head_on_preferred_side() == 0


def test_oblique_head_on_velocity_anchors_the_opposite_robot_side_on_entry():
    node = _node_for_interaction_events()
    oblique_head_on = [(8, [1.0, 0.1, -0.25, 0.07])]

    node._proposal_nominal_override(
        [0.0, 0.0, 0.0], [2.0, 0.0], oblique_head_on
    )

    assert node._head_on_event_active
    assert node._crossing_event_side == 0
    assert node._head_on_preferred_side() == -1
    assert node._passing_side_output_count == 0


def test_oblique_head_on_direction_choice_is_left_right_symmetric():
    node = _node_for_interaction_events()

    node._proposal_nominal_override(
        [0.0, 0.0, 0.0],
        [2.0, 0.0],
        [(8, [1.0, -0.1, -0.25, -0.07])],
    )

    assert node._head_on_event_active
    assert node._head_on_preferred_side() == 1


def test_45_degree_approach_is_classified_as_head_on_only():
    node = _node_for_interaction_events()
    diagonal = [(8, [1.0, -0.5, -0.2, 0.2])]

    node._proposal_nominal_override(
        [0.0, 0.0, 0.0], [2.0, 0.0], diagonal
    )

    assert node._crossing_event_side == 0
    assert node._head_on_event_active
    assert node._head_on_preferred_side() == -1
    assert node._velocity_event_class(-0.2, 0.2) == 'head_on'


def test_near_lateral_approach_remains_crossing():
    node = _node_for_interaction_events()
    crossing = [(8, [1.0, -0.5, -0.086, 0.235])]

    node._proposal_nominal_override(
        [0.0, 0.0, 0.0], [2.0, 0.0], crossing
    )

    assert node._crossing_event_side == -1
    assert not node._head_on_event_active


def test_same_pedestrian_can_change_event_class_without_overlap():
    node = _node_for_interaction_events()

    node._proposal_nominal_override(
        [0.0, 0.0, 0.0],
        [2.0, 0.0],
        [(8, [1.0, -0.5, 0.0, 0.2])],
    )
    assert node._crossing_event_side == -1
    assert not node._head_on_event_active

    node._proposal_nominal_override(
        [0.0, 0.0, 0.0],
        [2.0, 0.0],
        [(8, [1.0, 0.01, 0.0, 0.2])],
    )
    assert node._crossing_event_complete

    node._proposal_nominal_override(
        [0.0, 0.0, 0.0],
        [2.0, 0.0],
        [(8, [1.0, 0.01, -0.25, 0.0])],
    )
    assert node._crossing_event_side == 0
    assert not node._crossing_event_complete
    assert node._head_on_event_active


def test_head_on_first_output_anchors_one_side_until_path_clearance():
    node = _node_for_interaction_events()
    head_on = [(8, [1.0, 0.0, -0.25, 0.0])]

    node._proposal_nominal_override(
        [0.0, 0.0, 0.0], [2.0, 0.0], head_on
    )
    node._record_passing_side_direction(0.4)
    assert node._head_on_preferred_side() == 1

    # Unlike the finite crossing window, head-on proposal support remains
    # protected after five outputs while the frozen CV path is not clear.
    node._passing_side_output_count = 5
    assert node._proposal_nominal_override(
        [0.0, 0.0, 0.0], [0.0, -2.0], head_on
    ) == -0.2

    node._update_head_on_release([0.0, 0.57, 0.0])
    assert node._head_on_event_active
    node._update_head_on_release([0.0, 0.59, 0.0])
    assert not node._head_on_event_active
    assert node._head_on_event_complete
    assert node._head_on_preferred_side() == 0

    node._record_passing_side_direction(-0.4)
    assert node._passing_side == 0
    assert node._passing_side_output_count == node.proposal_protection_window_outputs


def test_diagonal_head_on_release_uses_orientation_independent_line_clearance():
    node = _node_for_interaction_events()
    inverse_sqrt_two = 1.0 / np.sqrt(2.0)
    node._head_on_event_active = True
    node._head_on_path_origin = np.array([0.0, 0.0])
    node._head_on_path_direction = np.array([
        inverse_sqrt_two, inverse_sqrt_two
    ])
    node._passing_side = 1

    node._update_head_on_release([
        0.57 * inverse_sqrt_two,
        -0.57 * inverse_sqrt_two,
        0.0,
    ])
    assert node._head_on_event_active

    node._update_head_on_release([
        0.59 * inverse_sqrt_two,
        -0.59 * inverse_sqrt_two,
        0.0,
    ])
    assert not node._head_on_event_active
    assert node._head_on_event_complete
