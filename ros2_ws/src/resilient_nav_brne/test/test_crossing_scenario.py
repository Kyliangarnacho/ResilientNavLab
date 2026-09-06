"""Tests for the small fixed crossing state machine."""

from resilient_nav_brne.crossing_scenario import CrossingScenario


def test_crossing_waits_for_plan_then_moves_and_stops_at_measured_target():
    scenario = CrossingScenario(start_delay_sec=0.5)
    assert scenario.command(now_sec=1.0, plan_ready=False, world_y=-4.5) == 0.0
    assert scenario.command(now_sec=2.0, plan_ready=True, world_y=-4.5) == 0.0
    assert scenario.command(now_sec=2.4, plan_ready=True, world_y=-4.5) == 0.0
    assert scenario.command(now_sec=2.5, plan_ready=True, world_y=-4.5) == 0.25
    assert scenario.command(now_sec=5.0, plan_ready=True, world_y=-2.5) == 0.0
    assert scenario.complete


def test_missing_fresh_input_and_duration_timeout_always_command_stop():
    scenario = CrossingScenario(start_delay_sec=0.0, maximum_duration_sec=1.0)
    assert scenario.command(now_sec=1.0, plan_ready=True, world_y=-4.5) == 0.0
    assert scenario.command(now_sec=1.1, plan_ready=True, world_y=-4.5) == 0.25
    assert scenario.command(now_sec=1.2, plan_ready=False, world_y=-4.2) == 0.0
    assert scenario.command(now_sec=2.2, plan_ready=True, world_y=-4.2) == 0.0
    assert scenario.failed
