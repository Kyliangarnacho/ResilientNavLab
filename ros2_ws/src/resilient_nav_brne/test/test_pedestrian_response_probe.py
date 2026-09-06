from resilient_nav_brne.brne_pedestrian_response_probe import run_probe


def test_fixed_seed_pedestrian_response_is_repeatable_and_bounded():
    first = run_probe()
    second = run_probe()

    assert first == second
    for case in first["cases"].values():
        assert 0.0 <= case["linear_x"] <= 0.3
        assert -0.8 <= case["angular_z"] <= 0.8
