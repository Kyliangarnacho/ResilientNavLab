"""Tests for the synthetic camera health feature demonstration."""

import pytest

from resilient_nav_health_assessment.camera_health_feature_demo import (
    build_demo_scenarios,
    evaluate_demo_scenarios,
    format_demo_table,
    main,
)


EXPECTED_SCENARIOS = [
    'pure_black',
    'pure_white',
    'uniform_gray',
    'gray_gradient',
    'checkerboard',
    'checkerboard_blur',
    'repeated_frame',
    'slight_change',
]


@pytest.fixture(scope='module')
def results_by_name():
    """Evaluate the deterministic scenes once for qualitative assertions."""
    return {
        result['scenario']: result
        for result in evaluate_demo_scenarios()
    }


def test_demo_constructs_all_requested_scenes():
    scenarios = build_demo_scenarios()

    assert [scenario.name for scenario in scenarios] == EXPECTED_SCENARIOS
    assert all(scenario.image.shape == (64, 64) for scenario in scenarios)


def test_black_has_high_dark_ratio(results_by_name):
    assert results_by_name['pure_black']['dark_ratio'] == 1.0


def test_white_has_high_bright_ratio(results_by_name):
    assert results_by_name['pure_white']['bright_ratio'] == 1.0


def test_sharp_texture_has_more_laplacian_variance_than_blur(
    results_by_name,
):
    sharp = results_by_name['checkerboard']
    blurred = results_by_name['checkerboard_blur']

    assert sharp['laplacian_variance'] > blurred['laplacian_variance']


def test_low_texture_is_not_treated_as_identical_to_blur(results_by_name):
    uniform = results_by_name['uniform_gray']
    blurred = results_by_name['checkerboard_blur']

    assert uniform['gray_std'] == 0.0
    assert uniform['laplacian_variance'] == 0.0
    assert blurred['gray_std'] > uniform['gray_std']
    assert blurred['laplacian_variance'] > uniform['laplacian_variance']
    assert blurred['entropy'] > uniform['entropy']


def test_repeated_frame_has_equal_fingerprint(results_by_name):
    repeated = results_by_name['repeated_frame']

    assert repeated['frame_diff_mean'] == 0.0
    assert repeated['fingerprint_same'] is True


def test_slight_change_has_different_fingerprint(results_by_name):
    changed = results_by_name['slight_change']

    assert 0.0 < changed['frame_diff_mean'] < 1.0
    assert changed['fingerprint_same'] is False


def test_demo_table_contains_requested_features_without_health_labels(
    results_by_name,
):
    table = format_demo_table(list(results_by_name.values()))

    for heading in [
        'mean_gray',
        'gray_std',
        'dark_ratio',
        'bright_ratio',
        'laplacian_variance',
        'edge_density',
        'entropy',
        'frame_diff_mean',
        'fingerprint_same',
    ]:
        assert heading in table
    assert 'HEALTHY' not in table
    assert 'FAULT' not in table
    assert len(table.splitlines()) == len(EXPECTED_SCENARIOS) + 2


def test_main_prints_the_complete_table(capsys):
    main()

    output = capsys.readouterr().out
    for scenario in EXPECTED_SCENARIOS:
        assert scenario in output


def test_demo_rejects_images_too_small_for_the_fixed_scenes():
    with pytest.raises(ValueError, match='at least 16'):
        build_demo_scenarios(size=15)
