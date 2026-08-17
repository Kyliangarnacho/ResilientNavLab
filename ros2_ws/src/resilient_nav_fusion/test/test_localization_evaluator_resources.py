"""Static isolation checks for the ROS evaluation boundary."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_evaluator_reads_only_truth_fixed_adaptive_and_publishes_evaluation_output():
    """Truth is consumed only by the evaluator and never by the runtime chain."""
    source = (
        PACKAGE_ROOT / 'resilient_nav_fusion' / 'localization_evaluator_node.py'
    ).read_text(encoding='utf-8').lower()

    for required in (
        '/evaluation/ground_truth_pose',
        '/odometry/faulted',
        '/odometry/adaptive',
        '/evaluation/localization_metrics',
    ):
        assert required in source
    for forbidden in (
        '/faulted/',
        'faultstatus',
        'scenario_id',
        'scenario_seed',
        'parameters_yaml',
        'cmd_vel',
        'fusionpolicy',
        'measurement_adapter',
        'robot_localization',
    ):
        assert forbidden not in source


def test_runtime_estimation_resources_never_consume_evaluation_topics():
    """The evaluation output cannot flow back into health, fusion, or EKF."""
    for relative_path in (
        'resilient_nav_fusion/fusion_policy.py',
        'resilient_nav_fusion/measurement_adapter.py',
        'config/adaptive_ekf.yaml',
        'launch/phase8_adaptive_ekf.launch.py',
    ):
        source = (PACKAGE_ROOT / relative_path).read_text(encoding='utf-8').lower()
        assert '/evaluation/' not in source
