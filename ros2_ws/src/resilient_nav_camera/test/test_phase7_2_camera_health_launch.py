"""Static checks for the phase 7.2 combined camera health launch."""

from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_DIR / 'launch' / 'phase7_2_camera_health.launch.py'
PACKAGE_FILE = PACKAGE_DIR / 'package.xml'
ORIGINAL_LAUNCH = PACKAGE_DIR / 'launch' / 'c920.launch.py'


def test_combined_launch_declares_required_arguments_and_defaults():
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    for argument in (
        'enable_rectification',
        'health_config',
        'source_topic',
        'run_evaluator',
        'evaluator_output_json',
        'run_watch',
        'run_image_view',
        'use_sim_time',
    ):
        assert f"'{argument}'" in source
    assert "default_value='/camera/c920/image_raw'" in source
    assert "'run_camera',\n            default_value='true'" in source
    assert "'run_evaluator',\n            default_value='false'" in source
    assert "'run_watch',\n            default_value='false'" in source
    assert "'run_image_view',\n            default_value='false'" in source


def test_combined_launch_reuses_c920_and_passes_health_parameters():
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert "'c920.launch.py'" in source
    assert "'enable_rectification': enable_rectification" in source
    assert "executable='camera_health_monitor'" in source
    assert "'image_topic': source_topic" in source
    assert "'health_topic': '/health/camera'" in source
    assert "'camera_health_topic': '/health/camera'" in source
    assert "'output_json_path': evaluator_output_json" in source
    assert "'use_sim_time': use_sim_time" in source


def test_optional_nodes_have_conditions_and_manual_event_is_not_included():
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert 'condition=IfCondition(run_evaluator)' in source
    assert 'condition=IfCondition(run_watch)' in source
    assert 'condition=IfCondition(run_image_view)' in source
    assert "executable='health_evaluator'" in source
    assert "executable='camera_health_watch'" in source
    assert "package='rqt_image_view'" in source
    assert "executable='rqt_image_view'" in source
    assert 'arguments=[source_topic]' in source
    assert "remappings=[('image', source_topic)]" in source
    assert 'manual_fault_event' not in source


def test_camera_package_declares_health_runtime_dependency():
    package_source = PACKAGE_FILE.read_text(encoding='utf-8')

    assert (
        '<exec_depend>resilient_nav_health_assessment</exec_depend>'
        in package_source
    )
    assert '<exec_depend>rqt_image_view</exec_depend>' in package_source


def test_original_phase7_1_launch_contract_remains_present():
    source = ORIGINAL_LAUNCH.read_text(encoding='utf-8')

    assert "package='usb_cam'" in source
    assert "namespace='/camera/c920'" in source
    assert "parameters=[config_file, {'camera_info_url': camera_info_url}]" in source
    assert 'condition=IfCondition(enable_rectification)' in source
    assert 'phase7_2_camera_health' not in source
