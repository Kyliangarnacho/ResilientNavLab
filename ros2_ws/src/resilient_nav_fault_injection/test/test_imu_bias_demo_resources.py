import importlib.util
from pathlib import Path

import pytest
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
BIAS_SCENARIO_FILE = (
    PACKAGE_DIR / 'config' / 'scenarios' / 'imu_bias_demo.yaml'
)
NOISE_SCENARIO_FILE = (
    PACKAGE_DIR / 'config' / 'scenarios' / 'imu_noise_demo.yaml'
)
DROPOUT_SCENARIO_FILE = (
    PACKAGE_DIR / 'config' / 'scenarios' / 'imu_dropout_demo.yaml'
)
DELAY_SCENARIO_FILE = (
    PACKAGE_DIR / 'config' / 'scenarios' / 'imu_delay_demo.yaml'
)
WHEEL_FREEZE_SCENARIO_FILE = (
    PACKAGE_DIR / 'config' / 'scenarios' / 'wheel_freeze_demo.yaml'
)
SCAN_SECTOR_BLINDNESS_SCENARIO_FILE = (
    PACKAGE_DIR / 'config' / 'scenarios' / 'scan_sector_blindness_demo.yaml'
)
BIAS_LAUNCH_FILE = PACKAGE_DIR / 'launch' / 'imu_bias_demo.launch.py'
NOISE_LAUNCH_FILE = PACKAGE_DIR / 'launch' / 'imu_noise_demo.launch.py'
DROPOUT_LAUNCH_FILE = PACKAGE_DIR / 'launch' / 'imu_dropout_demo.launch.py'
DELAY_LAUNCH_FILE = PACKAGE_DIR / 'launch' / 'imu_delay_demo.launch.py'
WHEEL_FREEZE_LAUNCH_FILE = (
    PACKAGE_DIR / 'launch' / 'wheel_freeze_demo.launch.py'
)
SCAN_SECTOR_BLINDNESS_LAUNCH_FILE = (
    PACKAGE_DIR / 'launch' / 'scan_sector_blindness_demo.launch.py'
)
SETUP_FILE = PACKAGE_DIR / 'setup.py'

REQUIRED_BIAS_PARAMETERS = {
    'use_sim_time': True,
    'input_topic': '/imu/data',
    'output_topic': '/faulted/imu/data',
    'status_topic': '/fault_injection/status',
    'enabled': True,
    'model': 'bias',
    'bias_rad_s': 0.15,
    'start_time_sec': 5.0,
    'end_time_sec': 15.0,
    'scenario_id': 'imu_bias_demo',
    'scenario_seed': 20260803,
    'event_id': 'imu_bias_001',
}

REQUIRED_NOISE_PARAMETERS = {
    'use_sim_time': True,
    'input_topic': '/imu/data',
    'output_topic': '/faulted/imu/data',
    'status_topic': '/fault_injection/status',
    'enabled': True,
    'model': 'gaussian_noise',
    'noise_sigma_rad_s': 0.05,
    'start_time_sec': 5.0,
    'end_time_sec': 15.0,
    'scenario_id': 'imu_noise_demo',
    'scenario_seed': 20260803,
    'event_id': 'imu_noise_001',
}

REQUIRED_DROPOUT_PARAMETERS = {
    'use_sim_time': True,
    'input_topic': '/imu/data',
    'output_topic': '/faulted/imu/data',
    'status_topic': '/fault_injection/status',
    'enabled': True,
    'model': 'dropout',
    'dropout_probability': 0.30,
    'start_time_sec': 5.0,
    'end_time_sec': 15.0,
    'scenario_id': 'imu_dropout_demo',
    'scenario_seed': 20260803,
    'event_id': 'imu_dropout_001',
}

REQUIRED_DELAY_PARAMETERS = {
    'use_sim_time': True,
    'input_topic': '/imu/data',
    'output_topic': '/faulted/imu/data',
    'status_topic': '/fault_injection/status',
    'enabled': True,
    'model': 'fixed_delay',
    'delay_sec': 0.50,
    'start_time_sec': 5.0,
    'end_time_sec': 15.0,
    'scenario_id': 'imu_delay_demo',
    'scenario_seed': 20260803,
    'event_id': 'imu_delay_001',
}

REQUIRED_WHEEL_FREEZE_PARAMETERS = {
    'use_sim_time': True,
    'input_topic': '/wheel/odometry',
    'output_topic': '/faulted/wheel/odometry',
    'status_topic': '/fault_injection/status',
    'enabled': True,
    'model': 'freeze',
    'start_time_sec': 5.0,
    'end_time_sec': 15.0,
    'scenario_id': 'wheel_freeze_demo',
    'scenario_seed': 20260803,
    'event_id': 'wheel_freeze_001',
}

REQUIRED_SCAN_SECTOR_BLINDNESS_PARAMETERS = {
    'use_sim_time': True,
    'input_topic': '/scan',
    'output_topic': '/faulted/scan',
    'status_topic': '/fault_injection/status',
    'enabled': True,
    'model': 'sector_blindness',
    'sector_center_rad': 0.0,
    'sector_width_rad': 1.0,
    'start_time_sec': 5.0,
    'end_time_sec': 15.0,
    'scenario_id': 'scan_sector_blindness_demo',
    'scenario_seed': 20260803,
    'event_id': 'scan_sector_blindness_001',
}


@pytest.fixture
def bias_scenario_parameters():
    with BIAS_SCENARIO_FILE.open('r', encoding='utf-8') as scenario_stream:
        scenario = yaml.safe_load(scenario_stream)

    return scenario['imu_bias_injector']['ros__parameters']


@pytest.fixture
def noise_scenario_parameters():
    with NOISE_SCENARIO_FILE.open('r', encoding='utf-8') as scenario_stream:
        scenario = yaml.safe_load(scenario_stream)

    return scenario['imu_fault_injector']['ros__parameters']


@pytest.fixture
def dropout_scenario_parameters():
    with DROPOUT_SCENARIO_FILE.open('r', encoding='utf-8') as scenario_stream:
        scenario = yaml.safe_load(scenario_stream)

    return scenario['imu_fault_injector']['ros__parameters']


@pytest.fixture
def delay_scenario_parameters():
    with DELAY_SCENARIO_FILE.open('r', encoding='utf-8') as scenario_stream:
        scenario = yaml.safe_load(scenario_stream)

    return scenario['imu_fault_injector']['ros__parameters']


@pytest.fixture
def wheel_freeze_scenario_parameters():
    with WHEEL_FREEZE_SCENARIO_FILE.open(
        'r',
        encoding='utf-8',
    ) as scenario_stream:
        scenario = yaml.safe_load(scenario_stream)

    return scenario['wheel_fault_injector']['ros__parameters']


@pytest.fixture
def scan_sector_blindness_scenario_parameters():
    with SCAN_SECTOR_BLINDNESS_SCENARIO_FILE.open(
        'r',
        encoding='utf-8',
    ) as scenario_stream:
        scenario = yaml.safe_load(scenario_stream)

    return scenario['scan_fault_injector']['ros__parameters']


def test_imu_bias_demo_yaml_is_parseable_and_complete(
    bias_scenario_parameters,
):
    assert bias_scenario_parameters == REQUIRED_BIAS_PARAMETERS


def test_imu_noise_demo_yaml_is_parseable_and_complete(
    noise_scenario_parameters,
):
    assert noise_scenario_parameters == REQUIRED_NOISE_PARAMETERS


def test_imu_dropout_demo_yaml_is_parseable_and_complete(
    dropout_scenario_parameters,
):
    assert dropout_scenario_parameters == REQUIRED_DROPOUT_PARAMETERS


def test_imu_delay_demo_yaml_is_parseable_and_complete(
    delay_scenario_parameters,
):
    assert delay_scenario_parameters == REQUIRED_DELAY_PARAMETERS


def test_wheel_freeze_demo_yaml_is_parseable_and_complete(
    wheel_freeze_scenario_parameters,
):
    assert wheel_freeze_scenario_parameters == REQUIRED_WHEEL_FREEZE_PARAMETERS


def test_scan_sector_blindness_demo_yaml_is_parseable_and_complete(
    scan_sector_blindness_scenario_parameters,
):
    assert (
        scan_sector_blindness_scenario_parameters
        == REQUIRED_SCAN_SECTOR_BLINDNESS_PARAMETERS
    )


@pytest.mark.parametrize(
    'scenario_parameters_fixture',
    [
        'bias_scenario_parameters',
        'noise_scenario_parameters',
        'dropout_scenario_parameters',
        'delay_scenario_parameters',
        'wheel_freeze_scenario_parameters',
        'scan_sector_blindness_scenario_parameters',
    ],
)
def test_fault_injection_demo_time_windows_are_valid(
    request,
    scenario_parameters_fixture,
):
    scenario_parameters = request.getfixturevalue(scenario_parameters_fixture)

    assert scenario_parameters['start_time_sec'] >= 0.0
    assert scenario_parameters['end_time_sec'] > scenario_parameters[
        'start_time_sec'
    ]


@pytest.mark.parametrize(
    'scenario_parameters_fixture',
    [
        'bias_scenario_parameters',
        'noise_scenario_parameters',
        'dropout_scenario_parameters',
        'delay_scenario_parameters',
        'wheel_freeze_scenario_parameters',
        'scan_sector_blindness_scenario_parameters',
    ],
)
def test_fault_injection_demo_input_and_output_topics_are_different(
    request,
    scenario_parameters_fixture,
):
    scenario_parameters = request.getfixturevalue(scenario_parameters_fixture)

    assert scenario_parameters['input_topic'] != scenario_parameters[
        'output_topic'
    ]


@pytest.mark.parametrize(
    ('module_name', 'launch_file'),
    [
        ('imu_bias_demo_launch', BIAS_LAUNCH_FILE),
        ('imu_noise_demo_launch', NOISE_LAUNCH_FILE),
        ('imu_dropout_demo_launch', DROPOUT_LAUNCH_FILE),
        ('imu_delay_demo_launch', DELAY_LAUNCH_FILE),
        ('wheel_freeze_demo_launch', WHEEL_FREEZE_LAUNCH_FILE),
        (
            'scan_sector_blindness_demo_launch',
            SCAN_SECTOR_BLINDNESS_LAUNCH_FILE,
        ),
    ],
)
def test_fault_injection_demo_launch_files_can_be_imported(
    module_name,
    launch_file,
):
    spec = importlib.util.spec_from_file_location(
        module_name,
        launch_file,
    )
    module = importlib.util.module_from_spec(spec)

    spec.loader.exec_module(module)

    assert hasattr(module, 'generate_launch_description')


def test_imu_delay_demo_launch_declares_use_sim_time_argument():
    launch_source = DELAY_LAUNCH_FILE.read_text(encoding='utf-8')

    assert "'use_sim_time'" in launch_source
    assert "default_value='true'" in launch_source
    assert 'ParameterValue' in launch_source


def test_setup_installs_launch_and_scenario_yaml_files():
    setup_source = SETUP_FILE.read_text(encoding='utf-8')

    assert "os.path.join('share', package_name, 'launch')" in setup_source
    assert "glob(os.path.join('launch', '*.launch.py'))" in setup_source
    assert (
        "os.path.join('share', package_name, 'config', 'scenarios')"
        in setup_source
    )
    assert (
        "glob(os.path.join('config', 'scenarios', '*.yaml'))"
        in setup_source
    )
    assert 'imu_fault_injector = ' in setup_source
    assert 'imu_bias_injector = ' in setup_source
    assert 'wheel_fault_injector = ' in setup_source
    assert 'scan_fault_injector = ' in setup_source
