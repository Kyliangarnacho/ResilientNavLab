"""Static resource checks for the C920 baseline launch."""

from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_FILE = PACKAGE_DIR / 'config' / 'c920.yaml'
CAMERA_INFO_FILE = PACKAGE_DIR / 'config' / 'c920_camera_info.yaml'
LAUNCH_FILE = PACKAGE_DIR / 'launch' / 'c920.launch.py'
SETUP_FILE = PACKAGE_DIR / 'setup.py'


def test_c920_config_declares_only_the_verified_capture_baseline():
    """The capture configuration keeps verified device and transport settings."""
    config_source = CONFIG_FILE.read_text(encoding='utf-8')

    for setting in [
        'video_device: "/dev/video0"',
        'framerate: 15.0',
        'io_method: "mmap"',
        'pixel_format: "mjpeg2rgb"',
        'image_width: 1280',
        'image_height: 720',
        'frame_id: "c920_camera_optical_frame"',
    ]:
        assert setting in config_source
    assert 'camera_name: "c920"' in config_source
    assert (
        'camera_info_url: '
        '"package://resilient_nav_camera/config/c920_camera_info.yaml"'
    ) in config_source


def test_c920_camera_info_uses_the_verified_legacy_pinhole_values():
    """The ROS CameraInfo resource preserves the accepted old K/D exactly."""
    camera_info_source = CAMERA_INFO_FILE.read_text(encoding='utf-8')

    for setting in [
        'image_width: 1280',
        'image_height: 720',
        'camera_name: c920',
        'distortion_model: plumb_bob',
        'rows: 1',
        'cols: 5',
        '932.82988067562258',
        '931.57268412771748',
        '637.24258737387333',
        '373.17256990919134',
        '0.022262962552999279',
        '-0.038141806307307465',
        '0.002411186067623861',
        '-0.0038095868601148979',
        '-0.20962046543174492',
    ]:
        assert setting in camera_info_source
    assert 'rectification_matrix:' in camera_info_source
    assert 'projection_matrix:' in camera_info_source


def test_launch_places_usb_cam_in_the_c920_namespace_and_starts_probe():
    """The launch resolves the requested image topic by using a namespace."""
    launch_source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert "package='usb_cam'" in launch_source
    assert "executable='usb_cam_node_exe'" in launch_source
    assert "namespace='/camera/c920'" in launch_source
    assert "executable='c920_probe'" in launch_source
    assert 'DeclareLaunchArgument(' in launch_source
    assert "'camera_info_url'" in launch_source
    expected_parameters = (
        'parameters=[config_file, {' + repr('camera_info_url')
        + ': camera_info_url}]'
    )
    assert expected_parameters in launch_source
    assert "'enable_rectification'" in launch_source
    assert "package='image_proc'" in launch_source
    assert "executable='rectify_node'" in launch_source
    assert "remappings=[('image', 'image_raw')]" in launch_source
    assert "executable='rectification_probe'" in launch_source


def test_setup_installs_c920_launch_and_configuration_resources():
    """The package installs its launch and YAML resources."""
    setup_source = SETUP_FILE.read_text(encoding='utf-8')

    assert "glob(os.path.join('launch', '*.launch.py'))" in setup_source
    assert "glob(os.path.join('config', '*.yaml'))" in setup_source


def test_setup_registers_the_temporary_calibration_reuse_validator():
    """The validation-only executable is available through ros2 run."""
    setup_source = SETUP_FILE.read_text(encoding='utf-8')

    assert 'calibration_reuse_validator = ' in setup_source
    assert 'resilient_nav_camera.calibration_reuse_validator:main' in setup_source
