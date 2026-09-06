"""Static contract tests for the Phase 8 fusion-runtime status message."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CMAKE_PATH = PACKAGE_ROOT / 'CMakeLists.txt'
PACKAGE_XML_PATH = PACKAGE_ROOT / 'package.xml'
MESSAGE_PATH = PACKAGE_ROOT / 'msg' / 'FusionStatus.msg'


def message_lines():
    """Return non-empty, non-comment FusionStatus lines in source order."""
    return [
        line.strip()
        for line in MESSAGE_PATH.read_text(encoding='utf-8').splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    ]


def test_fusion_status_is_registered_for_rosidl_generation():
    """The interface package must generate and export the new message."""
    cmake_source = CMAKE_PATH.read_text(encoding='utf-8')
    root = ET.parse(PACKAGE_XML_PATH).getroot()

    assert '"msg/FusionStatus.msg"' in cmake_source
    assert 'DEPENDENCIES builtin_interfaces geometry_msgs std_msgs' in cmake_source
    assert 'ament_export_dependencies(rosidl_default_runtime)' in cmake_source
    assert root.findtext('name') == 'resilient_nav_interfaces'
    assert {'builtin_interfaces', 'geometry_msgs', 'std_msgs'} <= {
        element.text for element in root.findall('depend')
    }
    assert 'rosidl_default_generators' in {
        element.text for element in root.findall('build_depend')
    }
    assert 'rosidl_default_runtime' in {
        element.text for element in root.findall('exec_depend')
    }


def test_fusion_status_contains_only_runtime_observation_fields():
    """Fusion status must expose decisions, not injected-fault ground truth."""
    assert message_lines() == [
        'std_msgs/Header header',
        'string chain_id',
        'uint8 state',
        'string[] accepted_measurements',
        'string[] rejected_measurements',
        'string[] reasons',
        'float32 confidence',
        'builtin_interfaces/Time window_start',
        'builtin_interfaces/Time window_end',
        'uint32 sample_count',
        'uint8 UNKNOWN=0',
        'uint8 NOMINAL=1',
        'uint8 DEGRADED=2',
        'uint8 HOLD=3',
    ]

    forbidden_tokens = (
        'faultstatus',
        'fault_status',
        'scenario',
        'seed',
        'model',
        'parameters',
        'ground_truth',
        'truth',
    )
    normalized = MESSAGE_PATH.read_text(encoding='utf-8').lower()
    assert not any(token in normalized for token in forbidden_tokens)
