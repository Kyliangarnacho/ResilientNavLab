"""Static contracts for the minimal BRNE pedestrian interfaces."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CMAKE_PATH = PACKAGE_ROOT / 'CMakeLists.txt'
PACKAGE_XML_PATH = PACKAGE_ROOT / 'package.xml'


def _message_lines(name):
    """Return non-empty, non-comment message lines in source order."""
    return [
        line.strip()
        for line in (PACKAGE_ROOT / 'msg' / name).read_text(encoding='utf-8').splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    ]


def test_pedestrian_messages_match_the_pinned_brne_minimum_contract():
    """Keep only the exact upstream fields required by the shadow node."""
    assert _message_lines('Pedestrian.msg') == [
        'std_msgs/Header header',
        'int64 id',
        'geometry_msgs/Pose pose',
        'geometry_msgs/Twist velocity',
    ]
    assert _message_lines('PedestrianArray.msg') == [
        'std_msgs/Header header',
        'Pedestrian[] pedestrians',
    ]


def test_pedestrian_interfaces_are_registered_with_geometry_dependencies():
    """Require ROSIDL generation and the only added message dependency."""
    cmake_source = CMAKE_PATH.read_text(encoding='utf-8')
    root = ET.parse(PACKAGE_XML_PATH).getroot()

    assert '"msg/Pedestrian.msg"' in cmake_source
    assert '"msg/PedestrianArray.msg"' in cmake_source
    assert 'DEPENDENCIES builtin_interfaces geometry_msgs std_msgs' in cmake_source
    assert 'geometry_msgs' in {element.text for element in root.findall('depend')}
