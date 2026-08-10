"""Resource and isolation checks for manual_fault_event."""

from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SETUP_FILE = PACKAGE_DIR / 'setup.py'
SOURCE_FILE = (
    PACKAGE_DIR
    / 'resilient_nav_fault_injection'
    / 'manual_fault_event.py'
)


def test_manual_fault_event_is_installed():
    setup_source = SETUP_FILE.read_text(encoding='utf-8')

    assert "'manual_fault_event = '" in setup_source
    assert 'manual_fault_event:main' in setup_source


def test_manual_event_only_publishes_fault_status():
    source = SOURCE_FILE.read_text(encoding='utf-8')

    assert 'create_publisher(' in source
    assert 'create_subscription(' not in source
    assert 'Image' not in source
    assert 'SensorHealth' not in source
    assert "DEFAULT_STATUS_TOPIC = '/fault_injection/status'" in source
