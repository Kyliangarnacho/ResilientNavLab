"""Architecture dependency checks for the pure Robot Domain."""

import ast
import os
from pathlib import Path


FORBIDDEN_IMPORTS = {
    'rclpy',
    'sensor_msgs',
    'nav_msgs',
    'geometry_msgs',
    'diagnostic_msgs',
}
PURE_DOMAIN_FILES = {
    'resilient_nav_agent/schemas.py',
    'resilient_nav_agent/sanitizer.py',
    'resilient_nav_agent/incidents.py',
    'resilient_nav_agent/evidence.py',
    'resilient_nav_agent/extension.py',
    'resilient_nav_agent/offline/schemas.py',
    'resilient_nav_agent/offline/case_builder.py',
    'resilient_nav_agent/offline/context.py',
    'resilient_nav_agent/offline/fixtures.py',
    'resilient_nav_agent/offline/runtime.py',
    'resilient_nav_agent/benchmark/schemas.py',
    'resilient_nav_agent/benchmark/scorer.py',
    'resilient_nav_agent/benchmark/runner.py',
    'resilient_nav_agent/tools.py',
}


def package_root():
    """Return the package source directory."""
    return Path(__file__).resolve().parents[1]


def repository_root():
    """Find the repository root without embedding an absolute path."""
    for candidate in Path(__file__).resolve().parents:
        if (candidate / '.git').is_dir():
            return candidate
    raise AssertionError('repository root not found')


def imported_roots(path):
    """Return top-level module names imported by one Python source file."""
    tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.split('.')[0])
    return roots


def test_pure_domain_modules_do_not_import_ros_transport():
    """ROS types must remain outside all RA-1A pure Domain modules."""
    source = package_root()
    violations = {}
    for filename in PURE_DOMAIN_FILES:
        forbidden = imported_roots(source / filename).intersection(
            FORBIDDEN_IMPORTS
        )
        if forbidden:
            violations[filename] = sorted(forbidden)

    assert violations == {}


def test_repository_contains_no_vendored_agent_core_source():
    """No non-ignored agent_core directory may exist inside this repository."""
    root = repository_root()
    ignored = {'.git', '.venv', 'build', 'install', 'log', '__pycache__'}
    candidates = []
    for current, directories, _files in os.walk(root):
        directories[:] = [name for name in directories if name not in ignored]
        if 'agent_core' in directories:
            candidates.append(str(Path(current) / 'agent_core'))

    assert candidates == []
