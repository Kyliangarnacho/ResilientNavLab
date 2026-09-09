"""Static tests for the five Phase 9 physical-disturbance scene variants."""

import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BASE_WORLD = PACKAGE_ROOT / 'worlds' / 'phase9_slam_world.sdf'
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'physical_disturbance_scene.launch.py'
PULSE_FILE = PACKAGE_ROOT / 'scripts' / 'physical_disturbance_pulse.py'
ROUTE_FILE = PACKAGE_ROOT / 'scripts' / 'physical_disturbance_route.py'


def load_scene_module():
    """Load the launch-local pure world generator for focused tests."""
    spec = importlib.util.spec_from_file_location('physical_scene_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_route_module():
    scripts = str(ROUTE_FILE.parent)
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    spec = importlib.util.spec_from_file_location('physical_route', ROUTE_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generated_world(tmp_path, scenario, **overrides):
    module = load_scene_module()
    config = module.PhysicalSceneConfig(scenario=scenario, **overrides)
    output = tmp_path / f'{scenario}.sdf'
    module.build_physical_world(BASE_WORLD, output, config)
    return ET.parse(output).getroot(), module


@pytest.mark.parametrize(
    'scenario',
    [
        'normal',
        'low_friction',
        'asymmetric_traction',
        'rough_surface',
        'external_impact',
        'wheel_block',
        'navigation_gauntlet',
    ],
)
def test_every_scene_reuses_all_phase9_models(tmp_path, scenario):
    generated, _ = generated_world(tmp_path, scenario)
    baseline = ET.parse(BASE_WORLD).getroot()
    baseline_names = {
        model.attrib['name'] for model in baseline.findall('world/model')
    }
    generated_names = {
        model.attrib['name'] for model in generated.findall('world/model')
    }

    assert baseline_names.issubset(generated_names)
    assert generated.find("world[@name='resilient_lab']") is not None


def test_low_friction_and_asymmetric_traction_are_distinct_and_tunable(tmp_path):
    low, _ = generated_world(tmp_path, 'low_friction', low_friction_mu=0.18)
    asymmetric, module = generated_world(
        tmp_path,
        'asymmetric_traction',
        asymmetric_low_mu=0.30,
        weak_wheel_side='right',
    )

    low_model = low.find("world/model[@name='physical_low_friction_zone']")
    strip = asymmetric.find("world/model[@name='physical_right_traction_strip']")
    assert low_model.findtext('link/collision/surface/friction/ode/mu') == '0.18'
    assert float(strip.findtext('pose').split()[1]) == pytest.approx(
        -3.5 - module.WHEEL_TRACK_OFFSET_M
    )
    assert strip.findtext('link/collision/surface/friction/ode/mu') == '0.3'
    assert strip.findtext('link/collision/geometry/box/size').split()[1] == '0.2'


def test_every_scene_has_same_off_route_lidar_landmarks(tmp_path):
    """Controlled comparison shares geometry with enough nearby asymmetry."""
    expected = {
        'physical_lidar_near_cylinder_west',
        'physical_lidar_near_box_south',
        'physical_lidar_near_cylinder_east',
    }
    for scenario in ('normal', 'low_friction', 'rough_surface', 'wheel_block'):
        root, _ = generated_world(tmp_path, scenario)
        names = {model.attrib['name'] for model in root.findall('world/model')}
        assert expected.issubset(names)


def test_rough_surface_has_five_shallow_rounded_parameterized_bumps(tmp_path):
    root, _ = generated_world(
        tmp_path,
        'rough_surface',
        roughness_height_m=0.01,
        roughness_spacing_m=0.4,
    )
    bars = [
        model
        for model in root.findall('world/model')
        if model.find("link/collision[@name='bump_collision']") is not None
    ]

    assert len(bars) == 5
    assert all(
        bar.find('link/collision/geometry/cylinder') is not None
        for bar in bars
    )
    assert all(float(bar.findtext('pose').split()[2]) < 0.0 for bar in bars)


def test_large_rough_zone_is_filled_across_its_length(tmp_path):
    root, _ = generated_world(
        tmp_path,
        'rough_surface',
        zone_length_m=4.4,
        zone_width_m=2.2,
        roughness_spacing_m=0.4,
    )
    bars = [
        model
        for model in root.findall('world/model')
        if model.find("link/collision[@name='bump_collision']") is not None
    ]

    assert len(bars) == 11
    assert all(
        bar.findtext('link/collision/geometry/cylinder/length') == '2.2'
        for bar in bars
    )


def test_navigation_gauntlet_combines_friction_bumps_and_wrench(tmp_path):
    root, _ = generated_world(
        tmp_path,
        'navigation_gauntlet',
        zone_length_m=4.5,
        zone_width_m=4.5,
        low_friction_mu=0.18,
        roughness_spacing_m=0.55,
    )

    patch = root.find("world/model[@name='physical_low_friction_zone']")
    bumps = root.findall("world/model/link/collision[@name='bump_collision']")
    plugins = root.findall('world/plugin')
    assert patch is not None
    assert len(bumps) == 3
    patch_x = float(patch.findtext('pose').split()[0])
    patch_length = float(
        patch.findtext('link/collision/geometry/box/size').split()[0]
    )
    patch_min_x = patch_x - 0.5 * patch_length
    bump_models = [
        model
        for model in root.findall('world/model')
        if model.find("link/collision[@name='bump_collision']") is not None
    ]
    bump_x = [float(model.findtext('pose').split()[0]) for model in bump_models]
    bump_widths = [
        float(
            model.findtext(
                "link/collision[@name='bump_collision']/geometry/cylinder/length"
            )
        )
        for model in bump_models
    ]
    assert max(bump_x) + 0.05 < patch_min_x
    assert max(bump_x) < -1.25
    assert all(width == pytest.approx(4.5 * 0.65) for width in bump_widths)
    assert any(
        plugin.attrib.get('filename') == 'gz-sim-apply-link-wrench-system'
        for plugin in plugins
    )


@pytest.mark.parametrize('scenario', ['external_impact', 'wheel_block'])
def test_dynamic_scenes_install_only_the_native_wrench_system(tmp_path, scenario):
    root, _ = generated_world(tmp_path, scenario)
    plugins = root.findall('world/plugin')

    assert any(
        plugin.attrib.get('filename') == 'gz-sim-apply-link-wrench-system'
        for plugin in plugins
    )


def test_scene_bounds_reject_invisible_or_extreme_settings():
    module = load_scene_module()

    with pytest.raises(ValueError, match='low_friction_mu'):
        module.PhysicalSceneConfig('low_friction', low_friction_mu=0.99)
    with pytest.raises(ValueError, match='roughness_height_m'):
        module.PhysicalSceneConfig('rough_surface', roughness_height_m=0.10)
    with pytest.raises(ValueError, match='impact force'):
        module.PhysicalSceneConfig(
            'external_impact', impact_force_x_n=31.0, impact_force_y_n=0.0
        )
    with pytest.raises(ValueError, match='pulse_count'):
        module.PhysicalSceneConfig('external_impact', disturbance_pulse_count=6)


def test_launch_and_pulse_expose_only_scene_level_controls():
    launch_source = LAUNCH_FILE.read_text(encoding='utf-8')
    pulse_source = PULSE_FILE.read_text(encoding='utf-8')

    for scenario in (
        'normal',
        'low_friction',
        'asymmetric_traction',
        'rough_surface',
        'external_impact',
        'wheel_block',
        'navigation_gauntlet',
    ):
        assert scenario in launch_source
    assert 'phase9_slam_world.sdf' in launch_source
    assert 'ros_gz_interfaces/msg/EntityWrench' in launch_source
    assert "PERSISTENT_TOPIC = '/world/resilient_lab/wrench/persistent'" in pulse_source
    assert 'entity.type = Entity.MODEL' in pulse_source
    assert "self.create_subscription(Twist, '/cmd_vel'" in pulse_source
    assert 'wheel_block_force_n' in pulse_source
    assert "'pulse_count': 1" in pulse_source
    assert "'pulse_interval_sec': 3.0" in pulse_source
    assert (
        "if config.scenario in {'external_impact', 'navigation_gauntlet'}"
        in launch_source
    )
    assert 'wrench.torque.y' not in pulse_source
    assert '/health/' not in launch_source
    assert '/fusion/' not in launch_source


def test_physical_route_contains_zone_restart_and_bidirectional_yaw():
    """The collection route excites translation, stopping, and both yaw signs."""
    source = ROUTE_FILE.read_text(encoding='utf-8')

    assert "'zone_acceleration'" in source
    assert "'zone_stop'" in source
    assert "'zone_restart_exit'" in source
    assert 'MotionCommand(0.22, 0.22)' in source
    assert 'MotionCommand(0.22, -0.22)' in source
    assert "Parameter('use_sim_time', value=True)" in source
    assert 'Physical disturbance route complete' in source


def test_stress_routes_remain_bounded_and_sustain_the_disturbance():
    route = load_route_module()
    loops = route.route_segments('stress_loops')
    shuttle = route.route_segments('stress_shuttle')

    assert loops[0][0] == 'normal_startup'
    assert sum(segment[2] for segment in loops[1:]) > 50.0
    assert loops[1][1].linear_x > 0.0
    assert loops[1][1].angular_z < 0.0
    assert loops[-1][1].linear_x < 0.0
    assert loops[-1][1].angular_z > 0.0
    assert shuttle[0][0] == 'normal_startup'
    assert sum(segment[2] for segment in shuttle[1:]) == pytest.approx(36.0)
    assert {
        segment[1].linear_x > 0.0 for segment in shuttle[1:]
    } == {True, False}
