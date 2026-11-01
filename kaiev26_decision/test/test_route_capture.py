import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from kaiev26_msgs.msg import Centerline, RouteContext, VehicleState
from nav_msgs.msg import Odometry
from kaiev26_decision import route_capture
from kaiev26_decision.route_capture import (
    capture_from_bag, discover_routes, make_tracking_profile,
    profile_parameters, resample_route, verify_origin,
)
from kaiev26_motion_control.motion_control_node import MotionControlNode, route_segment_index


PARAMETERS = {'map_origin.configured': True, 'map_origin.latitude_deg': 37.,
              'map_origin.longitude_deg': 127., 'map_origin.altitude_m': 0.,
              'frames.map': 'map', 'frames.base': 'base_footprint'}


def route(points):
    document = dict(route_name='test', route_id='test', capture_mode='manual_tui',
                    frame_id='map', reference_point='base_footprint',
                    map_origin=dict(latitude_deg=37., longitude_deg=127., altitude_m=0.),
                    waypoints=[dict(x=x, y=y, z=0., zone='NORMAL_ZONE') for x, y in points])
    document['tracking_profile'] = make_tracking_profile(document)
    return document


@pytest.mark.parametrize('package_path', ['src/kaiev26_maps', 'src/Simulation/kaiev26_maps'])
def test_default_route_directory_finds_vehicle_and_local_layouts(tmp_path, monkeypatch, package_path):
    destination = tmp_path / package_path / 'maps/midan'
    destination.mkdir(parents=True)
    (destination.parent.parent / 'package.xml').write_text('<package/>')
    monkeypatch.setattr(route_capture, '__file__',
                        str(tmp_path / 'src/Decision/kaiev26_decision/kaiev26_decision/route_capture.py'))
    assert route_capture.workspace_root() == tmp_path
    assert route_capture.route_directory() == destination


def test_default_route_directory_prefers_vehicle_layout_when_both_exist(tmp_path, monkeypatch):
    for package_path in route_capture.MAP_PACKAGE_PATHS:
        package = tmp_path / package_path
        package.mkdir(parents=True)
        (package / 'package.xml').write_text('<package/>')
    monkeypatch.setattr(route_capture, 'workspace_root', lambda: tmp_path)
    assert route_capture.route_directory() == tmp_path / 'src/kaiev26_maps/maps/midan'


def test_default_route_directory_falls_back_to_vehicle_source_path(tmp_path, monkeypatch):
    monkeypatch.setattr(route_capture, 'workspace_root', lambda: tmp_path)
    assert route_capture.route_directory() == tmp_path / 'src/kaiev26_maps/maps/midan'


def test_dynamics_timing_does_not_deserialize_obstacle_sensor_or_debug_data(tmp_path, monkeypatch):
    from kaiev26_decision.test_tui import TestSettings
    topics = TestSettings(case=4, course='custom:run', route_config='/tmp/route.yaml').record_topics
    selected = []

    def messages(bag, filtered):
        selected.extend(filtered)
        return iter(())

    monkeypatch.setattr(route_capture, 'bag_messages', messages)
    assert route_capture.recording_quality(tmp_path, topics) == {}
    assert '/vehicle/state' in selected
    assert '/ouster/points' not in selected
    assert not any(t.startswith(('/perception/', '/pv26_perception/')) for t in selected)


def test_straight_and_short_route_profiles():
    for points in ([(0., 0.), (.2, 0.)], [(float(x), 0.) for x in range(100)]):
        document = route(points)
        values = profile_parameters(document)
        assert values['hybrid_route_segment_phases'] == ['straight']


def test_small_heading_wobble_does_not_create_full_turn_phase():
    points = [(float(x), 0.) for x in range(21)]
    points += [(20.+50*math.sin(a), 50.-50*math.cos(a))
               for a in [i*math.radians(6)/20 for i in range(1, 21)]]
    x, y = points[-1]
    points += [(x+i*math.cos(math.radians(6)), y+i*math.sin(math.radians(6)))
               for i in range(1, 31)]
    document = route(points)
    assert document['tracking_profile']['corner_count'] == 0
    assert profile_parameters(document)['hybrid_route_segment_phases'] == ['straight']


def test_route_identity_cannot_escape_map_directory(tmp_path):
    from kaiev26_decision.route_capture import load_custom_route
    document = route([(0., 0.), (10., 0.)])
    document['route_id'] = '../escape'
    path = tmp_path / 'route.yaml'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='manually captured'):
        load_custom_route(path)


def test_multiple_corners_are_not_one_global_turn():
    points = [(float(x), 0.) for x in range(31)]
    points += [(30.+10*math.sin(a), 10.-10*math.cos(a))
               for a in [i*math.pi/100 for i in range(1, 51)]]
    points += [(40., float(y)) for y in range(11, 81)]
    points += [(30.+10*math.cos(a), 80.+10*math.sin(a))
               for a in [i*math.pi/100 for i in range(1, 51)]]
    points += [(float(x), 90.) for x in range(29, -20, -1)]
    document = route(points)
    phases = profile_parameters(document)['hybrid_route_segment_phases']
    assert phases.count('turning') == 2
    assert phases.count('straight') >= 3
    shifted = route([(x-22000, y+31000) for x, y in points])
    assert profile_parameters(shifted)['hybrid_route_segment_phases'] == phases
    assert make_tracking_profile(document) == document['tracking_profile']


@pytest.mark.parametrize('edit', ['geometry', 'hash', 'gap', 'phase', 'origin'])
def test_invalid_profile_not_registered(edit):
    doc = route([(0., 0.), (20., 0.)])
    if edit == 'geometry':
        doc['waypoints'][-1]['x'] = 21.
    elif edit == 'hash':
        doc['tracking_profile']['geometry_sha256'] = 'wrong'
    elif edit == 'gap':
        doc['tracking_profile']['segments'][0]['start_s_m'] = 1.
    elif edit == 'phase':
        doc['tracking_profile']['segments'][0]['phase'] = 'wrong'
    else:
        doc['map_origin']['latitude_deg'] = 38.
    with pytest.raises(ValueError):
        profile_parameters(doc)


def test_resampling_preserves_endpoints_not_time_spacing():
    points = resample_route([(0., 0., 0.), (.5, 1.5, 0.), (1., 2.4, 0.)])
    assert points == [(0., 0.), (1., 0.), (2., 0.), (2.4, 0.)]
    with pytest.raises(ValueError):
        resample_route([(0., 0., 0.), (.1, 100., 0.)])


def test_origin_checked_without_reprojection():
    document = route([(0., 0.), (10., 0.)])
    verify_origin(document, PARAMETERS)
    with pytest.raises(ValueError):
        verify_origin(document, {**PARAMETERS, 'map_origin.latitude_deg': 38.})


def test_recorded_mcap_to_yaml_and_discovery(tmp_path):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    bag = tmp_path / 'bag'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    for topic, kind in [('/vehicle/state', 'kaiev26_msgs/msg/VehicleState'),
                        ('/localization/odometry', 'nav_msgs/msg/Odometry')]:
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=kind, serialization_format='cdr'))
    for i in range(60):
        ns = 1_000_000_000 + i*100_000_000
        v = VehicleState(mode=VehicleState.MODE_MANUAL, speed_mps=1.)
        o = Odometry()
        for m in (v, o):
            m.header.stamp.sec, m.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        o.header.frame_id, o.child_frame_id = 'map', 'base_footprint'
        o.pose.pose.position.x = i*.1
        o.pose.pose.orientation.w = 1.
        o.pose.covariance[0] = o.pose.covariance[7] = .01
        for t, m in [('/vehicle/state', v), ('/localization/odometry', o)]:
            writer.write(t, serialize_message(m), ns)
    del writer
    root = tmp_path / 'kaiev26_maps/maps/midan'
    root.mkdir(parents=True)
    (root.parent.parent / 'package.xml').write_text('<package/>')
    existing_route = root / 'midan_outbound_route.yaml'
    existing_route.write_text('existing route\n')
    result = capture_from_bag(bag, 1_000_000_000, 5_000_000_000, PARAMETERS, 'test route', 'run', root)
    assert existing_route.read_text() == 'existing route\n'
    document = yaml.safe_load(Path(result['path']).read_text())
    assert document['waypoints'][0]['x'] == 0.
    assert document['waypoints'][-1]['x'] == 4.
    assert document['tracking_profile']['segments'][0]['phase'] == 'straight'
    routes, errors = discover_routes(root)
    assert not errors and routes['custom:run'][0] == 'test route'
    with pytest.raises(FileExistsError):
        capture_from_bag(bag, 1_000_000_000, 5_000_000_000, PARAMETERS, 'test route', 'run', root)
    assert yaml.safe_load(Path(result['path']).read_text()) == document


def test_segment_boundaries_and_non_route_fallback():
    node = MotionControlNode.__new__(MotionControlNode)
    node.hybrid_route_segment_boundaries_m = (0., 10., 20.)
    node.hybrid_route_segment_phases = ('straight', 'turning')
    node.hybrid_route_length_m = 20.
    node.target_stale_sec = .35
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_010_000_000))
    node.latest_target_path = Centerline(source='global_route_local_path')
    node.latest_route_context = RouteContext(route_projection_valid=True)
    node.latest_route_context.header.stamp.sec = 10
    for progress, phase in [(0., 'straight'), (9.999, 'straight'), (10., 'turning'), (20., 'turning')]:
        node.latest_route_context.progress_s = progress
        node.latest_route_context.distance_to_finish = 20.-progress
        assert node.select_hybrid_phase('turn_exit') == phase
    node.latest_target_path.source = 'obstacle_path'
    assert node.select_hybrid_phase('turn_exit') == 'turn_exit'
    assert not node.hybrid_route_phase_active
    assert route_segment_index(float('nan'), (0., 10.)) == -1


def test_manual_capture_ros_recorder_lifecycle(tmp_path, monkeypatch):
    """Real recorder, fake inputs, isolated DDS; no actuator receiver exists."""
    import time
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix
    from tf2_msgs.msg import TFMessage
    from kaiev26_msgs.msg import ActuatorCommand
    from std_msgs.msg import Bool
    from kaiev26_decision import test_tui

    monkeypatch.setenv('ROS_DOMAIN_ID', '184')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setattr(test_tui, 'workspace_root', lambda: tmp_path)
    destination = tmp_path / 'src/Simulation/kaiev26_maps/maps/midan'
    destination.mkdir(parents=True)
    (destination.parent.parent / 'package.xml').write_text('<package/>')
    rclpy.init()
    source = Node('eskf_localization')
    for name, value in {**PARAMETERS, 'frames.gnss': 'gps', 'mode.gnss_bypass': True,
                        'time.max_delay_sec': 1., 'time.sync_tolerance_sec': .01}.items():
        source.declare_parameter(name, value)
    node = test_tui.DecisionTestNode(test_tui.TestSettings(case=3, route_name='manual run'))
    node.route_dir = destination
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(node)
    publishers = {topic: source.create_publisher(kind, topic, 10) for topic, kind in (
        ('/vehicle/state', VehicleState), ('/localization/odometry', Odometry),
        ('/gnss/fix', NavSatFix), ('/tf_static', TFMessage))}
    emitted = []
    source.create_subscription(ActuatorCommand, '/planning/command', lambda m: emitted.append(m), 10)
    source.create_subscription(Bool, '/decision/test_run_active', lambda m: emitted.append(m), 10)

    def pump(duration, moving=False, origin=0.):
        begin = time.monotonic()
        while time.monotonic()-begin < duration:
            elapsed = time.monotonic()-begin
            stamp = source.get_clock().now().to_msg()
            vehicle = VehicleState(mode=VehicleState.MODE_MANUAL, speed_mps=1. if moving else 0.)
            odom = Odometry()
            vehicle.header.stamp = odom.header.stamp = stamp
            odom.header.frame_id, odom.child_frame_id = 'map', 'base_footprint'
            odom.pose.pose.position.x = origin+elapsed if moving else origin
            odom.pose.pose.orientation.w = 1.
            odom.pose.covariance[0] = odom.pose.covariance[7] = .01
            for topic, message in (('/vehicle/state', vehicle), ('/localization/odometry', odom),
                                   ('/gnss/fix', NavSatFix()), ('/tf_static', TFMessage())):
                publishers[topic].publish(message)
            for _ in range(8):
                executor.spin_once(timeout_sec=.005)
            time.sleep(.02)
    try:
        pump(1.2)
        node.request_run(node.settings)
        deadline = time.monotonic()+10.
        while node.state == 'CAPTURE_PREP' and time.monotonic() < deadline:
            pump(.2)
        assert node.state == 'WAIT_MOVEMENT', (node.state, list(node.logs), node.runtime_parameters)
        pump(2., moving=True)
        assert node.state == 'CAPTURING'
        node.request_stop()
        node.close_processes(node.finish_reason)
        assert node.capture_worker is not None
        node.capture_worker.join(15.)
        assert node.state == 'COMPLETE', node.finish_reason
        assert emitted == []
        saved = yaml.safe_load((Path(node.run_dir) / 'scenario.yaml').read_text())
        assert saved['recorded_topics']['/localization/odometry']['count'] > 10
        assert saved['recorded_topics'].get('/planning/command', {}).get('count', 0) == 0
        assert saved['generated_route']['length_m'] > 1.5
        registered, errors = discover_routes(destination)
        assert len(registered) == 1 and not errors
    finally:
        if node.recorder_process or node.launch_process:
            node.close_processes('test_cleanup')
        if node.capture_worker:
            node.capture_worker.join(15.)
        executor.shutdown()
        node.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
