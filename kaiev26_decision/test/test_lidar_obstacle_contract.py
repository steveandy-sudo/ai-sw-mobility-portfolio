from types import SimpleNamespace

from kaiev26_msgs.msg import PerceptionObject, PerceptionObjectArray
from rclpy.clock import ClockType
from rclpy.time import Time

from kaiev26_decision.perception_gateway_node import ObservationFrameJoiner, PerceptionGatewayNode


def test_gateway_keeps_raw_lidar_age_without_promoting_incomplete_vision_frame():
    gateway = PerceptionGatewayNode.__new__(PerceptionGatewayNode)
    gateway.latest_road_segments = gateway.latest_centerline = None
    gateway.latest_objects = gateway.latest_traffic_lights = gateway.latest_route_context = None
    gateway.observation_frames = ObservationFrameJoiner()
    gateway.stale_data_sec, gateway.lost_data_sec = 0.5, 2.0
    gateway.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=10, clock_type=ClockType.ROS_TIME))
    captured = []
    gateway.scene_pub = SimpleNamespace(publish=captured.append)
    gateway.events_pub = gateway.debug_pub = SimpleNamespace(publish=lambda _: None)
    objects = PerceptionObjectArray()
    objects.header.stamp.sec = 10
    objects.header.frame_id = 'base_footprint'
    obj = PerceptionObject(class_id=PerceptionObject.CLASS_OBSTACLE)
    obj.pose.position.x = 5.0
    obj.size.x = obj.size.y = 0.5
    objects.objects.append(obj)
    gateway.on_objects(objects)
    older = PerceptionObjectArray()
    older.header.stamp.sec = 9
    gateway.on_objects(older)
    assert gateway.latest_objects is None
    assert gateway.latest_obstacle_objects is objects
    gateway.publish_scene_summary()
    assert len(captured) == 1
    # D16: obstacles are judged by the planning engine; the summary keeps its defaults.
    assert not captured[0].obstacle_on_path
    assert captured[0].perception_health == 'LOST'
    assert not captured[0].centerline_valid
