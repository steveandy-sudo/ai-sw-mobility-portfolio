import rclpy
import pytest
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from kaiev26_aeb.colored_path_node import ColoredConePathNode


def detection(x, y, color, score=0.9):
    item = Detection3D()
    item.bbox.center.position.x = x
    item.bbox.center.position.y = y
    item.bbox.center.orientation.w = 1.0
    item.bbox.size.x = item.bbox.size.y = 0.25
    item.bbox.size.z = 0.5
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = color
    hypothesis.hypothesis.score = score
    item.results = [hypothesis]
    return item


def test_cone_perception_message_builds_shared_path_and_red_gate():
    rclpy.init()
    node = ColoredConePathNode()
    try:
        results = []
        node.emit = lambda result, stamp, extra=None: results.append(result)
        message = Detection3DArray()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        for x in (4.0, 8.0, 12.0):
            message.detections.extend((detection(x, 2.0, 'blue cone'),
                                       detection(x, -2.0, 'yellow cone')))
        for x in (15.0, 18.0):
            message.detections.extend((detection(x, 2.0, 'red cone'),
                                       detection(x, -2.0, 'red cone')))
        message.detections.append(detection(10.0, 0.0, 'unknown'))
        node.on_detections(message)
        node.tick()
        assert results[-1]['plan']['valid']
        assert len(results[-1]['plan']['red_gate']) == 2
        assert len(results[-1]['cones']) == 10
        assert results[-1]['plan']['path'][0, 1] == pytest.approx(0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
