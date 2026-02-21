"""Verifies that proto-derived ROS2 message types are importable and usable."""
from point_msgs import msg


def test_point_message_instantiation():
    p = msg.Point()
    p.x = 1.0
    p.y = 2.0
    p.z = 3.0
    p.label = 'hello'
    p.id = 42
    p.valid = True
    assert p.x == 1.0
    assert p.y == 2.0
    assert p.z == 3.0
    assert p.label == 'hello'
    assert p.id == 42
    assert p.valid is True
