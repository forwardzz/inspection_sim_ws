import math
from types import SimpleNamespace

import pytest

from inspection_sim_gui.ros_adapter import quaternion_to_yaw, yaw_to_quaternion


@pytest.mark.parametrize("yaw", [-math.pi, -0.5, 0.0, 0.5, math.pi])
def test_yaw_quaternion_round_trip(yaw):
    x, y, z, w = yaw_to_quaternion(yaw)
    quaternion = SimpleNamespace(x=x, y=y, z=z, w=w)

    actual = quaternion_to_yaw(quaternion)

    assert math.sin(actual) == pytest.approx(math.sin(yaw))
    assert math.cos(actual) == pytest.approx(math.cos(yaw))
