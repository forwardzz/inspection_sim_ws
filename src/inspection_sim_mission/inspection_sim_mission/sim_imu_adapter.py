import copy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .robot_config import FRAME_IMU, TOPIC_IMU_RAW, TOPIC_SIM_IMU_RAW


class SimImuAdapter(Node):
    def __init__(self):
        super().__init__("sim_imu_adapter")
        self.declare_parameter("input_topic", TOPIC_SIM_IMU_RAW)
        self.declare_parameter("output_topic", TOPIC_IMU_RAW)
        self.declare_parameter("frame_id", FRAME_IMU)
        self.declare_parameter("angular_velocity_z_sign", 1.0)
        self.declare_parameter("orientation_z_sign", 1.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.angular_velocity_z_sign = float(
            self.get_parameter("angular_velocity_z_sign").value
        )
        self.orientation_z_sign = float(self.get_parameter("orientation_z_sign").value)

        self.publisher = self.create_publisher(Imu, output_topic, qos_profile_sensor_data)
        self.create_subscription(Imu, input_topic, self._imu_cb, qos_profile_sensor_data)
        self.get_logger().info(f"Adapting {input_topic} -> {output_topic}")

    def _imu_cb(self, msg):
        adapted = copy.deepcopy(msg)
        adapted.header.frame_id = self.frame_id

        adapted.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]
        adapted.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001,
        ]
        adapted.orientation_covariance = [
            0.0025, 0.0, 0.0,
            0.0, 0.0025, 0.0,
            0.0, 0.0, 0.0025,
        ]

        adapted.angular_velocity.z *= self.angular_velocity_z_sign
        adapted.orientation.z *= self.orientation_z_sign
        self.publisher.publish(adapted)


def main(args=None):
    rclpy.init(args=args)
    node = SimImuAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
