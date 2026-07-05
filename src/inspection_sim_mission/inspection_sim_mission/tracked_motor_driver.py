#!/usr/bin/env python3
"""Tracked vehicle motor driver using RPi.GPIO direct drive."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .robot_config import (
    LEFT_IN1_PIN,
    LEFT_IN2_PIN,
    LEFT_PWM_PIN,
    MAX_ANGULAR_SPEED_RADPS,
    MAX_LINEAR_SPEED_MPS,
    RIGHT_IN1_PIN,
    RIGHT_IN2_PIN,
    RIGHT_PWM_PIN,
    TOPIC_CMD_VEL,
    TRACK_WIDTH_M,
    WHEEL_RADIUS_M,
)

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    GPIO = None
    HAS_GPIO = False


class TrackedMotorDriver(Node):
    def __init__(self):
        super().__init__("tracked_motor_driver")

        self.declare_parameter("max_rpm", 80.0)
        self.declare_parameter("wheel_radius", WHEEL_RADIUS_M)
        self.declare_parameter("track_width", TRACK_WIDTH_M)
        self.declare_parameter("min_breakout_pwm", 28.0)
        self.declare_parameter("max_pwm", 70.0)
        self.declare_parameter("max_linear_speed", MAX_LINEAR_SPEED_MPS)
        self.declare_parameter("max_angular_speed", MAX_ANGULAR_SPEED_RADPS)
        self.declare_parameter("max_linear_accel", 0.35)
        self.declare_parameter("max_angular_accel", 0.70)
        self.declare_parameter("control_period_sec", 0.05)
        self.declare_parameter("left_pwm_pin", LEFT_PWM_PIN)
        self.declare_parameter("left_in1_pin", LEFT_IN1_PIN)
        self.declare_parameter("left_in2_pin", LEFT_IN2_PIN)
        self.declare_parameter("right_pwm_pin", RIGHT_PWM_PIN)
        self.declare_parameter("right_in1_pin", RIGHT_IN1_PIN)
        self.declare_parameter("right_in2_pin", RIGHT_IN2_PIN)

        self.max_rpm = float(self.get_parameter("max_rpm").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.track_width = float(self.get_parameter("track_width").value)
        self.min_breakout_pwm = float(self.get_parameter("min_breakout_pwm").value)
        self.max_pwm = float(self.get_parameter("max_pwm").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.max_linear_accel = float(self.get_parameter("max_linear_accel").value)
        self.max_angular_accel = float(self.get_parameter("max_angular_accel").value)
        self.control_period_sec = float(self.get_parameter("control_period_sec").value)
        self.max_pwm = max(self.min_breakout_pwm, min(self.max_pwm, 100.0))

        self.left_pwm_pin = int(self.get_parameter("left_pwm_pin").value)
        self.left_in1_pin = int(self.get_parameter("left_in1_pin").value)
        self.left_in2_pin = int(self.get_parameter("left_in2_pin").value)
        self.right_pwm_pin = int(self.get_parameter("right_pwm_pin").value)
        self.right_in1_pin = int(self.get_parameter("right_in1_pin").value)
        self.right_in2_pin = int(self.get_parameter("right_in2_pin").value)
        self.left_motor = None
        self.right_motor = None

        if HAS_GPIO:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            for pin in [
                self.left_pwm_pin,
                self.left_in1_pin,
                self.left_in2_pin,
                self.right_pwm_pin,
                self.right_in1_pin,
                self.right_in2_pin,
            ]:
                GPIO.setup(pin, GPIO.OUT)
            self.left_motor = GPIO.PWM(self.left_pwm_pin, 100)
            self.right_motor = GPIO.PWM(self.right_pwm_pin, 100)
            self.left_motor.start(0)
            self.right_motor.start(0)
            self.get_logger().info(
                "GPIO motor pins initialized, "
                f"breakout pwm={self.min_breakout_pwm}%, max pwm={self.max_pwm}%"
            )
        else:
            self.get_logger().error("RPi.GPIO not available; /cmd_vel will be logged but motors will not move")

        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.target_vx = 0.0
        self.target_vz = 0.0
        self.current_vx = 0.0
        self.current_vz = 0.0

        self.create_subscription(Twist, TOPIC_CMD_VEL, self.cmd_vel_cb, 10)
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.5, self.watchdog_cb)
        self.create_timer(self.control_period_sec, self.control_cb)
        self.get_logger().info("Motor driver ready, waiting for /cmd_vel ...")

    def set_motor(self, is_left, target_rpm):
        if not HAS_GPIO:
            return

        if is_left:
            in1_pin = self.left_in1_pin
            in2_pin = self.left_in2_pin
            pwm_obj = self.left_motor
        else:
            in1_pin = self.right_in1_pin
            in2_pin = self.right_in2_pin
            pwm_obj = self.right_motor

        abs_rpm = abs(target_rpm)
        if abs_rpm <= 0.5:
            pwm_obj.ChangeDutyCycle(0)
            GPIO.output(in1_pin, False)
            GPIO.output(in2_pin, False)
            return

        abs_rpm = min(abs_rpm, self.max_rpm)
        duty = self.min_breakout_pwm + (
            self.max_pwm - self.min_breakout_pwm
        ) * (abs_rpm / self.max_rpm)

        if target_rpm > 0:
            GPIO.output(in1_pin, True)
            GPIO.output(in2_pin, False)
        else:
            GPIO.output(in1_pin, False)
            GPIO.output(in2_pin, True)
        pwm_obj.ChangeDutyCycle(duty)

    def cmd_vel_cb(self, msg):
        self.last_cmd_time = self.get_clock().now()
        self.target_vx = self._clamp(msg.linear.x, -self.max_linear_speed, self.max_linear_speed)
        self.target_vz = self._clamp(msg.angular.z, -self.max_angular_speed, self.max_angular_speed)

    def control_cb(self):
        self.current_vx = self._step_toward(
            self.current_vx,
            self.target_vx,
            self.max_linear_accel * self.control_period_sec,
        )
        self.current_vz = self._step_toward(
            self.current_vz,
            self.target_vz,
            self.max_angular_accel * self.control_period_sec,
        )

        left_rad_s = (
            self.current_vx - self.current_vz * self.track_width / 2.0
        ) / self.wheel_radius
        right_rad_s = (
            self.current_vx + self.current_vz * self.track_width / 2.0
        ) / self.wheel_radius
        self.left_rpm = left_rad_s * 60.0 / (2.0 * math.pi)
        self.right_rpm = right_rad_s * 60.0 / (2.0 * math.pi)
        self.set_motor(True, self.left_rpm)
        self.set_motor(False, self.right_rpm)

    def watchdog_cb(self):
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds / 1e9 > 0.5:
            self.target_vx = 0.0
            self.target_vz = 0.0

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, float(value)))

    @staticmethod
    def _step_toward(current, target, max_step):
        if target > current:
            return min(target, current + max_step)
        if target < current:
            return max(target, current - max_step)
        return current

    def destroy_node(self):
        self.set_motor(True, 0)
        self.set_motor(False, 0)
        if HAS_GPIO:
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackedMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
