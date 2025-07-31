#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time

class RobotArm(Node):
    def __init__(self):
        super().__init__("robot_arm_controller")

        # Publisher for joint commands
        self.publisher_ = self.create_publisher(JointState, "joint_command", 10)

        # Joint names (assuming 7 joints)
        self.joint_state = JointState()
        self.joint_state.name = [
            # "waist",
            # "shoulder",
            # "elbow",
            # "forearm_roll",
            # "wrist_angle",
            # "wrist_rotate",
            "gripper",
        ]

        # Default joint positions
        self.default_joints = [
          # 0.0, -1.16, -0.0, -2.3, -0.0, 1.6,
           1.1]

        # Gripper open/close positions (adjust based on your robot)
        self.gripper_open_pos = 0.0  # Example: 0.0 = open
        self.gripper_close_pos = 1.1  # Example: 1.1 = closed

        # Movement limits for non-gripper joints
        self.max_joints = np.array(self.default_joints) + 0.5
        self.min_joints = np.array(self.default_joints) - 0.5

        # Timing for gripper control
        self.time_start = time.time()
        self.last_gripper_toggle = time.time()
        self.gripper_is_open = False  # Start with closed gripper

        # Timer for control loop (20Hz)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        current_time = time.time()

        # Toggle gripper every 5 seconds
        if current_time - self.last_gripper_toggle >= 5.0:
            self.gripper_is_open = not self.gripper_is_open
            self.last_gripper_toggle = current_time

        # Update joint positions
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        # Sinusoidal motion for non-gripper joints
        joint_positions = (
            np.sin(current_time - self.time_start) * (self.max_joints - self.min_joints) * 0.5 + self.default_joints
        ).tolist()

        # Override gripper position (open/close)
        if self.gripper_is_open:
            joint_positions[-1] = self.gripper_open_pos  # Open gripper
        else:
            joint_positions[-1] = self.gripper_close_pos  # Close gripper

        self.joint_state.position = joint_positions
        self.publisher_.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    robot_arm = RobotArm()
    rclpy.spin(robot_arm)
    robot_arm.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()