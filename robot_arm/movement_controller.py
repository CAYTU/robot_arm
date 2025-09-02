#!/usr/bin/env python3
"""
Movement Controller Class for Robot Arm
Handles trajectory planning and smooth movement logic
"""

import time
import numpy as np
from queue import Queue

class RobotArmMovementController:
    def __init__(self, joint_names, default_joints, joint_limits):
        self.joint_names = joint_names
        self.default_joints = default_joints
        self.joint_limits = joint_limits

        # Position tracking
        self.current_positions = self.default_joints.copy()
        self.target_positions = self.default_joints.copy()

        # Trajectory control
        self.trajectory_active = False
        self.trajectory_start_time = 0
        self.trajectory_duration = 2.0
        self.trajectory_start_positions = self.default_joints.copy()

        # Maximum joint velocities (rad/s) - adjust based on your robot's capabilities
        self.max_joint_velocities = {
            "joint_0": 0.5,
            "joint_1": 0.3,
            "joint_2": 0.4,
            "joint_3": 0.8,
            "joint_4": 0.8,
            "joint_5": 1.0,
            "left_carriage_joint": 0.1,
            "right_carriage_joint": 0.1,
        }

        # Movement command queue for handling rapid changes
        self.movement_queue = Queue()
        self.last_command_time = time.time()
        self.command_debounce_time = 0.1

        # Callbacks for status updates
        self.on_status_update = None

    def set_status_callback(self, callback):
        """Set callback for status updates"""
        self.on_status_update = callback

    def update_trajectory_duration(self, duration):
        """Update the trajectory duration"""
        self.trajectory_duration = float(duration)

    def queue_joint_movement(self, joint_index, value):
        """Queue a joint movement command"""
        try:
            float_value = float(value)
            self.movement_queue.put((joint_index, float_value, time.time()))
        except ValueError as e:
            print(f"Error queuing movement: {e}")

    def process_movement_commands(self, smooth_movement_enabled):
        """Process queued movement commands with debouncing"""
        current_time = time.time()

        # Get all queued commands and keep only the latest for each joint
        latest_commands = {}
        while not self.movement_queue.empty():
            joint_index, value, timestamp = self.movement_queue.get()
            latest_commands[joint_index] = (value, timestamp)

        # Apply commands if debounce time has passed
        if latest_commands and (current_time - self.last_command_time) > self.command_debounce_time:
            for joint_index, (value, timestamp) in latest_commands.items():
                self.target_positions[joint_index] = value

            if smooth_movement_enabled:
                self.start_smooth_trajectory()
            else:
                # Immediate movement
                self.current_positions = self.target_positions.copy()

            self.last_command_time = current_time

    def start_smooth_trajectory(self):
        """Start a smooth trajectory to target positions"""
        if not np.allclose(self.current_positions, self.target_positions, atol=0.001):
            self.trajectory_start_positions = self.current_positions.copy()
            self.trajectory_start_time = time.time()
            self.trajectory_active = True

            # Calculate adaptive duration based on maximum joint movement
            max_movement = 0
            for i, joint_name in enumerate(self.joint_names):
                movement = abs(self.target_positions[i] - self.current_positions[i])
                max_vel = self.max_joint_velocities.get(joint_name, 0.5)
                required_time = movement / max_vel if max_vel > 0 else 0
                max_movement = max(max_movement, required_time)

            # Use either the calculated time or user-set duration, whichever is reasonable
            self.trajectory_duration = max(0.5, min(self.trajectory_duration, max_movement * 2))

            if self.on_status_update:
                self.on_status_update(f"Moving smoothly (duration: {self.trajectory_duration:.1f}s)")

    def interpolate_trajectory(self):
        """Interpolate current position along the trajectory using smooth S-curve"""
        if not self.trajectory_active:
            return False

        current_time = time.time()
        elapsed_time = current_time - self.trajectory_start_time

        if elapsed_time >= self.trajectory_duration:
            # Trajectory complete
            self.current_positions = self.target_positions.copy()
            self.trajectory_active = False
            if self.on_status_update:
                self.on_status_update("Movement complete")
            return False
        else:
            # Calculate smooth interpolation factor using S-curve
            t = elapsed_time / self.trajectory_duration  # 0 to 1

            # S-curve interpolation for smooth acceleration/deceleration
            # Using smoothstep function: 3t² - 2t³
            smooth_t = 3 * t * t - 2 * t * t * t

            # Interpolate each joint position
            for i in range(len(self.current_positions)):
                start_pos = self.trajectory_start_positions[i]
                target_pos = self.target_positions[i]
                self.current_positions[i] = start_pos + (target_pos - start_pos) * smooth_t

            return True

    def stop_movement(self):
        """Immediately stop all movement"""
        self.trajectory_active = False
        self.target_positions = self.current_positions.copy()

        if self.on_status_update:
            self.on_status_update("Movement stopped")

        return self.current_positions.copy()

    def set_positions(self, positions, smooth_movement_enabled, status_message="Moving"):
        """Set joint positions with optional smooth movement"""
        try:
            # Ensure positions are within limits
            safe_positions = []
            for i, (joint_name, pos) in enumerate(zip(self.joint_names, positions)):
                if joint_name in self.joint_limits:
                    min_val, max_val = self.joint_limits[joint_name]
                    safe_pos = max(min_val, min(max_val, pos))
                    safe_positions.append(safe_pos)
                else:
                    safe_positions.append(pos)

            self.target_positions = safe_positions.copy()

            if smooth_movement_enabled:
                self.start_smooth_trajectory()
            else:
                self.current_positions = safe_positions.copy()

            if self.on_status_update:
                self.on_status_update(status_message)

            return safe_positions

        except Exception as e:
            print(f"Error in set_positions: {e}")
            return None

    def reset_to_default(self, smooth_movement_enabled):
        """Reset all joints to default positions"""
        return self.set_positions(
            self.default_joints.copy(),
            smooth_movement_enabled,
            "Moving to default positions"
        )

    def zero_all_joints(self, smooth_movement_enabled):
        """Set all joints to zero position"""
        zero_positions = [0.0] * len(self.joint_names)
        return self.set_positions(
            zero_positions,
            smooth_movement_enabled,
            "Moving to zero position"
        )

    def go_to_home(self, smooth_movement_enabled):
        """Go to a safe home position"""
        # Safe home positions for your robot
        home_positions = [0.0, -0.5, -1.0, 0.0, -0.5, 0.0, 0.0, 0.0]
        return self.set_positions(
            home_positions,
            smooth_movement_enabled,
            "Moving to home position"
        )

    def go_to_preset_1(self, smooth_movement_enabled):
        """Go to preset position 1 - example reaching position"""
        preset_positions = [0.5, -0.8, 1.2, 0.0, -0.5, 0.0, 0.0, 0.0]
        return self.set_positions(
            preset_positions,
            smooth_movement_enabled,
            "Moving to Preset Position 1"
        )

    def go_to_preset_2(self, smooth_movement_enabled):
        """Go to preset position 2 - example inspection position"""
        preset_positions = [-0.5, -1.0, 0.8, 1.57, 0.3, -0.8, 0.0, 0.0]
        return self.set_positions(
            preset_positions,
            smooth_movement_enabled,
            "Moving to Preset Position 2"
        )

    def get_current_positions(self):
        """Get current joint positions"""
        return self.current_positions.copy()

    def get_target_positions(self):
        """Get target joint positions"""
        return self.target_positions.copy()

    def is_trajectory_active(self):
        """Check if trajectory is currently active"""
        return self.trajectory_active