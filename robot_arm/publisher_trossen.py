#!/usr/bin/env python3
# Robot Arm Joint Control GUI with Smooth Trajectory Interpolation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tkinter as tk
from tkinter import ttk
import threading
import numpy as np
import time
from queue import Queue

class SmoothRobotArmGUI(Node):
    def __init__(self):
        super().__init__("smooth_robot_arm_gui_controller")

        # Create the publisher
        self.publisher_ = self.create_publisher(JointState, "joint_command", 10)

        # Joint configuration - matching your robot's joints
        self.joint_names = [
            "joint_0",
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "left_carriage_joint",
            "right_carriage_joint",
        ]

        # Default joint positions (safe starting positions)
        self.default_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Joint limits (adjust these based on your robot's actual limits)
        self.joint_limits = {
            "joint_0": (-3.14, 3.14),      # Base rotation - full rotation
            "joint_1": (0, 1.57),          # Shoulder - typical range
            "joint_2": (0, 2.09),          # Elbow - typical range
            "joint_3": (-1.57, 1.57),     # Wrist roll - full rotation
            "joint_4": (-1.75, 1.75),     # Wrist pitch - typical range
            "joint_5": (-3.14, 3.14),     # Wrist yaw - full rotation
            "left_carriage_joint": (-0.04, 0.04),   # Gripper/carriage - small range
            "right_carriage_joint": (-0.1, 0.1),    # Gripper/carriage - small range
        }

        # Display names for joints (more user-friendly)
        self.joint_display_names = {
            "joint_0": "Base (Joint 0)",
            "joint_1": "Shoulder (Joint 1)",
            "joint_2": "Elbow (Joint 2)",
            "joint_3": "Wrist Roll (Joint 3)",
            "joint_4": "Wrist Pitch (Joint 4)",
            "joint_5": "Wrist Yaw (Joint 5)",
            "left_carriage_joint": "Left Carriage",
            "right_carriage_joint": "Right Carriage",
        }

        # Current joint positions (what the robot is currently at)
        self.current_positions = self.default_joints.copy()

        # Target joint positions (where we want to go)
        self.target_positions = self.default_joints.copy()

        # Trajectory interpolation parameters
        self.trajectory_active = False
        self.trajectory_start_time = 0
        self.trajectory_duration = 2.0  # seconds for smooth movement
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

        # Movement command queue for handling rapid slider changes
        self.movement_queue = Queue()
        self.last_command_time = time.time()
        self.command_debounce_time = 0.1  # seconds

        # Initialize GUI components dictionaries
        self.sliders = {}
        self.value_labels = {}
        self.status_label = None
        self.smooth_movement_var = None

        # Create the GUI
        self.setup_gui()

        # Timer for publishing joint states and trajectory interpolation
        timer_period = 0.02  # 50 Hz for smooth interpolation
        self.timer = self.create_timer(timer_period, self.update_and_publish)

        # Timer for processing movement commands (debounced)
        self.command_timer = self.create_timer(0.05, self.process_movement_commands)

        self.get_logger().info("Smooth Robot Arm GUI Controller started")

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("Smooth Robot Arm Joint Controller")
        self.root.geometry("950x750")

        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)

        # Title
        title_label = ttk.Label(main_frame, text="Smooth Robot Arm Joint Controller",
                               font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))

        # Control settings frame
        settings_frame = ttk.LabelFrame(main_frame, text="Movement Settings", padding="5")
        settings_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))

        # Smooth movement checkbox
        self.smooth_movement_var = tk.BooleanVar(value=True)
        smooth_checkbox = ttk.Checkbutton(settings_frame, text="Enable Smooth Movement",
                                         variable=self.smooth_movement_var)
        smooth_checkbox.grid(row=0, column=0, sticky=tk.W, padx=(0, 20))

        # Movement speed scale
        ttk.Label(settings_frame, text="Movement Speed:").grid(row=0, column=1, sticky=tk.W, padx=(0, 5))
        self.speed_scale = ttk.Scale(settings_frame, from_=0.5, to=5.0, orient=tk.HORIZONTAL,
                                    length=200, command=self.update_trajectory_duration)
        self.speed_scale.set(2.0)  # Default speed
        self.speed_scale.grid(row=0, column=2, sticky=tk.W, padx=(5, 10))

        self.speed_label = ttk.Label(settings_frame, text="2.0s")
        self.speed_label.grid(row=0, column=3, sticky=tk.W)

        # Create sliders for each joint
        for i, joint_name in enumerate(self.joint_names):
            row = i + 2

            # Joint name label (use display name)
            display_name = self.joint_display_names.get(joint_name, joint_name)
            name_label = ttk.Label(main_frame, text=display_name, width=20)
            name_label.grid(row=row, column=0, sticky=tk.W, padx=(0, 10), pady=2)

            # Get joint limits
            min_val, max_val = self.joint_limits[joint_name]

            # Slider
            slider = ttk.Scale(main_frame, from_=min_val, to=max_val,
                              orient=tk.HORIZONTAL, length=400,
                              command=lambda val, idx=i: self.slider_changed(idx, val))
            slider.set(self.default_joints[i])
            slider.grid(row=row, column=1, sticky=(tk.W, tk.E), padx=10, pady=2)

            # Value label
            value_label = ttk.Label(main_frame, text=f"{self.default_joints[i]:.3f}",
                                   width=8)
            value_label.grid(row=row, column=2, sticky=tk.W, padx=(10, 0), pady=2)

            # Store references to GUI components
            self.sliders[joint_name] = slider
            self.value_labels[joint_name] = value_label

        # Control buttons frame
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=len(self.joint_names) + 2, column=0, columnspan=3,
                         pady=(20, 0), sticky=(tk.W, tk.E))

        # Reset to default button
        reset_button = ttk.Button(button_frame, text="Reset to Default",
                                 command=self.reset_to_default)
        reset_button.pack(side=tk.LEFT, padx=(0, 10))

        # Zero all joints button
        zero_button = ttk.Button(button_frame, text="Zero All Joints",
                                command=self.zero_all_joints)
        zero_button.pack(side=tk.LEFT, padx=10)

        # Home position button (safe position)
        home_button = ttk.Button(button_frame, text="Home Position",
                                command=self.go_to_home)
        home_button.pack(side=tk.LEFT, padx=10)

        # Stop movement button
        stop_button = ttk.Button(button_frame, text="STOP MOVEMENT",
                                command=self.stop_movement)
        stop_button.pack(side=tk.LEFT, padx=10)

        # Preset positions frame
        preset_frame = ttk.Frame(main_frame)
        preset_frame.grid(row=len(self.joint_names) + 3, column=0, columnspan=3,
                         pady=(10, 0), sticky=(tk.W, tk.E))

        # Preset position buttons
        preset1_button = ttk.Button(preset_frame, text="Preset 1",
                                   command=self.preset_position_1)
        preset1_button.pack(side=tk.LEFT, padx=10)

        preset2_button = ttk.Button(preset_frame, text="Preset 2",
                                   command=self.preset_position_2)
        preset2_button.pack(side=tk.LEFT, padx=10)

        # Status label
        self.status_label = ttk.Label(main_frame, text="Status: Ready",
                                     font=("Arial", 10))
        self.status_label.grid(row=len(self.joint_names) + 4, column=0, columnspan=3,
                              pady=(20, 0))

        self.get_logger().info("GUI setup complete")

    def update_trajectory_duration(self, value):
        """Update the trajectory duration based on speed slider"""
        duration = float(value)
        self.trajectory_duration = duration
        self.speed_label.config(text=f"{duration:.1f}s")

    def slider_changed(self, joint_index, value):
        """Called when a slider value changes - adds command to queue for debouncing"""
        try:
            float_value = float(value)
            joint_name = self.joint_names[joint_index]

            # Update the display immediately
            if joint_name in self.value_labels:
                self.value_labels[joint_name].config(text=f"{float_value:.3f}")

            # Add movement command to queue (debounced processing)
            self.movement_queue.put((joint_index, float_value, time.time()))

        except (ValueError, KeyError) as e:
            self.get_logger().error(f"Error in slider_changed: {e}")

    def process_movement_commands(self):
        """Process movement commands from the queue with debouncing"""
        current_time = time.time()

        # Process all commands in queue, keeping only the latest for each joint
        latest_commands = {}
        while not self.movement_queue.empty():
            joint_index, value, timestamp = self.movement_queue.get()
            latest_commands[joint_index] = (value, timestamp)

        # Check if enough time has passed since last command
        if latest_commands and (current_time - self.last_command_time) > self.command_debounce_time:
            # Update target positions and start trajectory
            for joint_index, (value, timestamp) in latest_commands.items():
                self.target_positions[joint_index] = value

            if self.smooth_movement_var.get():
                self.start_smooth_trajectory()
            else:
                # Immediate movement (old behavior)
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
                required_time = movement / max_vel
                max_movement = max(max_movement, required_time)

            # Use either the calculated time or user-set duration, whichever is reasonable
            self.trajectory_duration = max(0.5, min(self.trajectory_duration, max_movement * 2))

            if hasattr(self, 'status_label') and self.status_label is not None:
                self.status_label.config(text=f"Status: Moving smoothly (duration: {self.trajectory_duration:.1f}s)")

    def interpolate_trajectory(self):
        """Interpolate current position along the trajectory using smooth S-curve"""
        if not self.trajectory_active:
            return

        current_time = time.time()
        elapsed_time = current_time - self.trajectory_start_time

        if elapsed_time >= self.trajectory_duration:
            # Trajectory complete
            self.current_positions = self.target_positions.copy()
            self.trajectory_active = False
            if hasattr(self, 'status_label') and self.status_label is not None:
                self.status_label.config(text="Status: Movement complete")
        else:
            # Calculate smooth interpolation factor using S-curve (sigmoid-based)
            t = elapsed_time / self.trajectory_duration  # 0 to 1

            # S-curve interpolation for smooth acceleration/deceleration
            # Using smoothstep function: 3t² - 2t³
            smooth_t = 3 * t * t - 2 * t * t * t

            # Interpolate each joint position
            for i in range(len(self.current_positions)):
                start_pos = self.trajectory_start_positions[i]
                target_pos = self.target_positions[i]
                self.current_positions[i] = start_pos + (target_pos - start_pos) * smooth_t

    def stop_movement(self):
        """Immediately stop all movement"""
        self.trajectory_active = False
        self.target_positions = self.current_positions.copy()

        # Update sliders to current positions
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in self.sliders:
                self.sliders[joint_name].set(self.current_positions[i])
                self.value_labels[joint_name].config(text=f"{self.current_positions[i]:.3f}")

        if hasattr(self, 'status_label') and self.status_label is not None:
            self.status_label.config(text="Status: Movement stopped")

        self.get_logger().info("Movement stopped by user")

    def reset_to_default(self):
        """Reset all joints to default positions"""
        try:
            self.target_positions = self.default_joints.copy()

            # Update sliders
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in self.sliders:
                    self.sliders[joint_name].set(self.default_joints[i])
                    self.value_labels[joint_name].config(text=f"{self.default_joints[i]:.3f}")

            if self.smooth_movement_var.get():
                self.start_smooth_trajectory()
            else:
                self.current_positions = self.default_joints.copy()

            if hasattr(self, 'status_label') and self.status_label is not None:
                self.status_label.config(text="Status: Moving to default positions")

            self.get_logger().info("Reset to default positions")
        except Exception as e:
            self.get_logger().error(f"Error in reset_to_default: {e}")

    def zero_all_joints(self):
        """Set all joints to zero position"""
        try:
            zero_positions = [0.0] * len(self.joint_names)
            self.target_positions = zero_positions.copy()

            # Update sliders
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in self.sliders:
                    self.sliders[joint_name].set(0.0)
                    self.value_labels[joint_name].config(text="0.000")

            if self.smooth_movement_var.get():
                self.start_smooth_trajectory()
            else:
                self.current_positions = zero_positions.copy()

            if hasattr(self, 'status_label') and self.status_label is not None:
                self.status_label.config(text="Status: Moving to zero position")

            self.get_logger().info("All joints moving to zero")
        except Exception as e:
            self.get_logger().error(f"Error in zero_all_joints: {e}")

    def go_to_home(self):
        """Go to a safe home position"""
        try:
            # Safe home positions for your robot
            home_positions = [0.0, -0.5, -1.0, 0.0, -0.5, 0.0, 0.0, 0.0]

            # Ensure positions are within limits
            for i, (joint_name, home_pos) in enumerate(zip(self.joint_names, home_positions)):
                min_val, max_val = self.joint_limits[joint_name]
                home_positions[i] = max(min_val, min(max_val, home_pos))

            self.target_positions = home_positions.copy()

            # Update sliders
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in self.sliders:
                    self.sliders[joint_name].set(home_positions[i])
                    self.value_labels[joint_name].config(text=f"{home_positions[i]:.3f}")

            if self.smooth_movement_var.get():
                self.start_smooth_trajectory()
            else:
                self.current_positions = home_positions.copy()

            if hasattr(self, 'status_label') and self.status_label is not None:
                self.status_label.config(text="Status: Moving to home position")

            self.get_logger().info("Moving to home position")
        except Exception as e:
            self.get_logger().error(f"Error in go_to_home: {e}")

    def preset_position_1(self):
        """Go to preset position 1 - example reaching position"""
        preset_positions = [0.5, -0.8, 1.2, 0.0, -0.5, 0.0, 0.0, 0.0]
        self.set_joint_positions(preset_positions, "Preset Position 1")

    def preset_position_2(self):
        """Go to preset position 2 - example inspection position"""
        preset_positions = [-0.5, -1.0, 0.8, 1.57, 0.3, -0.8, 0.0, 0.0]
        self.set_joint_positions(preset_positions, "Preset Position 2")

    def set_joint_positions(self, positions, status_message):
        """Helper function to set joint positions with smooth movement"""
        try:
            # Ensure positions are within limits
            safe_positions = []
            for i, (joint_name, pos) in enumerate(zip(self.joint_names, positions)):
                min_val, max_val = self.joint_limits[joint_name]
                safe_pos = max(min_val, min(max_val, pos))
                safe_positions.append(safe_pos)

            self.target_positions = safe_positions.copy()

            # Update sliders
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in self.sliders:
                    self.sliders[joint_name].set(safe_positions[i])
                    self.value_labels[joint_name].config(text=f"{safe_positions[i]:.3f}")

            if self.smooth_movement_var.get():
                self.start_smooth_trajectory()
            else:
                self.current_positions = safe_positions.copy()

            if hasattr(self, 'status_label') and self.status_label is not None:
                self.status_label.config(text=f"Status: Moving to {status_message}")

            self.get_logger().info(f"Moving to {status_message}")
        except Exception as e:
            self.get_logger().error(f"Error in set_joint_positions: {e}")

    def update_and_publish(self):
        """Update trajectory and publish current joint state"""
        try:
            # Update trajectory interpolation
            if self.trajectory_active:
                self.interpolate_trajectory()

            # Publish current joint state
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = self.joint_names
            joint_state.position = self.current_positions.copy()

            self.publisher_.publish(joint_state)

        except Exception as e:
            self.get_logger().error(f"Error in update_and_publish: {e}")

    def run_gui(self):
        """Run the GUI main loop"""
        try:
            self.root.mainloop()
        except Exception as e:
            self.get_logger().error(f"Error in GUI main loop: {e}")

def main(args=None):
    rclpy.init(args=args)

    try:
        # Create the smooth robot arm GUI controller
        robot_controller = SmoothRobotArmGUI()

        # Run ROS2 spinning in a separate thread
        ros_thread = threading.Thread(target=lambda: rclpy.spin(robot_controller), daemon=True)
        ros_thread.start()

        # Run the GUI (this will block until the window is closed)
        robot_controller.run_gui()

    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        # Cleanup
        try:
            robot_controller.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == "__main__":
    main()


