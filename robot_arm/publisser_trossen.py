#!/usr/bin/env python3
# Robot Arm Joint Control GUI with Sliders
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tkinter as tk
from tkinter import ttk
import threading
import numpy as np

class RobotArmGUI(Node):
    def __init__(self):
        super().__init__("robot_arm_gui_controller")

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
        # These are typical values for robotic arms - you may need to adjust
        self.joint_limits = {
            "joint_0": (-3.14, 3.14),
            "joint_1": (-1.57, 1.57),
            "joint_2": (-2.09, 2.09),
            "joint_3": (-3.14, 3.14),
            "joint_4": (-1.75, 1.75),
            "joint_5": (-3.14, 3.14),     #
            "left_carriage_joint": (-0.1, 0.1),
            "right_carriage_joint": (-0.1, 0.1),
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

        # Current joint positions
        self.current_positions = self.default_joints.copy()

        # Initialize GUI components dictionaries
        self.sliders = {}
        self.value_labels = {}
        self.status_label = None  # Initialize to None first

        # Create the GUI
        self.setup_gui()

        # Timer for publishing joint states
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_joint_state)

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("Robot Arm Joint Controller")
        self.root.geometry("900x700")

        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)

        # Title
        title_label = ttk.Label(main_frame, text="Robot Arm Joint Controller",
                               font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))

        # Create sliders for each joint
        for i, joint_name in enumerate(self.joint_names):
            row = i + 1

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
        button_frame.grid(row=len(self.joint_names) + 1, column=0, columnspan=3,
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

        # Preset positions frame
        preset_frame = ttk.Frame(main_frame)
        preset_frame.grid(row=len(self.joint_names) + 2, column=0, columnspan=3,
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
        self.status_label.grid(row=len(self.joint_names) + 3, column=0, columnspan=3,
                              pady=(20, 0))

    def cslider_changed(self, joint_index, value):
        """Called when a slider value changes"""
        try:
            float_value = float(value)
            self.current_positions[joint_index] = float_value

            # Update the value label
            joint_name = self.joint_names[joint_index]
            if joint_name in self.value_labels:
                self.value_labels[joint_name].config(text=f"{float_value:.3f}")

            # Update status
            if hasattr(self, 'status_label') and self.status_label is not None:
                display_name = self.joint_display_names.get(joint_name, joint_name)
                self.status_label.config(text=f"Status: {display_name} = {float_value:.3f}")

        except (ValueError, KeyError) as e:
            print(f"Error in slider_changed: {e}")

    def reset_to_default(self):
        """Reset all joints to default positions"""
        for i, (joint_name, default_pos) in enumerate(zip(self.joint_names, self.default_joints)):
            if joint_name in self.sliders and joint_name in self.value_labels:
                self.sliders[joint_name].set(default_pos)
                self.current_positions[i] = default_pos
                self.value_labels[joint_name].config(text=f"{default_pos:.3f}")

        if hasattr(self, 'status_label') and self.status_label is not None:
            self.status_label.config(text="Status: Reset to default positions")

    def zero_all_joints(self):
        """Set all joints to zero position"""
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in self.sliders and joint_name in self.value_labels:
                self.sliders[joint_name].set(0.0)
                self.current_positions[i] = 0.0
                self.value_labels[joint_name].config(text="0.000")

        if hasattr(self, 'status_label') and self.status_label is not None:
            self.status_label.config(text="Status: All joints zeroed")

    def go_to_home(self):
        """Go to a safe home position"""
        # Safe home positions for your robot
        home_positions = [0.0, -0.5, -1.0, 0.0, -0.5, 0.0, 0.0, 0.0]

        for i, (joint_name, home_pos) in enumerate(zip(self.joint_names, home_positions)):
            if joint_name in self.sliders and joint_name in self.value_labels:
                # Check if home position is within limits
                min_val, max_val = self.joint_limits[joint_name]
                safe_pos = max(min_val, min(max_val, home_pos))

                self.sliders[joint_name].set(safe_pos)
                self.current_positions[i] = safe_pos
                self.value_labels[joint_name].config(text=f"{safe_pos:.3f}")

        if hasattr(self, 'status_label') and self.status_label is not None:
            self.status_label.config(text="Status: Moved to home position")

    def preset_position_1(self):
        """Go to preset position 1 - example reaching position"""
        preset_positions = [0.5, -0.8, 1.2, 0.0, -0.5, 0.0, 0.0, 0.0]

        self.set_joint_positions(preset_positions, "Preset Position 1")

    def preset_position_2(self):
        """Go to preset position 2 - example inspection position"""
        preset_positions = [-0.5, -1.0, 0.8, 1.57, 0.3, -0.8, 0.0, 0.0]

        self.set_joint_positions(preset_positions, "Preset Position 2")

    def set_joint_positions(self, positions, status_message):
        """Helper function to set joint positions"""
        for i, (joint_name, pos) in enumerate(zip(self.joint_names, positions)):
            if joint_name in self.sliders and joint_name in self.value_labels:
                # Check if position is within limits
                min_val, max_val = self.joint_limits[joint_name]
                safe_pos = max(min_val, min(max_val, pos))

                self.sliders[joint_name].set(safe_pos)
                self.current_positions[i] = safe_pos
                self.value_labels[joint_name].config(text=f"{safe_pos:.3f}")

        if hasattr(self, 'status_label') and self.status_label is not None:
            self.status_label.config(text=f"Status: {status_message}")

    def publish_joint_state(self):
        """Publish current joint state"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.current_positions

        self.publisher_.publish(joint_state)

    def run_gui(self):
        """Run the GUI main loop"""
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)

    # Create the robot arm GUI controller
    robot_controller = RobotArmGUI()

    # Run ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(robot_controller), daemon=True)
    ros_thread.start()

    try:
        # Run the GUI (this will block until the window is closed)
        robot_controller.run_gui()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()