#!/usr/bin/env python3
"""
Main ROS2 Robot Arm Controller
Integrates GUI, movement controller, and ROS2 communication
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import time

from robot_arm.gui_setup import RobotArmGUISetup
from robot_arm.movement_controller import RobotArmMovementController

class SmoothRobotArmController(Node):
    def __init__(self):
        super().__init__("smooth_robot_arm_gui_controller")

        # Create publisher
        self.publisher_ = self.create_publisher(JointState, "joint_command", 10)

        # Create GUI
        self.gui = RobotArmGUISetup()

        # Create movement controller
        self.movement_controller = RobotArmMovementController(
            joint_names=self.gui.joint_names,
            default_joints=self.gui.default_joints,
            joint_limits=self.gui.joint_limits
        )

        # Setup connections between components
        self._setup_connections()

        # Setup GUI
        self.gui.create_gui()

        # Create ROS2 timers
        timer_period = 0.02  # 50 Hz for smooth interpolation
        self.timer = self.create_timer(timer_period, self._update_and_publish)
        self.command_timer = self.create_timer(0.05, self._process_movement_commands)

        self.get_logger().info("Smooth Robot Arm GUI Controller started")

    def _setup_connections(self):
        """Setup connections between GUI and movement controller"""
        # Connect GUI callbacks to controller methods
        self.gui.on_slider_changed = self._on_slider_changed
        self.gui.on_trajectory_duration_changed = self._on_trajectory_duration_changed
        self.gui.on_reset_to_default = self._on_reset_to_default
        self.gui.on_zero_all_joints = self._on_zero_all_joints
        self.gui.on_go_to_home = self._on_go_to_home
        self.gui.on_stop_movement = self._on_stop_movement
        self.gui.on_preset_position_1 = self._on_preset_position_1
        self.gui.on_preset_position_2 = self._on_preset_position_2

        # Connect movement controller status callback to GUI
        self.movement_controller.set_status_callback(self.gui.update_status)

    # GUI Event Handlers
    def _on_slider_changed(self, joint_index, value):
        """Handle slider value changes"""
        try:
            float_value = float(value)
            joint_name = self.gui.joint_names[joint_index]

            # Update GUI display
            self.gui.update_slider_value(joint_name, float_value)

            # Queue movement command
            self.movement_controller.queue_joint_movement(joint_index, float_value)

        except (ValueError, KeyError) as e:
            self.get_logger().error(f"Error in slider_changed: {e}")

    def _on_trajectory_duration_changed(self, value):
        """Handle trajectory duration changes"""
        duration = float(value)
        self.movement_controller.update_trajectory_duration(duration)
        self.gui.update_speed_label(duration)

    def _on_reset_to_default(self):
        """Handle reset to default positions"""
        try:
            smooth_enabled = self.gui.get_smooth_movement_enabled()
            positions = self.movement_controller.reset_to_default(smooth_enabled)

            if positions:
                # Update GUI sliders
                for i, joint_name in enumerate(self.gui.joint_names):
                    self.gui.set_slider_position(joint_name, positions[i])
                    self.gui.update_slider_value(joint_name, positions[i])

            self.get_logger().info("Reset to default positions")
        except Exception as e:
            self.get_logger().error(f"Error in reset_to_default: {e}")

    def _on_zero_all_joints(self):
        """Handle zero all joints"""
        try:
            smooth_enabled = self.gui.get_smooth_movement_enabled()
            positions = self.movement_controller.zero_all_joints(smooth_enabled)

            if positions:
                # Update GUI sliders
                for i, joint_name in enumerate(self.gui.joint_names):
                    self.gui.set_slider_position(joint_name, positions[i])
                    self.gui.update_slider_value(joint_name, positions[i])

            self.get_logger().info("All joints moving to zero")
        except Exception as e:
            self.get_logger().error(f"Error in zero_all_joints: {e}")

    def _on_go_to_home(self):
        """Handle go to home position"""
        try:
            smooth_enabled = self.gui.get_smooth_movement_enabled()
            positions = self.movement_controller.go_to_home(smooth_enabled)

            if positions:
                # Update GUI sliders
                for i, joint_name in enumerate(self.gui.joint_names):
                    self.gui.set_slider_position(joint_name, positions[i])
                    self.gui.update_slider_value(joint_name, positions[i])

            self.get_logger().info("Moving to home position")
        except Exception as e:
            self.get_logger().error(f"Error in go_to_home: {e}")

    def _on_stop_movement(self):
        """Handle stop movement"""
        try:
            current_positions = self.movement_controller.stop_movement()

            # Update GUI sliders to current positions
            for i, joint_name in enumerate(self.gui.joint_names):
                self.gui.set_slider_position(joint_name, current_positions[i])
                self.gui.update_slider_value(joint_name, current_positions[i])

            self.get_logger().info("Movement stopped by user")
        except Exception as e:
            self.get_logger().error(f"Error in stop_movement: {e}")

    def _on_preset_position_1(self):
        """Handle preset position 1"""
        try:
            smooth_enabled = self.gui.get_smooth_movement_enabled()
            positions = self.movement_controller.go_to_preset_1(smooth_enabled)

            if positions:
                # Update GUI sliders
                for i, joint_name in enumerate(self.gui.joint_names):
                    self.gui.set_slider_position(joint_name, positions[i])
                    self.gui.update_slider_value(joint_name, positions[i])

            self.get_logger().info("Moving to preset position 1")
        except Exception as e:
            self.get_logger().error(f"Error in preset_position_1: {e}")

    def _on_preset_position_2(self):
        """Handle preset position 2"""
        try:
            smooth_enabled = self.gui.get_smooth_movement_enabled()
            positions = self.movement_controller.go_to_preset_2(smooth_enabled)

            if positions:
                # Update GUI sliders
                for i, joint_name in enumerate(self.gui.joint_names):
                    self.gui.set_slider_position(joint_name, positions[i])
                    self.gui.update_slider_value(joint_name, positions[i])

            self.get_logger().info("Moving to preset position 2")
        except Exception as e:
            self.get_logger().error(f"Error in preset_position_2: {e}")

    # ROS2 Timer Callbacks
    def _process_movement_commands(self):
        """Process queued movement commands"""
        try:
            smooth_enabled = self.gui.get_smooth_movement_enabled()
            self.movement_controller.process_movement_commands(smooth_enabled)
        except Exception as e:
            self.get_logger().error(f"Error processing movement commands: {e}")

    def _update_and_publish(self):
        """Update trajectory and publish current joint state"""
        try:
            # Update trajectory interpolation
            self.movement_controller.interpolate_trajectory()

            # Get current positions
            current_positions = self.movement_controller.get_current_positions()

            # Publish current joint state
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = self.gui.joint_names
            joint_state.position = current_positions

            self.publisher_.publish(joint_state)

        except Exception as e:
            self.get_logger().error(f"Error in update_and_publish: {e}")

    def run_gui(self):
        """Run the GUI main loop"""
        try:
            self.gui.run()
        except Exception as e:
            self.get_logger().error(f"Error in GUI main loop: {e}")


def main(args=None):
    rclpy.init(args=args)

    try:
        # Create the robot arm controller
        robot_controller = SmoothRobotArmController()

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