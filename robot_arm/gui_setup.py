#!/usr/bin/env python3
"""
GUI Setup Class for Robot Arm Controller
Handles all GUI creation and layout
"""

import tkinter as tk
from tkinter import ttk

class RobotArmGUISetup:
    def __init__(self):
        # Joint configuration
        self.joint_names = [
            "joint_0", "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "left_carriage_joint", "right_carriage_joint"
        ]

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

        self.joint_limits = {
            "joint_0": (-3.14, 3.14),
            "joint_1": (0, 1.57),
            "joint_2": (0, 2.09),
            "joint_3": (-1.57, 1.57),
            "joint_4": (-1.75, 1.75),
            "joint_5": (-3.14, 3.14),
            "left_carriage_joint": (-0.04, 0.04),
            "right_carriage_joint": (-0.1, 0.1),
        }

        self.default_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # GUI components
        self.root = None
        self.sliders = {}
        self.value_labels = {}
        self.status_label = None
        self.smooth_movement_var = None
        self.speed_scale = None
        self.speed_label = None

        # Callback placeholders - will be set by the controller
        self.on_slider_changed = None
        self.on_trajectory_duration_changed = None
        self.on_reset_to_default = None
        self.on_zero_all_joints = None
        self.on_go_to_home = None
        self.on_stop_movement = None
        self.on_preset_position_1 = None
        self.on_preset_position_2 = None

    def create_gui(self):
        """Create and setup the GUI"""
        self.root = tk.Tk()
        self.root.title("Smooth Robot Arm Joint Controller")
        self.root.geometry("950x750")

        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)

        # Title
        self._create_title(main_frame)

        # Settings frame
        self._create_settings_frame(main_frame)

        # Joint sliders
        self._create_joint_sliders(main_frame)

        # Control buttons
        self._create_control_buttons(main_frame)

        # Preset buttons
        self._create_preset_buttons(main_frame)

        # Status label
        self._create_status_label(main_frame)

    def _create_title(self, parent):
        """Create the title label"""
        title_label = ttk.Label(parent, text="Smooth Robot Arm Joint Controller",
                               font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))

    def _create_settings_frame(self, parent):
        """Create the movement settings frame"""
        settings_frame = ttk.LabelFrame(parent, text="Movement Settings", padding="5")
        settings_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))

        # Smooth movement checkbox
        self.smooth_movement_var = tk.BooleanVar(value=True)
        smooth_checkbox = ttk.Checkbutton(settings_frame, text="Enable Smooth Movement",
                                         variable=self.smooth_movement_var)
        smooth_checkbox.grid(row=0, column=0, sticky=tk.W, padx=(0, 20))

        # Speed control
        ttk.Label(settings_frame, text="Movement Speed:").grid(row=0, column=1, sticky=tk.W, padx=(0, 5))
        self.speed_scale = ttk.Scale(settings_frame, from_=0.5, to=5.0, orient=tk.HORIZONTAL,
                                    length=200, command=self._on_speed_changed)
        self.speed_scale.set(2.0)
        self.speed_scale.grid(row=0, column=2, sticky=tk.W, padx=(5, 10))

        self.speed_label = ttk.Label(settings_frame, text="2.0s")
        self.speed_label.grid(row=0, column=3, sticky=tk.W)

    def _create_joint_sliders(self, parent):
        """Create joint control sliders"""
        for i, joint_name in enumerate(self.joint_names):
            row = i + 2

            display_name = self.joint_display_names.get(joint_name, joint_name)
            name_label = ttk.Label(parent, text=display_name, width=20)
            name_label.grid(row=row, column=0, sticky=tk.W, padx=(0, 10), pady=2)

            min_val, max_val = self.joint_limits[joint_name]

            slider = ttk.Scale(parent, from_=min_val, to=max_val,
                              orient=tk.HORIZONTAL, length=400,
                              command=lambda val, idx=i: self._on_slider_changed(idx, val))
            slider.set(self.default_joints[i])
            slider.grid(row=row, column=1, sticky=(tk.W, tk.E), padx=10, pady=2)

            value_label = ttk.Label(parent, text=f"{self.default_joints[i]:.3f}", width=8)
            value_label.grid(row=row, column=2, sticky=tk.W, padx=(10, 0), pady=2)

            self.sliders[joint_name] = slider
            self.value_labels[joint_name] = value_label

    def _create_control_buttons(self, parent):
        """Create control buttons"""
        button_frame = ttk.Frame(parent)
        button_frame.grid(row=len(self.joint_names) + 2, column=0, columnspan=3,
                         pady=(20, 0), sticky=(tk.W, tk.E))

        buttons = [
            ("Reset to Default", self._on_reset_clicked),
            ("Zero All Joints", self._on_zero_clicked),
            ("Home Position", self._on_home_clicked),
            ("STOP MOVEMENT", self._on_stop_clicked)
        ]

        for i, (text, command) in enumerate(buttons):
            btn = ttk.Button(button_frame, text=text, command=command)
            btn.pack(side=tk.LEFT, padx=(0 if i == 0 else 10, 10 if i == len(buttons)-1 else 0))

    def _create_preset_buttons(self, parent):
        """Create preset position buttons"""
        preset_frame = ttk.Frame(parent)
        preset_frame.grid(row=len(self.joint_names) + 3, column=0, columnspan=3,
                         pady=(10, 0), sticky=(tk.W, tk.E))

        preset1_button = ttk.Button(preset_frame, text="Preset 1", command=self._on_preset1_clicked)
        preset1_button.pack(side=tk.LEFT, padx=10)

        preset2_button = ttk.Button(preset_frame, text="Preset 2", command=self._on_preset2_clicked)
        preset2_button.pack(side=tk.LEFT, padx=10)

    def _create_status_label(self, parent):
        """Create status label"""
        self.status_label = ttk.Label(parent, text="Status: Ready", font=("Arial", 10))
        self.status_label.grid(row=len(self.joint_names) + 4, column=0, columnspan=3, pady=(20, 0))

    # Callback wrapper methods
    def _on_slider_changed(self, joint_idx, value):
        if self.on_slider_changed:
            self.on_slider_changed(joint_idx, value)

    def _on_speed_changed(self, value):
        if self.on_trajectory_duration_changed:
            self.on_trajectory_duration_changed(value)

    def _on_reset_clicked(self):
        if self.on_reset_to_default:
            self.on_reset_to_default()

    def _on_zero_clicked(self):
        if self.on_zero_all_joints:
            self.on_zero_all_joints()

    def _on_home_clicked(self):
        if self.on_go_to_home:
            self.on_go_to_home()

    def _on_stop_clicked(self):
        if self.on_stop_movement:
            self.on_stop_movement()

    def _on_preset1_clicked(self):
        if self.on_preset_position_1:
            self.on_preset_position_1()

    def _on_preset2_clicked(self):
        if self.on_preset_position_2:
            self.on_preset_position_2()

    def update_slider_value(self, joint_name, value):
        """Update a slider's displayed value"""
        if joint_name in self.value_labels:
            self.value_labels[joint_name].config(text=f"{value:.3f}")

    def update_status(self, message):
        """Update the status label"""
        if self.status_label:
            self.status_label.config(text=f"Status: {message}")

    def update_speed_label(self, duration):
        """Update the speed label"""
        if self.speed_label:
            self.speed_label.config(text=f"{duration:.1f}s")

    def set_slider_position(self, joint_name, position):
        """Set a slider to a specific position"""
        if joint_name in self.sliders:
            self.sliders[joint_name].set(position)

    def get_smooth_movement_enabled(self):
        """Check if smooth movement is enabled"""
        return self.smooth_movement_var.get() if self.smooth_movement_var else False

    def run(self):
        """Start the GUI main loop"""
        if self.root:
            self.root.mainloop()