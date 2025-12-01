"""
3D Scanner GUI Application
Python GUI for controlling 3D scanner and visualizing scan data

CORRECTED VERSION - HARDWARE SPECIFICATIONS & CALIBRATION:

ROTATION AXIS (GUI X = GRBL X):
- Calibration: 0.1mm GRBL = 10° motor rotation
- Formula: angle = GRBL_X_position × 100
- Full rotation: 360° = 3.6mm GRBL units
- Conversion: Direct value (no scaling needed)

HEIGHT AXIS (GUI Z = GRBL Y):
- Full step mode: G1 Y0.1 = 45° motor = 1mm actual movement
- M8 lead screw: 1 motor revolution (360°) = 8mm
- Formula: actual_mm = GRBL_Y_value × 10
- Conversion when sending: GRBL_Y = mm / 10
- Conversion when parsing: mm = GRBL_Y × 10

COORDINATE MAPPING:
- format_gcode_command(): GUI X → GRBL X (direct), GUI Z → GRBL Y (÷10)
- parse_grbl_status(): GRBL X → GUI X (direct), GRBL Y → GUI Z (×10)
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import serial
import serial.tools.list_ports
import threading
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import time
import os
from datetime import datetime  # Added for .k file export

class ScannerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("3D Scanner Control")
        self.root.geometry("1000x600")

        # Serial connection
        self.serial_conn = None
        self.is_connected = False
        self.is_scanning = False
        self.scan_data = []
        self.current_layer = 0
        self.current_step = 0
        self.current_angle = 0.0  # Current rotation angle in degrees
        self.step_by_step_active = False  # Step-by-step scan mode
        self.scan_step_thread = None  # Thread for sending SCAN_STEP commands
        self.min_points_for_export = 1  # Minimum points required to enable export (allow export with any points)
        self.current_vl53_distance = None  # Current VL53L0X/VL53L1 distance reading in mm
        self.vl53_sensor_type = "VL53L1"  # Sensor type: "VL53L0X" or "VL53L1" (default: VL53L1)
        self.scan_paused = False
        self.scan_start_angle = 0.0  # Starting angle for current rotation

        # Position tracking - CORRECTED CALIBRATION
        # Rotation axis (GRBL X → GUI X): 0.1mm GRBL = 10°, direct value
        # Height axis (GRBL Y → GUI Z): Full step mode, GRBL value × 10 = actual mm
        self.current_x_pos = 0.0  # Rotation position in mm (from GRBL X, direct value)
        self.current_y_pos = 0.0  # Height position in mm (from GRBL Y × 10, actual measurement)
        self.current_z_pos = 0.0  # GRBL Z position in mm (unused)
        self.grbl_state = "Idle"  # GRBL state

        # Create GUI
        self.create_widgets()

    def create_widgets(self):
        # Main container - horizontal layout
        main_frame = ttk.Frame(self.root, padding="5")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Create notebook for tabs
        notebook = ttk.Notebook(main_frame)
        notebook.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Tab 1: Control Panel
        control_tab = ttk.Frame(notebook, padding="5")
        notebook.add(control_tab, text="Control")

        # Tab 2: Test
        test_tab = ttk.Frame(notebook, padding="5")
        notebook.add(test_tab, text="Test")

        # Left panel - Controls (in Control tab)
        control_frame = ttk.LabelFrame(control_tab, text="Control Panel", padding="5")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)

        # Right panel - 3D Visualization (shared)
        viz_frame = ttk.LabelFrame(main_frame, text="3D Visualization", padding="5")
        viz_frame.grid(row=0, column=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)

        # Port selection (in Control tab)
        ttk.Label(control_frame, text="Port:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(control_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, sticky=(tk.W, tk.E), pady=2, padx=2)
        self.refresh_ports()

        ttk.Button(control_frame, text="Refresh", command=self.refresh_ports, width=8).grid(row=0, column=2, padx=2)

        # Connect/Disconnect button
        self.connect_btn = ttk.Button(control_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=1, column=0, columnspan=3, pady=5, sticky=(tk.W, tk.E))

        # Status label
        self.status_label = ttk.Label(control_frame, text="Status: Disconnected", foreground="red")
        self.status_label.grid(row=2, column=0, columnspan=3, pady=2)

        # Test tab content
        self.setup_test_tab(test_tab)

        # Track test points
        self.test_points = []
        self.is_testing = False
        self.vl53_reading_active = False
        self.z_layer_test_active = False  # Z layer test mode
        self.z_layer_test_data = []  # Store (z_height, distance) pairs

        # Geometry parameters
        geometry_frame = ttk.LabelFrame(control_frame, text="Thông số hình học", padding="5")
        geometry_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=3)

        ttk.Label(geometry_frame, text="Khoảng cách tâm (cm):").grid(row=0, column=0, sticky=tk.W, pady=1)
        self.center_distance_var = tk.StringVar(value="16.5")
        ttk.Entry(geometry_frame, textvariable=self.center_distance_var, width=8).grid(row=0, column=1, sticky=tk.W, padx=2)

        ttk.Label(geometry_frame, text="Bán kính đĩa (cm):").grid(row=0, column=2, sticky=tk.W, padx=(10,0), pady=1)
        self.disk_radius_var = tk.StringVar(value="5.0")
        ttk.Entry(geometry_frame, textvariable=self.disk_radius_var, width=8).grid(row=0, column=3, sticky=tk.W, padx=2)
        
        # VL53L0X/VL53L1 offset calibration (mm)
        ttk.Label(geometry_frame, text="VL53 Offset (mm):").grid(row=3, column=0, sticky=tk.W, pady=1)
        self.vl53_offset_var = tk.StringVar(value="0.0")
        ttk.Entry(geometry_frame, textvariable=self.vl53_offset_var, width=8).grid(row=3, column=1, sticky=tk.W, padx=2)

        ttk.Label(geometry_frame, text="Số điểm scan/vòng:").grid(row=1, column=0, sticky=tk.W, pady=1)
        self.points_per_revolution_var = tk.StringVar(value="36")
        ttk.Entry(geometry_frame, textvariable=self.points_per_revolution_var, width=8).grid(row=1, column=1, sticky=tk.W, padx=2)

        ttk.Label(geometry_frame, text="Chiều cao tối đa (mm):").grid(row=1, column=2, sticky=tk.W, padx=(10,0), pady=1)
        self.z_travel_var = tk.StringVar(value="100")
        ttk.Entry(geometry_frame, textvariable=self.z_travel_var, width=8).grid(row=1, column=3, sticky=tk.W, padx=2)
        
        ttk.Label(geometry_frame, text="Layer height scan (mm):").grid(row=2, column=2, sticky=tk.W, padx=(10,0), pady=1)
        self.scan_layer_height_var = tk.StringVar(value="2.0")
        ttk.Entry(geometry_frame, textvariable=self.scan_layer_height_var, width=8).grid(row=2, column=3, sticky=tk.W, padx=2)

        # Scan controls
        scan_frame = ttk.Frame(control_frame)
        scan_frame.grid(row=4, column=0, columnspan=3, pady=5, sticky=(tk.W, tk.E))

        self.scan_up_btn = ttk.Button(scan_frame, text="Scan ↑ (Dưới→Trên)", command=self.start_scan_up, state=tk.DISABLED)
        self.scan_up_btn.grid(row=0, column=0, padx=2, sticky=(tk.W, tk.E))

        self.scan_down_btn = ttk.Button(scan_frame, text="Scan ↓ (Trên→Dưới)", command=self.start_scan_down, state=tk.DISABLED)
        self.scan_down_btn.grid(row=0, column=1, padx=2, sticky=(tk.W, tk.E))

        self.pause_btn = ttk.Button(scan_frame, text="Tạm dừng", command=self.pause_scan, state=tk.DISABLED)
        self.pause_btn.grid(row=0, column=2, padx=2, sticky=(tk.W, tk.E))

        self.resume_btn = ttk.Button(scan_frame, text="Tiếp tục", command=self.resume_scan, state=tk.DISABLED)
        self.resume_btn.grid(row=0, column=3, padx=2, sticky=(tk.W, tk.E))

        self.clear_btn = ttk.Button(scan_frame, text="Clear", command=self.clear_data, state=tk.NORMAL)
        self.clear_btn.grid(row=1, column=0, columnspan=4, padx=2, pady=(5,0), sticky=(tk.W, tk.E))

        scan_frame.columnconfigure(0, weight=1)
        scan_frame.columnconfigure(1, weight=1)

        # Manual control buttons
        manual_frame = ttk.LabelFrame(control_frame, text="Manual Control", padding="5")
        manual_frame.grid(row=5, column=0, columnspan=3, pady=5, sticky=(tk.W, tk.E))

        self.move_to_top_btn = ttk.Button(manual_frame, text="Lên đỉnh", command=self.move_to_top, state=tk.DISABLED)
        self.move_to_top_btn.grid(row=0, column=0, padx=2, sticky=(tk.W, tk.E))

        self.home_btn = ttk.Button(manual_frame, text="Về Home", command=self.go_home, state=tk.DISABLED)
        self.home_btn.grid(row=0, column=1, padx=2, sticky=(tk.W, tk.E))

        manual_frame.columnconfigure(0, weight=1)
        manual_frame.columnconfigure(1, weight=1)
        scan_frame.columnconfigure(2, weight=1)

        # Progress
        progress_frame = ttk.Frame(control_frame)
        progress_frame.grid(row=6, column=0, columnspan=3, pady=2, sticky=(tk.W, tk.E))

        ttk.Label(progress_frame, text="Progress:").grid(row=0, column=0, sticky=tk.W)
        self.progress_var = tk.StringVar(value="0%")
        ttk.Label(progress_frame, textvariable=self.progress_var).grid(row=0, column=1, sticky=tk.W, padx=5)

        # Current angle display
        ttk.Label(progress_frame, text="Góc quay:").grid(row=1, column=0, sticky=tk.W, pady=(3,0))
        self.current_angle_var = tk.StringVar(value="0.0°")
        ttk.Label(progress_frame, textvariable=self.current_angle_var).grid(row=1, column=1, sticky=tk.W, padx=5, pady=(3,0))

        self.progress_bar = ttk.Progressbar(control_frame, mode='determinate')
        self.progress_bar.grid(row=6, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=2)

        # Export buttons
        export_frame = ttk.Frame(control_frame)
        export_frame.grid(row=7, column=0, columnspan=3, pady=3, sticky=(tk.W, tk.E))

        self.export_stl_btn = ttk.Button(export_frame, text="Export STL", command=self.export_stl, state=tk.NORMAL)
        self.export_stl_btn.grid(row=0, column=0, padx=2, sticky=(tk.W, tk.E))

        self.export_k_btn = ttk.Button(export_frame, text="Export .k", command=self.export_k, state=tk.NORMAL)
        self.export_k_btn.grid(row=0, column=1, padx=2, sticky=(tk.W, tk.E))

        self.export_step_btn = ttk.Button(export_frame, text="Export STEP", command=self.export_step, state=tk.NORMAL)
        self.export_step_btn.grid(row=1, column=0, columnspan=2, padx=2, pady=(5,0), sticky=(tk.W, tk.E))

        export_frame.columnconfigure(0, weight=1)
        export_frame.columnconfigure(1, weight=1)

        # Data info
        info_frame = ttk.LabelFrame(control_frame, text="Scan Info", padding="5")
        info_frame.grid(row=8, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=3)

        self.info_text = tk.Text(info_frame, height=3, width=35, font=("Consolas", 8))
        self.info_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar = ttk.Scrollbar(info_frame, orient=tk.VERTICAL, command=self.info_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.info_text.configure(yscrollcommand=scrollbar.set)
        info_frame.rowconfigure(0, weight=1)
        info_frame.columnconfigure(0, weight=1)

        # Container frame for canvas and toolbar
        canvas_container = tk.Frame(viz_frame)
        canvas_container.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Matplotlib figure
        self.fig = Figure(figsize=(6, 5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_zlabel('Z (cm)')
        self.ax.set_title('3D Scan Point Cloud')

        self.canvas = FigureCanvasTkAgg(self.fig, canvas_container)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Toolbar
        toolbar = NavigationToolbar2Tk(self.canvas, canvas_container)
        toolbar.update()

        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(2, weight=1)
        main_frame.rowconfigure(0, weight=1)
        control_tab.columnconfigure(0, weight=1)
        control_tab.rowconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)
        control_frame.columnconfigure(3, weight=1)
        control_frame.rowconfigure(8, weight=1)
        scan_frame.columnconfigure(0, weight=1)
        scan_frame.columnconfigure(1, weight=1)
        viz_frame.columnconfigure(0, weight=1)
        viz_frame.rowconfigure(0, weight=1)

        # Bind tab change event
        notebook.bind("<<NotebookTabChanged>>", self.on_tab_changed)

    def setup_test_tab(self, test_tab):
        """Setup Test tab content with GRBL-style controls"""
        # Info label at top
        info_label = ttk.Label(test_tab,
                              text="⚠️ Connect to device in Control tab first, then use this tab to test GRBL firmware features",
                              font=("Arial", 9), foreground="blue")
        info_label.grid(row=0, column=0, sticky=(tk.W, tk.E), padx=5, pady=5)

        # Main container with horizontal layout
        main_test_frame = ttk.Frame(test_tab)
        main_test_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)

        # Left panel: Sliders
        left_panel = ttk.Frame(main_test_frame)
        left_panel.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5)

        # Feed rate slider
        feed_frame = ttk.Frame(left_panel)
        feed_frame.grid(row=0, column=0, pady=10)

        ttk.Label(feed_frame, text="Speed (F)", font=("Arial", 9)).grid(row=0, column=0, pady=2)
        self.speed_var = tk.DoubleVar(value=500.0)
        speed_slider = tk.Scale(feed_frame, from_=1, to=2000, resolution=1, orient=tk.VERTICAL,
                               variable=self.speed_var, length=200,
                               command=self.on_speed_change)
        speed_slider.grid(row=1, column=0)
        self.speed_label = ttk.Label(feed_frame, text="F500", font=("Arial", 10, "bold"))
        self.speed_label.grid(row=2, column=0, pady=2)

        # Distance slider
        dist_frame = ttk.Frame(left_panel)
        dist_frame.grid(row=0, column=1, padx=20, pady=10)

        ttk.Label(dist_frame, text="Step (mm)", font=("Arial", 9)).grid(row=0, column=0, pady=2)
        self.step_var = tk.DoubleVar(value=0.1)
        step_slider = tk.Scale(dist_frame, from_=0.1, to=500, resolution=0.1, orient=tk.VERTICAL,
                               variable=self.step_var, length=200,
                               command=self.on_step_change)
        step_slider.grid(row=1, column=0)
        self.step_label = ttk.Label(dist_frame, text="0.1", font=("Arial", 10, "bold"))
        self.step_label.grid(row=2, column=0, pady=2)

        # Center panel: Directional controls
        center_panel = ttk.Frame(main_test_frame)
        center_panel.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=20)

        # Create 3x3 grid of directional buttons
        self.direction_buttons = {}
        directions = [
            (0, 0, "NW", "↖", lambda: self.move_direction("NW")),
            (0, 1, "N", "↑", lambda: self.move_direction("N")),
            (0, 2, "NE", "↗", lambda: self.move_direction("NE")),
            (1, 0, "W", "←", lambda: self.move_direction("W")),
            (1, 1, "HOME", "⌂", self.go_home_test),
            (1, 2, "E", "→", lambda: self.move_direction("E")),
            (2, 0, "SW", "↙", lambda: self.move_direction("SW")),
            (2, 1, "S", "↓", lambda: self.move_direction("S")),
            (2, 2, "SE", "↘", lambda: self.move_direction("SE")),
        ]

        for row, col, direction, icon, cmd in directions:
            btn = ttk.Button(center_panel, text=icon, command=cmd, width=4, state=tk.DISABLED)
            btn.grid(row=row, column=col, padx=2, pady=2)
            self.direction_buttons[direction] = btn

        # Right panel: VL53L0X/VL53L1 display
        right_panel = ttk.LabelFrame(main_test_frame, text="VL53 Distance Sensor", padding="10")
        right_panel.grid(row=0, column=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5)

        # Sensor type selection
        ttk.Label(right_panel, text="Sensor Type:", font=("Arial", 9)).grid(row=0, column=0, sticky=tk.W, pady=2)
        self.vl53_sensor_type_var = tk.StringVar(value="VL53L1")
        sensor_type_combo = ttk.Combobox(right_panel, textvariable=self.vl53_sensor_type_var, 
                                        values=["VL53L0X", "VL53L1"], state="readonly", width=10)
        sensor_type_combo.grid(row=0, column=1, sticky=tk.W, padx=2, pady=2)
        sensor_type_combo.bind("<<ComboboxSelected>>", self.on_sensor_type_changed)

        # Large distance display
        self.vl53_distance_var = tk.StringVar(value="--")
        distance_display = ttk.Label(right_panel, textvariable=self.vl53_distance_var,
                                    font=("Arial", 24, "bold"), foreground="blue")
        distance_display.grid(row=1, column=0, pady=10)

        ttk.Label(right_panel, text="mm", font=("Arial", 12)).grid(row=1, column=1, sticky=tk.W, padx=5)

        # Start/Stop reading button
        self.vl53_read_btn = ttk.Button(right_panel, text="Start Reading",
                                        command=self.toggle_vl53_reading, state=tk.DISABLED)
        self.vl53_read_btn.grid(row=2, column=0, columnspan=2, pady=10, sticky=(tk.W, tk.E))

        # Status
        self.vl53_status_var = tk.StringVar(value="Stopped")
        ttk.Label(right_panel, textvariable=self.vl53_status_var, foreground="gray").grid(
            row=3, column=0, columnspan=2, pady=5)

        # Visual distance bar
        self.distance_canvas = tk.Canvas(right_panel, width=200, height=20, bg="lightgray")
        self.distance_canvas.grid(row=4, column=0, columnspan=2, pady=10)
        self.distance_bar = self.distance_canvas.create_rectangle(0, 0, 0, 20, fill="green")

        # Bottom panel: Position display and controls
        bottom_panel = ttk.Frame(test_tab)
        bottom_panel.grid(row=2, column=0, sticky=(tk.W, tk.E), padx=5, pady=5)

        # Current position display - CORRECTED LABELS
        pos_frame = ttk.LabelFrame(bottom_panel, text="Current Position", padding="5")
        pos_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=5)

        # X position displays angle (from GRBL Y axis after swap)
        ttk.Label(pos_frame, text="X (Góc quay, 0.1mm=10°):").grid(row=0, column=0, padx=5)
        self.test_x_pos_var = tk.StringVar(value="0.0°")
        ttk.Label(pos_frame, textvariable=self.test_x_pos_var, font=("Arial", 10, "bold")).grid(row=0, column=1, padx=5)

        # Z position displays height (from GRBL Y axis, M8 lead screw)
        ttk.Label(pos_frame, text="Z (Chiều cao, M8 1rev=8mm):").grid(row=0, column=2, padx=5)
        self.test_z_pos_var = tk.StringVar(value="0.0 mm")
        ttk.Label(pos_frame, textvariable=self.test_z_pos_var, font=("Arial", 10, "bold")).grid(row=0, column=3, padx=5)

        # Axis rotation controls
        axis_frame = ttk.LabelFrame(bottom_panel, text="Axis Rotation Controls", padding="10")
        axis_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=5)

        # X Axis rotation controls
        x_axis_frame = ttk.Frame(axis_frame)
        x_axis_frame.grid(row=0, column=0, padx=10, pady=5, sticky=(tk.W, tk.E))

        ttk.Label(x_axis_frame, text="X Axis (Rotation):", font=("Arial", 10, "bold")).grid(row=0, column=0, columnspan=4, sticky=tk.W, pady=5)

        self.rotate_x_cw_btn = ttk.Button(x_axis_frame, text="X CW ↻",
                                          command=self.rotate_x_cw_test, state=tk.DISABLED, width=12)
        self.rotate_x_cw_btn.grid(row=1, column=0, padx=5, pady=2)

        self.rotate_x_ccw_btn = ttk.Button(x_axis_frame, text="X CCW ↺",
                                           command=self.rotate_x_ccw_test, state=tk.DISABLED, width=12)
        self.rotate_x_ccw_btn.grid(row=1, column=1, padx=5, pady=2)

        self.rotate_x_full_cw_btn = ttk.Button(x_axis_frame, text="X 360° CW",
                                               command=self.rotate_x_full_cw_test, state=tk.DISABLED, width=12)
        self.rotate_x_full_cw_btn.grid(row=1, column=2, padx=5, pady=2)

        self.rotate_x_full_ccw_btn = ttk.Button(x_axis_frame, text="X 360° CCW",
                                                command=self.rotate_x_full_ccw_test, state=tk.DISABLED, width=12)
        self.rotate_x_full_ccw_btn.grid(row=1, column=3, padx=5, pady=2)

        # Z Axis controls (mapped from GRBL Y, vertical movement)
        y_axis_frame = ttk.Frame(axis_frame)
        y_axis_frame.grid(row=1, column=0, padx=10, pady=5, sticky=(tk.W, tk.E))

        ttk.Label(y_axis_frame, text="Z Axis (Chiều cao, M8 leadscrew 1rev=8mm):", font=("Arial", 10, "bold")).grid(row=0, column=0, columnspan=4, sticky=tk.W, pady=5)

        self.rotate_y_cw_btn = ttk.Button(y_axis_frame, text="Z Lên ↑",
                                          command=self.rotate_y_cw_test, state=tk.DISABLED, width=12)
        self.rotate_y_cw_btn.grid(row=1, column=0, padx=5, pady=2)

        self.rotate_y_ccw_btn = ttk.Button(y_axis_frame, text="Z Xuống ↓",
                                           command=self.rotate_y_ccw_test, state=tk.DISABLED, width=12)
        self.rotate_y_ccw_btn.grid(row=1, column=1, padx=5, pady=2)

        self.rotate_y_full_cw_btn = ttk.Button(y_axis_frame, text="Z Lên Hết",
                                               command=self.rotate_y_full_cw_test, state=tk.DISABLED, width=12)
        self.rotate_y_full_cw_btn.grid(row=1, column=2, padx=5, pady=2)

        self.rotate_y_full_ccw_btn = ttk.Button(y_axis_frame, text="Z Xuống Hết",
                                                command=self.rotate_y_full_ccw_test, state=tk.DISABLED, width=12)
        self.rotate_y_full_ccw_btn.grid(row=1, column=3, padx=5, pady=2)

        # Z Layer Test (kiểm tra lệch tâm)
        z_test_frame = ttk.LabelFrame(bottom_panel, text="Z Layer Test - Kiểm tra lệch tâm", padding="5")
        z_test_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # Parameters for Z test
        z_test_params = ttk.Frame(z_test_frame)
        z_test_params.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=2)
        
        ttk.Label(z_test_params, text="Layer height (mm):").grid(row=0, column=0, sticky=tk.W, padx=2)
        self.z_test_layer_height_var = tk.StringVar(value="2.0")
        ttk.Entry(z_test_params, textvariable=self.z_test_layer_height_var, width=8).grid(row=0, column=1, padx=2)
        
        ttk.Label(z_test_params, text="Số lớp:").grid(row=0, column=2, sticky=tk.W, padx=(10,2))
        self.z_test_num_layers_var = tk.StringVar(value="10")
        ttk.Entry(z_test_params, textvariable=self.z_test_num_layers_var, width=8).grid(row=0, column=3, padx=2)
        
        # Start/Stop button
        self.z_test_btn = ttk.Button(z_test_frame, text="Bắt đầu Test Z", 
                                     command=self.toggle_z_layer_test, state=tk.DISABLED)
        self.z_test_btn.grid(row=1, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))
        
        # Results display
        z_results_frame = ttk.Frame(z_test_frame)
        z_results_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        ttk.Label(z_results_frame, text="Kết quả đo (Z height → Distance):", font=("Arial", 9, "bold")).grid(row=0, column=0, sticky=tk.W)
        
        self.z_test_results_text = tk.Text(z_results_frame, height=6, width=50, font=("Consolas", 8), wrap=tk.WORD)
        self.z_test_results_text.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        z_results_scrollbar = ttk.Scrollbar(z_results_frame, orient=tk.VERTICAL, command=self.z_test_results_text.yview)
        z_results_scrollbar.grid(row=1, column=1, sticky=(tk.N, tk.S))
        self.z_test_results_text.configure(yscrollcommand=z_results_scrollbar.set)
        
        z_test_frame.columnconfigure(0, weight=1)
        z_results_frame.columnconfigure(0, weight=1)
        z_results_frame.rowconfigure(1, weight=1)

        # Serial communication log
        log_frame = ttk.LabelFrame(bottom_panel, text="Serial Communication Log", padding="5")
        log_frame.grid(row=4, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)

        self.serial_log_text = tk.Text(log_frame, height=8, width=80, font=("Consolas", 9), wrap=tk.WORD)
        self.serial_log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        serial_log_scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.serial_log_text.yview)
        serial_log_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.serial_log_text.configure(yscrollcommand=serial_log_scrollbar.set)

        # Clear log button
        clear_log_btn = ttk.Button(log_frame, text="Clear Log", command=self.clear_serial_log, width=12)
        clear_log_btn.grid(row=1, column=0, pady=5, sticky=tk.W)

        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)

        # Configure grid weights
        test_tab.columnconfigure(0, weight=1)
        test_tab.rowconfigure(1, weight=1)
        test_tab.rowconfigure(2, weight=1)
        main_test_frame.columnconfigure(1, weight=1)
        main_test_frame.rowconfigure(0, weight=1)
        bottom_panel.rowconfigure(4, weight=1)
        bottom_panel.columnconfigure(0, weight=1)

    def on_tab_changed(self, event):
        """Handle tab change"""
        if self.test_points:
            self.test_points = []
            self.update_visualization()

    def refresh_ports(self):
        """Refresh available serial ports"""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        if port_list and not self.port_var.get():
            self.port_var.set(port_list[0])

    def toggle_connection(self):
        """Connect or disconnect from serial port"""
        if not self.is_connected:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "Please select a serial port")
                return

            try:
                self.serial_conn = serial.Serial(port, 115200, timeout=2, write_timeout=2)  # Increased timeout to 2s
                time.sleep(2)
                self.is_connected = True
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text="Status: Connected", foreground="green")

                # ========================================
                # STARTUP BANNER - Display calibration info
                # ========================================
                self.log_info("=" * 60)
                self.log_info("3D SCANNER - CALIBRATION INFO")
                self.log_info("=" * 60)
                self.log_info("ROTATION (GUI X = GRBL X):")
                self.log_info("  • 0.1mm GRBL = 10° motor")
                self.log_info("  • angle = GRBL_X × 100")
                self.log_info("  • 1 full rotation = 3.6mm GRBL")
                self.log_info("")
                self.log_info("HEIGHT (GUI Z = GRBL Y):")
                self.log_info("  • Full step: G1 X0.1 = 45° = 1mm actual")
                self.log_info("  • M8 lead screw: 1 rev = 8mm")
                self.log_info("  • Conversion: mm = GRBL_X × 10")
                self.log_info("=" * 60)

                # Enable buttons
                if hasattr(self, 'scan_up_btn'):
                    self.scan_up_btn.config(state=tk.NORMAL)
                if hasattr(self, 'scan_down_btn'):
                    self.scan_down_btn.config(state=tk.NORMAL)
                if hasattr(self, 'vl53_read_btn'):
                    self.vl53_read_btn.config(state=tk.NORMAL)
                if hasattr(self, 'direction_buttons'):
                    for btn in self.direction_buttons.values():
                        btn.config(state=tk.NORMAL)
                if hasattr(self, 'rotate_x_cw_btn'):
                    self.rotate_x_cw_btn.config(state=tk.NORMAL)
                    self.rotate_x_ccw_btn.config(state=tk.NORMAL)
                    self.rotate_x_full_cw_btn.config(state=tk.NORMAL)
                    self.rotate_x_full_ccw_btn.config(state=tk.NORMAL)
                if hasattr(self, 'rotate_y_cw_btn'):
                    self.rotate_y_cw_btn.config(state=tk.NORMAL)
                    self.rotate_y_ccw_btn.config(state=tk.NORMAL)
                    self.rotate_y_full_cw_btn.config(state=tk.NORMAL)
                    self.rotate_y_full_ccw_btn.config(state=tk.NORMAL)
                if hasattr(self, 'z_test_btn'):
                    self.z_test_btn.config(state=tk.NORMAL)

                # Start serial reading thread
                self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
                self.serial_thread.start()

                # Start status query thread
                self.status_query_active = True
                self.status_query_thread = threading.Thread(target=self.status_query_loop, daemon=True)
                self.status_query_thread.start()

                self.log_info("Connected to " + port)

                # Initialize GRBL
                time.sleep(2)
                if self.serial_conn:
                    self.serial_conn.reset_input_buffer()
                    self.send_serial_command(b"\x18\n", log=True)
                    time.sleep(0.5)
                    self.send_serial_command("?\n", log=True)
                    self.send_serial_command("$X\n", log=True)
                    self.log_info("GRBL initialized - watching for position updates...")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to connect: {str(e)}")
        else:
            self.disconnect()

    def disconnect(self):
        """Disconnect from serial port"""
        if self.is_connected:
            self.status_query_active = False
            if self.is_scanning:
                if self.serial_conn:
                    self.serial_conn.write(b"STOP\n")
                self.is_scanning = False
            self.is_connected = False
            if self.serial_conn:
                self.serial_conn.close()
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="Status: Disconnected", foreground="red")

            # Disable buttons
            if hasattr(self, 'scan_up_btn'):
                self.scan_up_btn.config(state=tk.DISABLED)
            if hasattr(self, 'scan_down_btn'):
                self.scan_down_btn.config(state=tk.DISABLED)
            self.pause_btn.config(state=tk.DISABLED)
            if hasattr(self, 'resume_btn'):
                self.resume_btn.config(state=tk.DISABLED)
            if hasattr(self, 'vl53_read_btn'):
                self.vl53_reading_active = False
                self.vl53_read_btn.config(state=tk.DISABLED)
                self.vl53_read_btn.config(text="Start Reading")
                self.vl53_status_var.set("Stopped")
                if hasattr(self, 'vl53_distance_var'):
                    self.vl53_distance_var.set("--")
            if hasattr(self, 'direction_buttons'):
                for btn in self.direction_buttons.values():
                    btn.config(state=tk.DISABLED)
            if hasattr(self, 'rotate_x_cw_btn'):
                self.rotate_x_cw_btn.config(state=tk.DISABLED)
                self.rotate_x_ccw_btn.config(state=tk.DISABLED)
                self.rotate_x_full_cw_btn.config(state=tk.DISABLED)
                self.rotate_x_full_ccw_btn.config(state=tk.DISABLED)
            if hasattr(self, 'rotate_y_cw_btn'):
                self.rotate_y_cw_btn.config(state=tk.DISABLED)
                self.rotate_y_ccw_btn.config(state=tk.DISABLED)
                self.rotate_y_full_cw_btn.config(state=tk.DISABLED)
                self.rotate_y_full_ccw_btn.config(state=tk.DISABLED)
            if hasattr(self, 'move_to_top_btn'):
                self.move_to_top_btn.config(state=tk.DISABLED)
            if hasattr(self, 'home_btn'):
                self.home_btn.config(state=tk.DISABLED)
            if hasattr(self, 'z_test_btn'):
                self.z_test_btn.config(state=tk.DISABLED)
                if self.z_layer_test_active:
                    self.z_layer_test_active = False
                    self.z_test_btn.config(text="Bắt đầu Test Z")
            self.log_info("Disconnected")

    def send_serial_command(self, command, log=True):
        """Send command to serial and log it"""
        if self.serial_conn:
            try:
                # Clear input buffer before sending to prevent timeout
                if self.serial_conn.in_waiting > 0:
                    self.serial_conn.reset_input_buffer()
                
                # Clear output buffer if it's getting full (prevent write timeout)
                try:
                    if hasattr(self.serial_conn, 'out_waiting') and self.serial_conn.out_waiting > 100:
                        self.serial_conn.reset_output_buffer()
                except:
                    pass  # Some serial implementations don't support out_waiting
                
                if isinstance(command, str):
                    command_bytes = command.encode()
                else:
                    command_bytes = command

                # Send to serial with timeout handling
                try:
                    self.serial_conn.write(command_bytes)
                    self.serial_conn.flush()  # Ensure data is sent immediately
                except serial.SerialTimeoutException:
                    # If write timeout, clear output buffer and retry once
                    try:
                        self.serial_conn.reset_output_buffer()
                        time.sleep(0.1)  # Small delay
                        self.serial_conn.write(command_bytes)
                        self.serial_conn.flush()
                    except Exception as retry_e:
                        raise Exception(f"Write timeout (retry failed): {retry_e}")

                if log:
                    try:
                        cmd_str = command_bytes.decode('utf-8', errors='replace').strip()
                        cmd_str = cmd_str.replace('\x18', '[RESET]').replace('\r', '').replace('\n', '')

                        # ============================================
                        # CONSOLE LOG: Print ALL sent commands
                        # ============================================
                        print(f"[SERIAL TX] {cmd_str}")

                        # Log to GUI
                        self.log_serial_send(cmd_str)
                    except:
                        print(f"[SERIAL TX] [BINARY: {len(command_bytes)} bytes]")
                        self.log_serial_send(f"[BINARY: {len(command_bytes)} bytes]")
            except Exception as e:
                print(f"[ERROR] Failed to send command: {str(e)}")
                self.log_info(f"Error sending command: {str(e)}")

    def status_query_loop(self):
        """Periodically send status query (?) to GRBL"""
        while self.status_query_active and self.is_connected:
            try:
                if self.serial_conn:
                    self.send_serial_command("?\n", log=False)
                time.sleep(0.2)
            except Exception as e:
                if self.status_query_active:
                    self.log_info(f"Status query error: {str(e)}")
                break

    def read_serial(self):
        """Read data from serial port in background thread"""
        while self.is_connected and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # CALLBACK: Log ALL serial data to console
                        self.on_serial_received(line)
                else:
                    # If no data waiting, clear any stale data in buffer periodically
                    time.sleep(0.01)  # Small delay to prevent CPU spinning
            except Exception as e:
                if self.is_connected:
                    self.log_info(f"Serial error: {str(e)}")
                break

    def on_serial_received(self, line):
        """
        CALLBACK 1: Called for EVERY line received from serial port (BEFORE parsing)
        This is where ALL serial data arrives FIRST
        """
        # ============================================
        # CONSOLE LOG: Print RAW received data (BEFORE PARSE)
        # ============================================
        print(f"[RX RAW] {line}")

        # Log to GUI serial log
        self.log_serial_receive(line)

        # Process the data (this will call parse if needed)
        self.process_serial_data(line)

    def on_data_parsed(self, data_type, parsed_data):
        """
        CALLBACK 2: Called AFTER data has been successfully parsed
        This shows what was extracted from the serial data
        """
        # ============================================
        # CONSOLE LOG: Print PARSED data (AFTER PARSE)
        # ============================================
        print(f"[PARSED] Type: {data_type}, Data: {parsed_data}")

        # Log to GUI if needed
        if data_type == "position":
            angle = parsed_data.get('angle', 0)
            z_mm = parsed_data.get('z', 0)  # Z height (actual mm)
            print(f"[RESULT] Position → Angle={angle:.1f}°, Z={z_mm:.1f}mm")
            # This log is usually disabled by threshold check in parse_grbl_status
            # Only shown here for debugging if called directly

    def process_serial_data(self, line):
        """Process incoming serial data from GRBL firmware"""
        # Clear input buffer after processing to prevent overflow
        if self.serial_conn and self.serial_conn.in_waiting > 100:  # If buffer has more than 100 bytes
            try:
                # Read and discard excess data
                while self.serial_conn.in_waiting > 50:
                    self.serial_conn.read(self.serial_conn.in_waiting)
            except:
                pass
        
        # Handle GRBL status reports: <Idle|MPos:0.000,0.000,0.000|FS:0,0>
        if line.startswith("<"):
            # ============================================
            # CONSOLE LOG: Confirm we caught GRBL status
            # ============================================
            print(f"[PROCESS] Detected GRBL status line, calling parse_grbl_status()...")
            self.parse_grbl_status(line)
            return

        # Handle GRBL responses
        if line.startswith("ok"):
            return
        elif line.startswith("error:"):
            self.log_info(f"GRBL Error: {line}")
            return
        elif line.startswith("ALARM:"):
            self.log_info(f"GRBL Alarm: {line}")
            return

        # Handle VL53L0X/VL53L1 distance reading
        # Format: VL53L1_DISTANCE:138 hoặc VL53L0X_DISTANCE:138
        if "VL53L0X" in line.upper() or "VL53L1" in line.upper() or line.startswith("DISTANCE:"):
            try:
                import re
                # Auto-detect sensor type from response and update UI
                if "VL53L1" in line.upper():
                    self.vl53_sensor_type = "VL53L1"
                    # Update UI dropdown if it exists
                    if hasattr(self, 'vl53_sensor_type_var'):
                        self.root.after(0, lambda: self.vl53_sensor_type_var.set("VL53L1"))
                elif "VL53L0X" in line.upper():
                    self.vl53_sensor_type = "VL53L0X"
                    # Update UI dropdown if it exists
                    if hasattr(self, 'vl53_sensor_type_var'):
                        self.root.after(0, lambda: self.vl53_sensor_type_var.set("VL53L0X"))
                
                # Parse format: VL53L1_DISTANCE:138
                if ":" in line:
                    parts = line.split(":")
                    if len(parts) > 1:
                        dist_str = parts[1].strip()
                        # Remove all non-numeric characters except decimal point
                        dist_str = re.sub(r'[^0-9.]', '', dist_str)
                        if dist_str:
                            distance_mm = float(dist_str)
                            self.update_vl53_display(distance_mm)
                            return

                # Fallback: Try to find number with unit (mm/cm)
                match = re.search(r'(\d+\.?\d*)\s*(mm|cm)', line, re.IGNORECASE)
                if match:
                    distance = float(match.group(1))
                    unit = match.group(2).lower()
                    if unit == "cm":
                        distance = distance * 10
                    self.update_vl53_display(distance)
                    return
            except Exception as e:
                # Silently ignore parse errors
                pass
            return

        # Legacy protocol handlers (for backward compatibility with custom firmware)
        # ... (rest of the protocol handling code remains the same as original)

    def parse_grbl_status(self, status_line):
        """Parse GRBL status report and update position display
        CORRECTED CALIBRATION: 0.1mm = 10 degrees, so angle = position_mm × 100
        Mapping: GRBL X -> GUI X (angle), GRBL Y -> GUI Z (height)
        NOTE: format_gcode_command swaps X/Y when sending, so we swap back when parsing

        GRBL 0.9j format: <Idle,MPos:0.100,0.000,0.000,WPos:0.100,0.000,0.000>
        """
        print(f"\n{'='*70}")
        print(f"[PARSE] ▶ START PARSING")
        print(f"[PARSE] Input: {status_line}")

        try:
            # Step 1: Remove < and >
            print(f"[PARSE] Step 1: Removing < and >...")
            clean = status_line.strip("<>")
            print(f"[PARSE] Step 1 result: '{clean}'")

            # Step 2: Find MPos section
            # GRBL 0.9j format: Idle,MPos:X,Y,Z,WPos:X,Y,Z
            print(f"[PARSE] Step 2: Looking for 'MPos:' in string...")

            if "MPos:" not in clean:
                print(f"[PARSE] ✗ ERROR: No 'MPos:' found in status!")
                print(f"{'='*70}\n")
                return

            print(f"[PARSE] Step 2: ✓ Found 'MPos:'")

            # Step 3: Extract state (first part before comma)
            print(f"[PARSE] Step 3: Extracting state...")
            first_comma = clean.find(',')
            if first_comma > 0:
                self.grbl_state = clean[:first_comma]
                print(f"[PARSE] Step 3: State = '{self.grbl_state}'")
            else:
                print(f"[PARSE] Step 3: Warning - no comma found, can't extract state")

            # Step 4: Extract MPos values
            print(f"[PARSE] Step 4: Extracting MPos section...")
            mpos_start = clean.find("MPos:") + 5  # After "MPos:"
            print(f"[PARSE] Step 4: MPos starts at index {mpos_start}")

            # Find end of MPos (either WPos or end of string)
            mpos_end = clean.find(",WPos:", mpos_start)
            if mpos_end == -1:
                mpos_end = len(clean)  # No WPos, go to end

            print(f"[PARSE] Step 4: MPos ends at index {mpos_end}")

            mpos_str = clean[mpos_start:mpos_end]
            print(f"[PARSE] Step 4: MPos string = '{mpos_str}'")

            # Step 5: Split MPos values by comma
            print(f"[PARSE] Step 5: Splitting MPos values by comma...")
            positions = mpos_str.split(",")
            print(f"[PARSE] Step 5: Got {len(positions)} values: {positions}")

            if len(positions) < 2:
                print(f"[PARSE] ✗ ERROR: Need at least 2 positions, got {len(positions)}")
                print(f"{'='*70}\n")
                return

            # Step 6: Parse X and Y
            # NOTE: format_gcode_command swaps X and Y when sending:
            #   GUI X (rotation) → sent as GRBL X (direct value)
            #   GUI Z (height) → sent as GRBL Y (divided by 10)
            # So when parsing:
            #   positions[0] (GRBL X) = rotation (direct mm value)
            #   positions[1] (GRBL Y) = height units → multiply by 10 to get mm
            print(f"[PARSE] Step 6: Parsing position values...")
            try:
                # GRBL X (positions[0]) is rotation (GUI X) - direct value
                x_mm = float(positions[0])
                print(f"[PARSE] Step 6: X (rotation from GRBL X) = {x_mm} mm")
            except ValueError as e:
                print(f"[PARSE] ✗ ERROR parsing X from '{positions[1]}': {e}")
                print(f"{'='*70}\n")
                return

            try:
                # GRBL Y (positions[1]) is height (GUI Z) - need to multiply by 10
                # Because: G1 Y0.1 = 45° motor = 1mm actual movement
                # Formula: mm_actual = GRBL_Y_value × 10
                y_grbl_units = float(positions[1])
                y_mm = y_grbl_units * 10.0  # Convert GRBL units to actual mm
                print(f"[PARSE] Step 6: Y (height from GRBL Y) = {y_grbl_units} units = {y_mm:.1f} mm")
            except ValueError as e:
                print(f"[PARSE] ✗ ERROR parsing Y from '{positions[0]}': {e}")
                print(f"{'='*70}\n")
                return

            # Parse Z if available
            if len(positions) >= 3:
                try:
                    z_mm = float(positions[2])
                    print(f"[PARSE] Step 6: Z = {z_mm} mm")
                    self.current_z_pos = z_mm
                except:
                    print(f"[PARSE] Step 6: Warning - couldn't parse Z")

            # Step 7: Update internal state
            print(f"[PARSE] Step 7: Updating internal variables...")
            self.current_x_pos = x_mm  # Rotation from GRBL X (direct mm)
            self.current_y_pos = y_mm  # Height from GRBL Y (converted to actual mm)
            print(f"[PARSE] Step 7: ✓ Internal state updated")
            print(f"[PARSE] Step 7: X = {x_mm:.3f}mm (rotation from GRBL X), Y = {y_mm:.1f}mm (height from GRBL Y)")

            # Step 8: Calculate angle from X position
            print(f"[PARSE] Step 8: Calculating angle from X position...")
            print(f"[PARSE] Step 8: Formula: {x_mm:.3f} mm × 100 = ? degrees")
            angle = x_mm * 100.0
            print(f"[PARSE] Step 8: Raw angle = {angle:.1f}°")

            # Step 9: Normalize to 0-360
            print(f"[PARSE] Step 9: Normalizing angle...")
            angle = angle % 360.0
            if angle < 0:
                angle += 360.0
            print(f"[PARSE] Step 9: ✓ Final angle = {angle:.1f}°")

            self.current_angle = angle

            # Step 10: Calculate Z height (already converted to mm in step 6)
            # M8 lead screw: 1 motor revolution (360°) = 8mm
            # Full step: 45° motor = 1mm, 90° motor = 2mm, etc.
            z_height_mm = y_mm
            print(f"[PARSE] Step 10: Z height = {z_height_mm:.1f}mm (actual measurement)")

            # Step 11: Log result (simplified - only angle for X)
            print(f"[PARSE] Step 11: Logging result...")
            result_msg = f"Angle={angle:.1f}°, Z={z_height_mm:.1f}mm"
            print(f"[RESULT] ✓ {result_msg}")
            # Only log to GUI if significant change (>=1.5mm, close to layer height of 2mm)
            # This prevents spam during slow Z movement between layers
            if not hasattr(self, '_last_logged_z') or abs(z_height_mm - self._last_logged_z) >= 1.5:
                self.log_info(f"✓ Position: {result_msg}")
                self._last_logged_z = z_height_mm

            # Step 12: Update GUI
            print(f"[PARSE] Step 12: Updating GUI display...")
            has_x_var = hasattr(self, 'test_x_pos_var')
            has_z_var = hasattr(self, 'test_z_pos_var')
            print(f"[PARSE] Step 12: Variables exist - X:{has_x_var}, Z:{has_z_var}")

            if has_x_var:
                try:
                    self.test_x_pos_var.set(f"{angle:.1f}°")
                    print(f"[GUI] ✓ X display set to: {angle:.1f}°")
                except Exception as e:
                    print(f"[GUI] ✗ Failed to set X: {e}")

            if has_z_var:
                try:
                    self.test_z_pos_var.set(f"{z_height_mm:.2f} mm")
                    print(f"[GUI] ✓ Z display set to: {z_height_mm:.2f} mm")
                except Exception as e:
                    print(f"[GUI] ✗ Failed to set Z: {e}")

            # Backup update via root.after
            try:
                self.root.after(0, lambda: self._update_position_vars(angle, z_height_mm))
                print(f"[GUI] ✓ Scheduled backup update")
            except Exception as e:
                print(f"[GUI] ✗ Failed to schedule backup: {e}")

            # Call parsed callback
            parsed_data = {
                'x': x_mm,
                'y': y_mm,
                'z': z_height_mm,  # Z height in mm
                'angle': angle,
                'state': self.grbl_state
            }
            self.on_data_parsed('position', parsed_data)

            print(f"{'='*70}")
            print(f"[PARSE] ✓✓✓ PARSE COMPLETE - SUCCESS!")
            print(f"{'='*70}\n")

        except Exception as e:
            print(f"\n{'='*70}")
            print(f"[PARSE] ✗✗✗ EXCEPTION!")
            print(f"[PARSE] Type: {type(e).__name__}")
            print(f"[PARSE] Message: {str(e)}")
            print(f"{'='*70}")

            import traceback
            traceback.print_exc()

            self.log_info(f"ERROR: {str(e)}")
            print(f"{'='*70}\n")

    def update_test_position_display(self):
        """Update position display in test tab
        CORRECTED:
        - X shows rotation angle (from GRBL X): 0.1mm GRBL = 10°, angle = value × 100
        - Z shows height (from GRBL Y): converted to actual mm (GRBL × 10)
        """
        try:
            if hasattr(self, 'test_x_pos_var'):
                # X shows angle (from GRBL X, calculated as GRBL_X × 100)
                self.test_x_pos_var.set(f"{self.current_angle:.1f}°")

            if hasattr(self, 'test_z_pos_var'):
                # Z shows height (from GRBL Y, already converted to actual mm)
                self.test_z_pos_var.set(f"{self.current_y_pos:.1f} mm")
        except Exception as e:
            self.log_info(f"Error updating position display: {str(e)}")

    def _update_position_vars(self, angle, height):
        """Helper function to update position variables (called from main thread)"""
        try:
            if hasattr(self, 'test_x_pos_var'):
                self.test_x_pos_var.set(f"{angle:.1f}°")
            else:
                # Variable not initialized yet
                pass

            if hasattr(self, 'test_z_pos_var'):
                self.test_z_pos_var.set(f"{height:.2f} mm")
            else:
                # Variable not initialized yet
                pass
        except Exception as e:
            # Silently ignore errors during initialization
            pass

    def update_vl53_display(self, distance_mm):
        """Update VL53L0X distance display with offset calibration"""
        # Store current distance reading for scan processing
        if distance_mm > 0 and distance_mm < 8190:
            # Apply offset calibration
            try:
                offset = float(self.vl53_offset_var.get())
                distance_mm = distance_mm + offset
            except:
                offset = 0.0
            
            self.current_vl53_distance = distance_mm
        
        if hasattr(self, 'vl53_distance_var'):
            if distance_mm >= 8190:
                self.vl53_distance_var.set("OUT OF RANGE")
                if hasattr(self, 'vl53_status_var'):
                    self.vl53_status_var.set("Out of range (>2000mm)")
            elif distance_mm == 0:
                self.vl53_distance_var.set("ERROR")
                if hasattr(self, 'vl53_status_var'):
                    self.vl53_status_var.set("Error/Timeout")
            else:
                self.vl53_distance_var.set(f"{distance_mm:.1f} mm")
                if hasattr(self, 'vl53_status_var') and self.vl53_reading_active:
                    if distance_mm < 20:
                        self.vl53_status_var.set("Too close (<20mm)")
                    elif distance_mm > 2000:
                        self.vl53_status_var.set("Too far (>2000mm)")
                    else:
                        self.vl53_status_var.set("OK")

        # Update visual bar
        if hasattr(self, 'distance_canvas'):
            max_distance = 2000.0
            if distance_mm == 0 or distance_mm >= 8190:
                bar_width = 0
                color = "gray"
            else:
                display_distance = min(max(distance_mm, 0), max_distance)
                bar_width = min(200, int((display_distance / max_distance) * 200))
                if distance_mm < 100:
                    color = "red"
                elif distance_mm < 500:
                    color = "orange"
                else:
                    color = "green"
            self.distance_canvas.coords(self.distance_bar, 0, 0, bar_width, 20)
            self.distance_canvas.itemconfig(self.distance_bar, fill=color)

    def format_gcode_command(self, x_move=0.0, y_move=0.0, z_move=0.0, feed_rate=1.0):
        """Format G-code commands

        CALIBRATION:
        - x_move (rotation, GUI X → GRBL X): Direct value, 0.1mm = 10°
        - y_move (height, GUI Z → GRBL Y): Convert mm to GRBL units (÷ 10)
          Example: y_move=2.0mm → G1 Y0.2 → motor 90° → 2mm actual movement
        """
        feed_rate_float = max(1.0, float(feed_rate))
        feed_rate = int(feed_rate_float) if feed_rate_float.is_integer() else feed_rate_float

        commands = []
        commands.append("G91\n")

        move_parts = ["G1"]

        # Rotation axis: GUI X → GRBL X (direct value)
        if abs(x_move) >= 0.0001:  # Reduced threshold for smaller steps
            # Format with enough precision - ensure at least 3 decimal places for small values
            if abs(x_move) < 1.0:
                x_str = f"{x_move:.4f}".rstrip('0').rstrip('.')
            else:
                x_str = f"{x_move:.3f}".rstrip('0').rstrip('.')
            # Ensure we have at least one digit after decimal point for GRBL
            if '.' not in x_str or x_str.endswith('.'):
                x_str = f"{x_move:.4f}".rstrip('0').rstrip('.')
            move_parts.append(f"X{x_str}")

        # Height axis: GUI Z → GRBL Y (convert mm to GRBL units)
        # Formula: GRBL_Y_value = mm / 10
        # Example: 2mm → 0.2, 5mm → 0.5, 10mm → 1.0
        if abs(y_move) >= 0.01:
            y_grbl_units = y_move * 0.1  # Convert mm to GRBL units
            y_str = f"{y_grbl_units:.2f}".rstrip('0').rstrip('.')
            move_parts.append(f"Y{y_str}")

        if isinstance(feed_rate, int) or (isinstance(feed_rate, float) and feed_rate.is_integer()):
            f_str = f"{int(feed_rate)}"
        else:
            f_str = f"{feed_rate:.1f}".rstrip('0').rstrip('.')
        move_parts.append(f"F{f_str}")

        commands.append("".join(move_parts) + "\n")
        commands.append("G90\n")

        return commands

    def send_gcode_commands(self, commands, delay=0.1):
        """Send multiple G-code commands sequentially with delay to prevent buffer overflow"""
        if not self.serial_conn:
            return

        for cmd in commands:
            try:
                self.send_serial_command(cmd, log=True)
                time.sleep(delay)  # Increased delay to prevent timeout
            except Exception as e:
                # Log but continue with remaining commands
                if "timeout" not in str(e).lower():
                    self.log_info(f"Command error (continuing): {e}")

    def on_speed_change(self, value):
        """Update speed label"""
        speed_float = max(1.0, float(value))
        if speed_float.is_integer():
            speed = int(speed_float)
        else:
            speed = speed_float
        self.speed_var.set(speed)
        self.speed_label.config(text=f"F{speed}")

    def on_step_change(self, value):
        """Update step label"""
        step = max(0.1, float(value))
        self.step_var.set(step)
        if isinstance(step, float) and step.is_integer():
            self.step_label.config(text=f"{int(step)}")
        else:
            self.step_label.config(text=f"{step:.1f}")

    def calculate_one_revolution_distance(self):
        """Calculate exact distance for 1 full revolution

        X axis (rotation): 360° = 360/100 mm = 3.6mm
        Because: 0.1mm = 10°, so 1° = 0.1/10 mm, 360° = 360 × (0.1/10) = 3.6mm

        Y axis (height, M8 lead screw): 1 revolution = 8mm
        """
        return 3.6  # Fixed value for X axis based on calibration

    def move_direction(self, direction):
        """Move in specified direction using G-code"""
        if not self.is_connected:
            return

        step = max(0.1, float(self.step_var.get()))
        speed = max(1, float(self.speed_var.get()))
        speed = int(speed) if speed.is_integer() else speed

        if direction == "HOME":
            self.go_home_test()
            return

        x_move = 0.0
        z_move = 0.0

        if direction == "N":
            z_move = step
        elif direction == "S":
            z_move = -step
        elif direction == "E":
            x_move = step
        elif direction == "W":
            x_move = -step
        elif direction == "NE":
            x_move = step * 0.707
            z_move = step * 0.707
        elif direction == "NW":
            x_move = -step * 0.707
            z_move = step * 0.707
        elif direction == "SE":
            x_move = step * 0.707
            z_move = -step * 0.707
        elif direction == "SW":
            x_move = -step * 0.707
            z_move = -step * 0.707

        if x_move != 0 or z_move != 0:
            commands = self.format_gcode_command(x_move=x_move, z_move=z_move, feed_rate=speed)

            if self.serial_conn:
                self.send_gcode_commands(commands, delay=0.05)
                cmd_str = " ".join([c.strip() for c in commands])
                self.log_info(f"Moving {direction}: {cmd_str}")

    def go_home_test(self):
        """Go to home position"""
        if not self.is_connected:
            return

        if self.serial_conn:
            self.send_serial_command("G28\n", log=True)
            self.current_x_pos = 0.0
            self.current_y_pos = 0.0
            self.current_z_pos = 0.0
            self.current_angle = 0.0
            self.update_test_position_display()
            self.log_info("Going to home (G28)")

    def on_sensor_type_changed(self, event=None):
        """Handle sensor type change"""
        self.vl53_sensor_type = self.vl53_sensor_type_var.get()
        if self.vl53_reading_active:
            # Restart reading with new sensor type
            self.vl53_reading_active = False
            time.sleep(0.1)
            self.vl53_reading_active = True
            reading_thread = threading.Thread(target=self.continuous_vl53_read, daemon=True)
            reading_thread.start()

    def toggle_vl53_reading(self):
        """Toggle VL53L0X/VL53L1 reading"""
        if not self.is_connected:
            return

        # Update sensor type from UI
        self.vl53_sensor_type = self.vl53_sensor_type_var.get()

        if not self.vl53_reading_active:
            self.vl53_reading_active = True
            self.vl53_read_btn.config(text="Stop Reading")
            self.vl53_status_var.set(f"Reading {self.vl53_sensor_type}...")

            reading_thread = threading.Thread(target=self.continuous_vl53_read, daemon=True)
            reading_thread.start()
        else:
            self.vl53_reading_active = False
            self.vl53_read_btn.config(text="Start Reading")
            self.vl53_status_var.set("Stopped")

    def continuous_vl53_read(self):
        """Continuously read from VL53L0X or VL53L1"""
        while self.vl53_reading_active and self.is_connected:
            try:
                if self.serial_conn:
                    # Get current sensor type from UI
                    current_type = self.vl53_sensor_type_var.get()
                    # Use sensor type to determine command
                    if current_type == "VL53L1":
                        self.send_serial_command("READ_VL53L1\n", log=True)
                    else:
                        self.send_serial_command("READ_VL53L0X\n", log=True)
                time.sleep(0.2)
            except Exception as e:
                if self.vl53_reading_active:
                    self.vl53_status_var.set(f"Error: {str(e)}")
                break

    def rotate_x_cw_test(self):
        """Rotate X clockwise"""
        if not self.is_connected:
            return

        step = max(0.1, float(self.step_var.get()))
        speed = max(1, float(self.speed_var.get()))

        commands = self.format_gcode_command(x_move=step, feed_rate=speed)
        if self.serial_conn:
            self.send_gcode_commands(commands, delay=0.05)
            cmd_str = " ".join([c.strip() for c in commands])
            self.log_info(f"X CW: {cmd_str}")

    def rotate_x_ccw_test(self):
        """Rotate X counter-clockwise"""
        if not self.is_connected:
            return

        step = max(0.1, float(self.step_var.get()))
        speed = max(1, float(self.speed_var.get()))

        commands = self.format_gcode_command(x_move=-step, feed_rate=speed)
        if self.serial_conn:
            self.send_gcode_commands(commands, delay=0.05)
            cmd_str = " ".join([c.strip() for c in commands])
            self.log_info(f"X CCW: {cmd_str}")

    def rotate_x_full_cw_test(self):
        """Rotate X 360° clockwise"""
        if not self.is_connected:
            return

        speed = max(1, float(self.speed_var.get()))
        distance = self.calculate_one_revolution_distance()

        commands = self.format_gcode_command(x_move=distance, feed_rate=speed)
        if self.serial_conn:
            self.send_gcode_commands(commands, delay=0.05)
            cmd_str = " ".join([c.strip() for c in commands])
            self.log_info(f"X 360° CW (3.6mm): {cmd_str}")

    def rotate_x_full_ccw_test(self):
        """Rotate X 360° counter-clockwise"""
        if not self.is_connected:
            return

        speed = max(1, float(self.speed_var.get()))
        distance = self.calculate_one_revolution_distance()

        commands = self.format_gcode_command(x_move=-distance, feed_rate=speed)
        if self.serial_conn:
            self.send_gcode_commands(commands, delay=0.05)
            cmd_str = " ".join([c.strip() for c in commands])
            self.log_info(f"X 360° CCW (-3.6mm): {cmd_str}")

    def rotate_y_cw_test(self):
        """Move Z up (mapped from GRBL Y)"""
        if not self.is_connected:
            return

        step = max(0.1, float(self.step_var.get()))
        speed = max(1, float(self.speed_var.get()))

        commands = self.format_gcode_command(y_move=step, feed_rate=speed)
        if self.serial_conn:
            self.send_gcode_commands(commands, delay=0.05)
            cmd_str = " ".join([c.strip() for c in commands])
            self.log_info(f"Z Lên: {cmd_str}")

    def rotate_y_ccw_test(self):
        """Move Z down (mapped from GRBL Y)"""
        if not self.is_connected:
            return

        step = max(0.1, float(self.step_var.get()))
        speed = max(1, float(self.speed_var.get()))

        commands = self.format_gcode_command(y_move=-step, feed_rate=speed)
        if self.serial_conn:
            self.send_gcode_commands(commands, delay=0.05)
            cmd_str = " ".join([c.strip() for c in commands])
            self.log_info(f"Z Xuống: {cmd_str}")

    def rotate_y_full_cw_test(self):
        """Move Z full up"""
        if not self.is_connected:
            return

        speed = max(1, float(self.speed_var.get()))
        z_travel = int(self.z_travel_var.get())

        commands = self.format_gcode_command(y_move=z_travel, feed_rate=speed)
        if self.serial_conn:
            self.send_gcode_commands(commands, delay=0.05)
            cmd_str = " ".join([c.strip() for c in commands])
            self.log_info(f"Z Lên Hết: {cmd_str}")

    def rotate_y_full_ccw_test(self):
        """Move Z full down"""
        if not self.is_connected:
            return

        speed = max(1, float(self.speed_var.get()))
        z_travel = int(self.z_travel_var.get())

        commands = self.format_gcode_command(y_move=-z_travel, feed_rate=speed)
        if self.serial_conn:
            self.send_gcode_commands(commands, delay=0.05)
            cmd_str = " ".join([c.strip() for c in commands])
            self.log_info(f"Z Xuống Hết: {cmd_str}")

    def toggle_z_layer_test(self):
        """Toggle Z layer test - chỉ chạy Z và đo khoảng cách ở mỗi lớp"""
        if not self.is_connected:
            return

        if not self.z_layer_test_active:
            # Start test
            self.z_layer_test_active = True
            self.z_test_btn.config(text="Dừng Test Z")
            self.z_layer_test_data = []
            
            # Clear results
            if hasattr(self, 'z_test_results_text'):
                self.z_test_results_text.delete('1.0', tk.END)
                self.z_test_results_text.insert(tk.END, "Bắt đầu test Z...\n")
                self.z_test_results_text.insert(tk.END, "Format: Z (mm) → Distance (mm)\n")
                self.z_test_results_text.insert(tk.END, "=" * 40 + "\n")
            
            # Start test thread
            test_thread = threading.Thread(target=self.z_layer_test_loop, daemon=True)
            test_thread.start()
        else:
            # Stop test
            self.z_layer_test_active = False
            self.z_test_btn.config(text="Bắt đầu Test Z")
            if hasattr(self, 'z_test_results_text'):
                self.z_test_results_text.insert(tk.END, "\nTest đã dừng.\n")

    def z_layer_test_loop(self):
        """Z layer test loop - di chuyển Z qua các lớp và đo khoảng cách"""
        if not self.is_connected or not self.serial_conn:
            return
        
        try:
            # Get parameters
            try:
                layer_height_mm = float(self.z_test_layer_height_var.get())
                num_layers = int(self.z_test_num_layers_var.get())
            except:
                layer_height_mm = 2.0
                num_layers = 10
                self.log_info("Invalid parameters, using defaults: layer_height=2.0mm, num_layers=10")
            
            speed = 1.0  # Slow speed for accuracy
            
            # Get current sensor type
            current_type = self.vl53_sensor_type_var.get() if hasattr(self, 'vl53_sensor_type_var') else self.vl53_sensor_type
            
            self.log_info(f"Bắt đầu Z layer test: {num_layers} lớp, mỗi lớp {layer_height_mm}mm")
            
            # Start from current position
            start_z = self.current_y_pos
            
            for layer in range(num_layers):
                if not self.z_layer_test_active:
                    break
                
                # Get current Z position
                current_z = self.current_y_pos
                
                # Clear old sensor data
                self.current_vl53_distance = None
                
                # Clear serial input buffer more carefully to prevent losing sensor data
                if self.serial_conn:
                    try:
                        # Read any pending data first (this is safer than reset)
                        max_flush_attempts = 5
                        for _ in range(max_flush_attempts):
                            if self.serial_conn.in_waiting > 0:
                                # Read and discard pending data
                                self.serial_conn.read(self.serial_conn.in_waiting)
                                time.sleep(0.05)  # Small delay to let more data arrive if any
                            else:
                                break
                        # Small delay after flushing to ensure buffer is clear
                        time.sleep(0.1)
                    except:
                        pass
                
                # Read sensor
                try:
                    if self.serial_conn:
                        if current_type == "VL53L1":
                            self.send_serial_command("READ_VL53L1\n", log=False)
                        else:
                            self.send_serial_command("READ_VL53L0X\n", log=False)
                        
                        # Small delay after sending command to ensure it's sent
                        time.sleep(0.1)
                    
                    # Wait for sensor reading (increased to 1.5s to handle buffer delays)
                    wait_start = time.time()
                    sensor_data_received = False
                    while time.time() - wait_start < 1.5:
                        if self.current_vl53_distance is not None:
                            if self.current_vl53_distance > 0 and self.current_vl53_distance < 8190:
                                sensor_data_received = True
                                break
                        time.sleep(0.05)
                    
                    if sensor_data_received:
                        distance = self.current_vl53_distance
                        # Store data
                        self.z_layer_test_data.append((current_z, distance))
                        
                        # Update results display
                        result_text = f"Z={current_z:6.2f}mm → Distance={distance:6.1f}mm\n"
                        if hasattr(self, 'z_test_results_text'):
                            self.root.after(0, lambda txt=result_text: self.z_test_results_text.insert(tk.END, txt))
                            self.root.after(0, lambda: self.z_test_results_text.see(tk.END))
                        
                        self.log_info(f"Layer {layer+1}/{num_layers}: Z={current_z:.2f}mm, Distance={distance:.1f}mm")
                    else:
                        result_text = f"Z={current_z:6.2f}mm → ERROR (No data)\n"
                        if hasattr(self, 'z_test_results_text'):
                            self.root.after(0, lambda txt=result_text: self.z_test_results_text.insert(tk.END, txt))
                        self.log_info(f"Layer {layer+1}/{num_layers}: Z={current_z:.2f}mm - No sensor data")
                except Exception as e:
                    self.log_info(f"Error reading sensor at layer {layer+1}: {e}")
                
                if not self.z_layer_test_active:
                    break
                
                # Move Z up by layer height (except for last layer)
                if layer < num_layers - 1:
                    commands = self.format_gcode_command(y_move=layer_height_mm, feed_rate=speed)
                    if self.serial_conn:
                        self.send_gcode_commands(commands, delay=0.1)
                        # Wait for movement to complete
                        time.sleep(0.5)
                        
                        # Wait for Z position to update
                        wait_start = time.time()
                        while time.time() - wait_start < 2.0:
                            if abs(self.current_y_pos - current_z) >= layer_height_mm * 0.5:
                                break
                            time.sleep(0.1)
            
            # Test complete
            self.z_layer_test_active = False
            if hasattr(self, 'z_test_btn'):
                self.root.after(0, lambda: self.z_test_btn.config(text="Bắt đầu Test Z"))
            
            # Summary
            if hasattr(self, 'z_test_results_text'):
                summary = "\n" + "=" * 40 + "\n"
                summary += f"Test hoàn thành: {len(self.z_layer_test_data)} điểm đo\n"
                if len(self.z_layer_test_data) > 0:
                    distances = [d[1] for d in self.z_layer_test_data]
                    min_dist = min(distances)
                    max_dist = max(distances)
                    avg_dist = sum(distances) / len(distances)
                    summary += f"Distance: Min={min_dist:.1f}mm, Max={max_dist:.1f}mm, Avg={avg_dist:.1f}mm\n"
                    summary += f"Chênh lệch: {max_dist - min_dist:.1f}mm\n"
                    if max_dist - min_dist > 5.0:
                        summary += "⚠ Cảnh báo: Chênh lệch lớn (>5mm) - có thể bị lệch tâm!\n"
                    else:
                        summary += "✓ Khoảng cách ổn định - cảm biến có vẻ đúng tâm\n"
                self.root.after(0, lambda txt=summary: self.z_test_results_text.insert(tk.END, txt))
            
            self.log_info("Z layer test hoàn thành")
            
        except Exception as e:
            self.log_info(f"Z layer test error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.z_layer_test_active = False
            if hasattr(self, 'z_test_btn'):
                self.root.after(0, lambda: self.z_test_btn.config(text="Bắt đầu Test Z"))

    def calculate_point_from_scan(self, angle_deg, distance_mm, z_height_mm):
        """Calculate 3D point from scan data
        angle_deg: rotation angle in degrees
        distance_mm: distance from sensor in mm
        z_height_mm: height in mm
        Returns: (x, y, z) in mm or None if filtered out
        """
        try:
            center_distance_cm = float(self.center_distance_var.get())
            disk_radius_cm = float(self.disk_radius_var.get())

            # Convert distance from mm to cm for calculation
            distance_cm = distance_mm / 10.0

            # Filter 1: Remove negative or zero distances (error readings)
            if distance_cm <= 0:
                self.log_info(f"⚠ Filtered point at {angle_deg:.1f}°: Invalid distance ({distance_mm:.1f}mm)")
                return None

            # Filter 2: Calculate valid range
            # Cảm biến ở bên cạnh, cách tâm center_distance_cm
            # Bán kính đĩa disk_radius_cm
            # Khoảng cách tối thiểu: center - radius = 15 - 5 = 10cm
            # Khoảng cách tối đa: center + radius = 15 + 5 = 20cm
            min_distance_cm = center_distance_cm - disk_radius_cm  # 10cm
            max_distance_cm = center_distance_cm + disk_radius_cm  # 20cm

            # Filter 3: Remove values outside valid range
            if distance_cm < min_distance_cm or distance_cm > max_distance_cm:
                self.log_info(f"⚠ Filtered point at {angle_deg:.1f}°: Out of range ({distance_mm:.1f}mm, valid: {min_distance_cm*10:.1f}-{max_distance_cm*10:.1f}mm)")
                return None  # Out of range, skip this point

            # Calculate radius from center
            # Logic from MATLAB: r = centerDistance - distance
            # This gives radius from turntable center to object surface
            radius_from_center = center_distance_cm - distance_cm

            # Filter 4: Remove values around 0 (midThresh in MATLAB)
            # Tắt filter này vì với hình trụ, radius gần 0 là bình thường
            # Chỉ filter các giá trị rất gần 0 (có thể là noise)
            # mid_thresh_upper = 0.2  # Disabled - allow points near center
            # mid_thresh_lower = -0.2  # Disabled - allow points near center
            # if mid_thresh_lower < radius_from_center < mid_thresh_upper:
            #     self.log_info(f"⚠ Filtered point at {angle_deg:.1f}°: Too close to center (radius={radius_from_center:.2f}cm)")
            #     return None  # Too close to center, likely error

            # Convert angle to radians
            angle_rad = np.radians(angle_deg)

            # Calculate x, y in cm (cylindrical coordinates), then convert to mm
            x_cm = radius_from_center * np.cos(angle_rad)
            y_cm = radius_from_center * np.sin(angle_rad)
            x_mm = x_cm * 10.0  # Convert cm to mm for display
            y_mm = y_cm * 10.0  # Convert cm to mm for display

            # Z height: keep in mm (no conversion needed)
            z_mm = z_height_mm

            return (x_mm, y_mm, z_mm)
        except Exception as e:
            self.log_info(f"Error calculating point: {e}")
            return None

    def process_scan_data_point(self):
        """Process current position and sensor reading to create scan point"""
        if self.current_vl53_distance is None:
            return
        
        # Get current angle and height
        angle = self.current_angle
        z_height = self.current_y_pos  # Height from GRBL Y
        
        # Calculate 3D point
        point = self.calculate_point_from_scan(angle, self.current_vl53_distance, z_height)
        if point:
            # Store point with angle and height for later connection
            # Format: (x, y, z, angle, height)
            point_with_meta = point + (angle, z_height)
            self.scan_data.append(point_with_meta)
            # Update visualization in main thread (thread-safe)
            self.root.after(0, self.update_visualization)
            self.log_info(f"Point added: angle={angle:.1f}°, dist={self.current_vl53_distance:.1f}mm, z={z_height:.1f}mm, point=({point[0]:.1f}, {point[1]:.1f}, {point[2]:.1f})mm")

    def scan_rotation_loop(self):
        """Main scan loop: rotate X continuously, read sensor, move Z up after each rotation"""
        if not self.is_connected or not self.serial_conn:
            return
        
        try:
            # Get layer height from UI (separate from test layer height)
            try:
                layer_height_mm = float(self.scan_layer_height_var.get())
                if layer_height_mm <= 0:
                    layer_height_mm = 2.0
                    self.log_info(f"Invalid layer height, using default: {layer_height_mm}mm")
            except:
                layer_height_mm = 2.0  # Default 2mm per layer
                self.log_info(f"Error reading layer height, using default: {layer_height_mm}mm")
            
            # Get number of points per revolution
            try:
                points_per_rev = int(self.points_per_revolution_var.get())
                if points_per_rev < 1:
                    points_per_rev = 36  # Default: 36 points = 10° per step
                    self.log_info(f"Invalid points per revolution, using default: {points_per_rev}")
            except:
                points_per_rev = 36  # Default: 36 points = 10° per step
                self.log_info(f"Error reading points per revolution, using default: {points_per_rev}")
            
            # Calculate angle step per point
            angle_step = 360.0 / points_per_rev
            
            # Calculate one revolution distance (3.6mm for 360 degrees)
            one_rev_distance = self.calculate_one_revolution_distance()
            
            speed = 1.0  # Increased from 1.0 to ensure movement is detected
            
            # Calculate step distance in mm for each angle increment
            # angle_step degrees = one_rev_distance * (angle_step / 360.0)
            step_distance_mm = one_rev_distance * (angle_step / 360.0)

            self.log_info(f"Scan settings: {points_per_rev} points/vòng, {angle_step:.2f}° per point ({step_distance_mm:.3f}mm per step), speed=F{int(speed)}")

            # Track layer number
            layer_number = 0

            # Get max height for scan
            try:
                max_height = float(self.z_travel_var.get())
            except:
                max_height = 100.0  # Default 100mm
                self.log_info(f"Invalid max height, using default: {max_height}mm")
            
            estimated_total_layers = int(max_height / layer_height_mm) + 1
            start_z_position = self.current_y_pos  # Record starting Z position
            
            self.log_info(f"Scan sẽ chạy từ Z={start_z_position:.2f}mm đến Z={start_z_position + max_height:.2f}mm ({estimated_total_layers} lớp)")

            while self.is_scanning and not self.scan_paused:
                # Record starting position
                start_z = self.current_y_pos
                layer_number += 1
                
                # Check if we've reached max height
                current_z_height = self.current_y_pos - start_z_position
                if current_z_height >= max_height:
                    self.log_info(f"Đã đạt chiều cao tối đa: {current_z_height:.2f}mm >= {max_height:.2f}mm. Dừng scan.")
                    break
                
                # Count points in current layer (points with same height as start_z)
                points_in_current_layer = sum(1 for p in self.scan_data 
                                            if len(p) >= 5 and abs(p[4] - start_z) < 0.1)

                # Update window title with layer info
                self.root.title(f"3D Scanner Control - Layer {layer_number}/{estimated_total_layers} at Z={start_z:.2f}mm - Points: {points_in_current_layer}")
                
                self.log_info(f"=== Layer {layer_number}/{estimated_total_layers} at Z={start_z:.2f}mm - Current points: {points_in_current_layer} ===")

                # Step-by-step rotation: Move to each angle, STOP, read sensor, then continue
                points_collected = 0

                for point_num in range(points_per_rev):
                    if not self.is_scanning or self.scan_paused:
                        break

                    # Step 1: Send command to rotate one step (e.g., 10° = 0.1mm)
                    current_angle_before = self.current_angle
                    self.log_info(f"Point {point_num + 1}/{points_per_rev}: Rotating {angle_step:.1f}° from {current_angle_before:.1f}°")

                    # Send movement command (x_move for rotation axis)
                    # Note: format_gcode_command maps x_move to GRBL X (rotation)
                    move_commands = self.format_gcode_command(x_move=step_distance_mm, feed_rate=speed)
                    # Log the actual command being sent
                    cmd_str = " ".join([c.strip() for c in move_commands])
                    self.log_info(f"→ Sending G-code: {cmd_str}")
                    if self.serial_conn:
                        self.send_gcode_commands(move_commands, delay=0.1)

                    # Step 2: Wait for motor to complete movement (motor STOPS at target)
                    # Wait at least 0.5s for movement to complete
                    time.sleep(0.5)

                    # Verify angle has changed
                    movement_timeout = time.time() + 5.0  # Max 5s for movement
                    angle_moved = False
                    while time.time() < movement_timeout:
                        current_angle_after = self.current_angle
                        angle_diff = abs(current_angle_after - current_angle_before)
                        if angle_diff > 180:
                            angle_diff = 360 - angle_diff

                        if angle_diff >= angle_step * 0.3:  # At least 30% of expected movement
                            angle_moved = True
                            break
                        time.sleep(0.1)

                    if not angle_moved:
                        self.log_info(f"⚠ Warning: Angle did not change significantly after movement command")

                    # Step 3: Motor is now STOPPED - Read sensor (single attempt, no retry)
                    # Clear old sensor data to ensure we get FRESH reading
                    self.current_vl53_distance = None
                    
                    # Clear serial input buffer more carefully to prevent losing sensor data
                    # First, read and discard any pending data to avoid clearing data in transit
                    if self.serial_conn:
                        try:
                            # Read any pending data first (this is safer than reset)
                            max_flush_attempts = 5
                            for _ in range(max_flush_attempts):
                                if self.serial_conn.in_waiting > 0:
                                    # Read and discard pending data
                                    self.serial_conn.read(self.serial_conn.in_waiting)
                                    time.sleep(0.05)  # Small delay to let more data arrive if any
                                else:
                                    break
                            # Small delay after flushing to ensure buffer is clear
                            time.sleep(0.1)
                        except:
                            pass
                    
                    # Single attempt to read sensor - if invalid, skip this point
                    try:
                        if self.serial_conn:
                            # Get current sensor type from UI
                            current_type = self.vl53_sensor_type_var.get() if hasattr(self, 'vl53_sensor_type_var') else self.vl53_sensor_type
                            # Use sensor type to determine command
                            if current_type == "VL53L1":
                                self.send_serial_command("READ_VL53L1\n", log=False)
                            else:
                                self.send_serial_command("READ_VL53L0X\n", log=False)
                            
                            # Small delay after sending command to ensure it's sent
                            time.sleep(0.1)

                        # Wait for sensor reading (increased to 1.5s to handle buffer delays)
                        wait_start = time.time()
                        sensor_data_received = False
                        while time.time() - wait_start < 1.5:
                            if self.current_vl53_distance is not None:
                                # Got data - check if valid
                                if self.current_vl53_distance > 0 and self.current_vl53_distance < 8190:
                                    sensor_data_received = True
                                    break
                                else:
                                    # Invalid data, skip this point
                                    break
                            time.sleep(0.05)

                        # Step 4: Process point only if we have valid data
                        if sensor_data_received:
                            self.process_scan_data_point()
                            points_collected += 1
                            
                            # Count current points in this layer
                            current_z = self.current_y_pos
                            points_in_layer = sum(1 for p in self.scan_data 
                                                if len(p) >= 5 and abs(p[4] - current_z) < 0.1)
                            
                            # Update window title with current layer and point count
                            self.root.title(f"3D Scanner Control - Layer {layer_number}/{estimated_total_layers} at Z={current_z:.2f}mm - Points: {points_in_layer}")
                            
                            self.log_info(f"✓ Point {points_collected}/{points_per_rev} at {self.current_angle:.1f}° - Distance: {self.current_vl53_distance}mm (Layer {layer_number}: {points_in_layer} points)")
                        else:
                            self.log_info(f"⚠ Skipped point at {self.current_angle:.1f}° - No valid sensor data")
                    except Exception as e:
                        self.log_info(f"⚠ Skipped point at {self.current_angle:.1f}° - Error: {e}")
                
                self.log_info(f"Rotation complete. Collected {points_collected}/{points_per_rev} points")
                
                # Update title after rotation complete
                current_z = self.current_y_pos
                points_in_layer = sum(1 for p in self.scan_data 
                                    if len(p) >= 5 and abs(p[4] - current_z) < 0.1)
                self.root.title(f"3D Scanner Control - Layer {layer_number}/{estimated_total_layers} at Z={current_z:.2f}mm - Points: {points_in_layer}")
                
                if not self.is_scanning or self.scan_paused:
                    break
                
                # Check again if we've reached max height before moving Z
                current_z_height = self.current_y_pos - start_z_position
                if current_z_height >= max_height:
                    self.log_info(f"Đã đạt chiều cao tối đa: {current_z_height:.2f}mm >= {max_height:.2f}mm. Dừng scan.")
                    break
                
                # Move Z up by layer height (ONLY ONCE per rotation)
                # Note: y_move in format_gcode_command maps to GRBL Y (height axis)
                # format_gcode_command will convert: 2mm → G1 X0.2 (2mm ÷ 10 = 0.2 GRBL units)
                start_z_before = self.current_y_pos
                self.log_info(f"=== Moving Z up {layer_height_mm}mm (from {start_z_before:.1f}mm) - Layer {self.current_layer + 1} ===")
                
                # Create command to move Z (y_move will be converted to GRBL X units)
                # Example: y_move=2.0mm → G1 X0.2 F1 (auto-converted in format_gcode_command)
                commands = self.format_gcode_command(y_move=layer_height_mm, feed_rate=speed)
                
                # Log the command for debugging
                cmd_str = " ".join([c.strip() for c in commands])
                self.log_info(f"Z move command: {cmd_str}")
                print(f"[SCAN] Z move command (Layer {self.current_layer + 1}): {cmd_str}")
                
                # Send commands ONCE
                if self.serial_conn:
                    # Send commands with minimal delay
                    self.send_gcode_commands(commands, delay=0.05)
                    # Wait 0.5s for Z movement to complete (slow speed F1)
                    time.sleep(0.5)
                    
                    # Quick check if Z moved (optional verification)
                    current_z = self.current_y_pos
                    z_moved = abs(current_z - start_z_before)
                    if z_moved >= layer_height_mm * 0.5:  # At least 50% of target movement
                        self.log_info(f"✓ Z moved to {current_z:.2f}mm (moved {z_moved:.2f}mm)")
                    else:
                        self.log_info(f"Z position: {current_z:.2f}mm (expected move: {layer_height_mm}mm, actual: {z_moved:.2f}mm)")
                else:
                    self.log_info("ERROR: Serial connection lost!")
                    break
                
                # Update progress
                self.current_layer += 1
                total_points = len(self.scan_data)
                self.root.after(0, lambda: self.progress_var.set(f"Layer {self.current_layer}, Points: {total_points}"))
                
        except Exception as e:
            self.log_info(f"Scan error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.is_scanning = False
            # Update title when scan stops - show total points
            total_points = len(self.scan_data)
            self.root.after(0, lambda: self.root.title(f"3D Scanner Control - Total Points: {total_points}"))
            self.root.after(0, lambda: self.scan_up_btn.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.scan_down_btn.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.pause_btn.config(state=tk.DISABLED))
            self.root.after(0, lambda: self.resume_btn.config(state=tk.DISABLED))
            self.log_info("Scan stopped")

    def start_scan_up(self):
        """Start scanning from bottom to top"""
        if not self.is_connected:
            messagebox.showerror("Error", "Not connected to device")
            return
        
        if self.is_scanning:
            return
        
        self.is_scanning = True
        self.scan_paused = False
        self.scan_data = []
        self.current_layer = 0
        
        self.scan_up_btn.config(state=tk.DISABLED)
        self.scan_down_btn.config(state=tk.DISABLED)
        self.pause_btn.config(state=tk.NORMAL)
        self.resume_btn.config(state=tk.DISABLED)
        
        self.log_info("Starting scan...")
        
        # Start scan thread
        scan_thread = threading.Thread(target=self.scan_rotation_loop, daemon=True)
        scan_thread.start()

    def start_scan_down(self):
        """Start scanning from top to bottom (same as up for now)"""
        self.start_scan_up()

    def pause_scan(self):
        """Pause scanning"""
        if self.is_scanning:
            self.scan_paused = True
            self.pause_btn.config(state=tk.DISABLED)
            self.resume_btn.config(state=tk.NORMAL)
            self.log_info("Scan paused")

    def resume_scan(self):
        """Resume scanning"""
        if self.is_scanning and self.scan_paused:
            self.scan_paused = False
            self.pause_btn.config(state=tk.NORMAL)
            self.resume_btn.config(state=tk.DISABLED)
            self.log_info("Scan resumed")

    def clear_data(self):
        self.scan_data = []
        self.log_info("Data cleared")

    def move_to_top(self):
        self.log_info("Move to top not implemented")

    def go_home(self):
        self.log_info("Go home not implemented")

    def send_config(self):
        self.log_info("Send config not implemented")

    def update_visualization(self):
        """Update 3D visualization with scan data - improved surface mesh + point cloud"""
        try:
            self.ax.clear()

            if len(self.scan_data) > 0:
                # Extract x, y, z coordinates
                x_coords = np.array([p[0] for p in self.scan_data])
                y_coords = np.array([p[1] for p in self.scan_data])
                z_coords = np.array([p[2] for p in self.scan_data])

                # ============================================
                # IMPROVED MESH GENERATION
                # ============================================
                try:
                    if len(self.scan_data) > 0 and len(self.scan_data[0]) >= 5:
                        print(f"\n[MESH] Starting mesh generation with {len(self.scan_data)} points")

                        # Step 1: Group points by layer (height)
                        layer_groups = {}  # height -> list of (x, y, z, angle, index)

                        for idx, point_data in enumerate(self.scan_data):
                            if len(point_data) >= 5:
                                x, y, z, angle, height = point_data[:5]
                                # Round height to 0.1mm precision for grouping
                                height_key = round(height, 1)
                                if height_key not in layer_groups:
                                    layer_groups[height_key] = []
                                layer_groups[height_key].append((x, y, z, angle, idx))

                        print(f"[MESH] Found {len(layer_groups)} layers")

                        # Step 2: Sort layers by height
                        sorted_heights = sorted(layer_groups.keys())

                        # Step 3: Create mesh between adjacent layers
                        total_triangles = 0

                        for layer_idx in range(len(sorted_heights) - 1):
                            height1 = sorted_heights[layer_idx]
                            height2 = sorted_heights[layer_idx + 1]

                            # Get points from both layers, sorted by angle
                            layer1_points = sorted(layer_groups[height1], key=lambda p: p[3])  # Sort by angle
                            layer2_points = sorted(layer_groups[height2], key=lambda p: p[3])  # Sort by angle

                            print(
                                f"[MESH] Layer {height1:.1f}mm ({len(layer1_points)} pts) <-> {height2:.1f}mm ({len(layer2_points)} pts)")

                            # Step 4: Create triangulation between two layers
                            # Strategy: For each point in layer1, find closest point in layer2
                            # Then create triangles to connect them

                            triangles_in_layer = 0

                            # Method: Match points by angle proximity
                            for i in range(len(layer1_points)):
                                x1, y1, z1, angle1, idx1 = layer1_points[i]

                                # Find closest point in layer2 by angle
                                min_angle_diff = 360
                                closest_j = -1
                                for j in range(len(layer2_points)):
                                    x2, y2, z2, angle2, idx2 = layer2_points[j]
                                    angle_diff = abs(angle2 - angle1)
                                    if angle_diff > 180:
                                        angle_diff = 360 - angle_diff

                                    if angle_diff < min_angle_diff:
                                        min_angle_diff = angle_diff
                                        closest_j = j

                                # If found a close match (within 15 degrees)
                                if closest_j >= 0 and min_angle_diff < 15:
                                    x2, y2, z2, angle2, idx2 = layer2_points[closest_j]

                                    # Also get next point in layer1 (for quad)
                                    next_i = (i + 1) % len(layer1_points)
                                    x1_next, y1_next, z1_next, angle1_next, idx1_next = layer1_points[next_i]

                                    # Find matching point in layer2 for next point
                                    min_angle_diff_next = 360
                                    closest_j_next = -1
                                    for j in range(len(layer2_points)):
                                        x2_j, y2_j, z2_j, angle2_j, idx2_j = layer2_points[j]
                                        angle_diff = abs(angle2_j - angle1_next)
                                        if angle_diff > 180:
                                            angle_diff = 360 - angle_diff

                                        if angle_diff < min_angle_diff_next:
                                            min_angle_diff_next = angle_diff
                                            closest_j_next = j

                                    # If both matches found, create quad (2 triangles)
                                    if closest_j_next >= 0 and min_angle_diff_next < 15:
                                        x2_next, y2_next, z2_next, angle2_next, idx2_next = layer2_points[
                                            closest_j_next]

                                        # Check if points are not too far apart (prevent connecting distant points)
                                        # Maximum distance threshold: 50mm
                                        dist1 = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
                                        dist2 = np.sqrt((x1_next - x1) ** 2 + (y1_next - y1) ** 2 + (z1_next - z1) ** 2)
                                        dist3 = np.sqrt((x2_next - x2) ** 2 + (y2_next - y2) ** 2 + (z2_next - z2) ** 2)

                                        if dist1 < 50 and dist2 < 50 and dist3 < 50:
                                            # Create two triangles to form a quad
                                            # Triangle 1: (p1_L1, p1_L2, p2_L1)
                                            triangle1 = np.array([[x1, y1, z1],
                                                                  [x2, y2, z2],
                                                                  [x1_next, y1_next, z1_next]])

                                            # Triangle 2: (p1_L2, p2_L2, p2_L1)
                                            triangle2 = np.array([[x2, y2, z2],
                                                                  [x2_next, y2_next, z2_next],
                                                                  [x1_next, y1_next, z1_next]])

                                            # Draw triangles
                                            for triangle in [triangle1, triangle2]:
                                                # Calculate normal vector to check if triangle is valid
                                                v1 = triangle[1] - triangle[0]
                                                v2 = triangle[2] - triangle[0]
                                                normal = np.cross(v1, v2)

                                                # Only draw if triangle has non-zero area
                                                if np.linalg.norm(normal) > 0.01:
                                                    poly = Poly3DCollection([triangle],
                                                                            alpha=0.6,
                                                                            facecolor='cyan',
                                                                            edgecolor='blue',
                                                                            linewidths=0.5)
                                                    self.ax.add_collection3d(poly)
                                                    triangles_in_layer += 1

                            print(f"[MESH] Created {triangles_in_layer} triangles between layers")
                            total_triangles += triangles_in_layer

                        print(f"[MESH] Total triangles created: {total_triangles}")

                        # Step 5: Add top and bottom caps (optional)
                        # Top cap: connect all points in top layer
                        if len(sorted_heights) >= 1:
                            # Bottom cap
                            bottom_layer = sorted(layer_groups[sorted_heights[0]], key=lambda p: p[3])
                            if len(bottom_layer) >= 3:
                                # Create fan triangulation from center
                                center_x = np.mean([p[0] for p in bottom_layer])
                                center_y = np.mean([p[1] for p in bottom_layer])
                                center_z = sorted_heights[0]

                                for i in range(len(bottom_layer)):
                                    x1, y1, z1, _, _ = bottom_layer[i]
                                    next_i = (i + 1) % len(bottom_layer)
                                    x2, y2, z2, _, _ = bottom_layer[next_i]

                                    triangle = np.array([[center_x, center_y, center_z],
                                                         [x1, y1, z1],
                                                         [x2, y2, z2]])

                                    poly = Poly3DCollection([triangle],
                                                            alpha=0.7,
                                                            facecolor='lightgreen',
                                                            edgecolor='green',
                                                            linewidths=0.5)
                                    self.ax.add_collection3d(poly)

                            # Top cap
                            top_layer = sorted(layer_groups[sorted_heights[-1]], key=lambda p: p[3])
                            if len(top_layer) >= 3:
                                center_x = np.mean([p[0] for p in top_layer])
                                center_y = np.mean([p[1] for p in top_layer])
                                center_z = sorted_heights[-1]

                                for i in range(len(top_layer)):
                                    x1, y1, z1, _, _ = top_layer[i]
                                    next_i = (i + 1) % len(top_layer)
                                    x2, y2, z2, _, _ = top_layer[next_i]

                                    triangle = np.array([[center_x, center_y, center_z],
                                                         [x1, y1, z1],
                                                         [x2, y2, z2]])

                                    poly = Poly3DCollection([triangle],
                                                            alpha=0.7,
                                                            facecolor='lightcoral',
                                                            edgecolor='red',
                                                            linewidths=0.5)
                                    self.ax.add_collection3d(poly)

                except Exception as e:
                    print(f"[MESH] Error creating mesh: {e}")
                    import traceback
                    traceback.print_exc()

                # Always draw point cloud on top for reference
                if len(z_coords) > 0:
                    min_z = min(z_coords)
                    max_z = max(z_coords)
                    if max_z > min_z:
                        z_normalized = [(z - min_z) / (max_z - min_z) for z in z_coords]
                        self.ax.scatter(x_coords, y_coords, z_coords,
                                        c=z_normalized, cmap='viridis',
                                        s=8, alpha=0.8, edgecolors='black', linewidths=0.3)
                    else:
                        self.ax.scatter(x_coords, y_coords, z_coords,
                                        c='blue', s=8, alpha=0.8, edgecolors='black', linewidths=0.3)

                # Set labels
                self.ax.set_xlabel('X (mm)', fontsize=10)
                self.ax.set_ylabel('Y (mm)', fontsize=10)
                self.ax.set_zlabel('Z (mm)', fontsize=10)
                self.ax.set_title(f'3D Scan Mesh - {len(self.scan_data)} points', fontsize=11, fontweight='bold')

                # Calculate bounds with padding
                if len(x_coords) > 0 and len(y_coords) > 0 and len(z_coords) > 0:
                    x_range = max(x_coords) - min(x_coords)
                    y_range = max(y_coords) - min(y_coords)
                    z_range = max(z_coords) - min(z_coords)

                    max_range = max(x_range, y_range, z_range) or 1
                    padding = max_range * 0.1

                    mid_x = (max(x_coords) + min(x_coords)) / 2
                    mid_y = (max(y_coords) + min(y_coords)) / 2
                    mid_z = (max(z_coords) + min(z_coords)) / 2

                    self.ax.set_xlim(mid_x - max_range / 2 - padding, mid_x + max_range / 2 + padding)
                    self.ax.set_ylim(mid_y - max_range / 2 - padding, mid_y + max_range / 2 + padding)
                    self.ax.set_zlim(mid_z - max_range / 2 - padding, mid_z + max_range / 2 + padding)

                    try:
                        self.ax.set_box_aspect([1, 1, 1])
                    except:
                        pass
                else:
                    self.ax.set_xlim(-10, 10)
                    self.ax.set_ylim(-10, 10)
                    self.ax.set_zlim(0, 20)
            else:
                self.ax.set_xlabel('X (mm)', fontsize=10)
                self.ax.set_ylabel('Y (mm)', fontsize=10)
                self.ax.set_zlabel('Z (mm)', fontsize=10)
                self.ax.set_title('3D Scan Mesh - No data', fontsize=11)
                self.ax.set_xlim(-10, 10)
                self.ax.set_ylim(-10, 10)
                self.ax.set_zlim(0, 20)

            self.canvas.draw()
            self.canvas.flush_events()

        except Exception as e:
            self.log_info(f"Visualization error: {e}")
            import traceback
            traceback.print_exc()

    def export_stl(self):
        """Export scan data to STL file"""
        if len(self.scan_data) < 3:
            messagebox.showerror("Error", "Not enough scan data to export. Please scan first.")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".stl",
            filetypes=[("STL files", "*.stl"), ("All files", "*.*")]
        )
        
        if not filename:
            return
        
        try:
            self.log_info(f"Exporting STL to {filename}...")
            vertices, faces = self.generate_mesh_from_scan_data()
            self.write_stl_file(filename, vertices, faces)
            self.log_info(f"STL export completed: {len(faces)} triangles")
            messagebox.showinfo("Success", f"STL file exported successfully!\n{len(faces)} triangles")
        except Exception as e:
            self.log_info(f"STL export error: {e}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Error", f"Failed to export STL: {str(e)}")

    def export_k(self):
        """Export scan data to LS-DYNA Keyword file (.k) - HyperMesh compatible format"""
        if len(self.scan_data) < 3:
            messagebox.showerror("Error", "Not enough scan data to export. Please scan first.")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".k",
            filetypes=[("LS-DYNA Keyword files", "*.k"), ("All files", "*.*")]
        )
        
        if not filename:
            return
        
        try:
            self.log_info(f"Exporting LS-DYNA .k to {filename}...")
            vertices, faces = self.generate_mesh_from_scan_data()
            self.write_k_file(filename, vertices, faces)
            self.log_info(f"LS-DYNA .k export completed: {len(faces)} triangles")
            messagebox.showinfo("Success", f"LS-DYNA .k file exported successfully!\n{len(faces)} triangles")
        except Exception as e:
            self.log_info(f"LS-DYNA .k export error: {e}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Error", f"Failed to export .k: {str(e)}")
    
    def export_step(self):
        """Export scan data to STEP file"""
        if len(self.scan_data) < 3:
            messagebox.showerror("Error", "Not enough scan data to export. Please scan first.")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".step",
            filetypes=[("STEP files", "*.step"), ("STP files", "*.stp"), ("All files", "*.*")]
        )
        
        if not filename:
            return
        
        try:
            self.log_info(f"Exporting STEP to {filename}...")
            vertices, faces = self.generate_mesh_from_scan_data()
            self.write_step_file(filename, vertices, faces)
            self.log_info(f"STEP export completed: {len(faces)} triangles")
            messagebox.showinfo("Success", f"STEP file exported successfully!\n{len(faces)} triangles")
        except Exception as e:
            self.log_info(f"STEP export error: {e}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Error", f"Failed to export STEP: {str(e)}")
    
    def generate_mesh_from_scan_data(self):
        """Generate mesh (vertices and faces) from scan data"""
        if len(self.scan_data) < 3:
            raise ValueError("Not enough points to generate mesh")
        
        # Group points by layer (height)
        layer_groups = {}
        for idx, point_data in enumerate(self.scan_data):
            if len(point_data) >= 5:
                x, y, z, angle, height = point_data[:5]
                height_key = round(height, 1)
                if height_key not in layer_groups:
                    layer_groups[height_key] = []
                layer_groups[height_key].append((x, y, z, angle, idx))
        
        sorted_heights = sorted(layer_groups.keys())
        vertices = []
        faces = []
        vertex_index_map = {}  # Map (x, y, z) -> vertex index
        
        def get_vertex_index(x, y, z):
            """Get or create vertex index"""
            key = (round(x, 3), round(y, 3), round(z, 3))
            if key not in vertex_index_map:
                vertex_index_map[key] = len(vertices)
                vertices.append([x, y, z])
            return vertex_index_map[key]
        
        # Create mesh between adjacent layers
        for layer_idx in range(len(sorted_heights) - 1):
            height1 = sorted_heights[layer_idx]
            height2 = sorted_heights[layer_idx + 1]
            
            layer1_points = sorted(layer_groups[height1], key=lambda p: p[3])
            layer2_points = sorted(layer_groups[height2], key=lambda p: p[3])
            
            # Connect points between layers
            for i in range(len(layer1_points)):
                x1, y1, z1, angle1, idx1 = layer1_points[i]
                next_i = (i + 1) % len(layer1_points)
                x1_next, y1_next, z1_next, angle1_next, idx1_next = layer1_points[next_i]
                
                # Find closest points in layer2
                min_angle_diff = 360
                closest_j = -1
                for j in range(len(layer2_points)):
                    x2, y2, z2, angle2, idx2 = layer2_points[j]
                    angle_diff = abs(angle2 - angle1)
                    if angle_diff > 180:
                        angle_diff = 360 - angle_diff
                    if angle_diff < min_angle_diff:
                        min_angle_diff = angle_diff
                        closest_j = j
                
                if closest_j >= 0 and min_angle_diff < 15:
                    x2, y2, z2, angle2, idx2 = layer2_points[closest_j]
                    next_j = (closest_j + 1) % len(layer2_points)
                    x2_next, y2_next, z2_next, angle2_next, idx2_next = layer2_points[next_j]
                    
                    # Check distances to prevent connecting distant points
                    dist1 = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
                    dist2 = np.sqrt((x1_next - x1)**2 + (y1_next - y1)**2 + (z1_next - z1)**2)
                    dist3 = np.sqrt((x2_next - x2)**2 + (y2_next - y2)**2 + (z2_next - z2)**2)
                    
                    if dist1 < 50 and dist2 < 50 and dist3 < 50:
                        # Create quad as two triangles
                        v1 = get_vertex_index(x1, y1, z1)
                        v2 = get_vertex_index(x2, y2, z2)
                        v3 = get_vertex_index(x1_next, y1_next, z1_next)
                        v4 = get_vertex_index(x2_next, y2_next, z2_next)
                        
                        # Triangle 1: v1, v2, v3
                        faces.append([v1, v2, v3])
                        # Triangle 2: v2, v4, v3
                        faces.append([v2, v4, v3])
        
        # Add top and bottom caps
        if len(sorted_heights) >= 1:
            # Bottom cap
            bottom_layer = sorted(layer_groups[sorted_heights[0]], key=lambda p: p[3])
            if len(bottom_layer) >= 3:
                center_x = np.mean([p[0] for p in bottom_layer])
                center_y = np.mean([p[1] for p in bottom_layer])
                center_z = sorted_heights[0]
                center_idx = get_vertex_index(center_x, center_y, center_z)
                
                for i in range(len(bottom_layer)):
                    x1, y1, z1, _, _ = bottom_layer[i]
                    next_i = (i + 1) % len(bottom_layer)
                    x2, y2, z2, _, _ = bottom_layer[next_i]
                    v1 = get_vertex_index(x1, y1, z1)
                    v2 = get_vertex_index(x2, y2, z2)
                    faces.append([center_idx, v1, v2])
            
            # Top cap
            top_layer = sorted(layer_groups[sorted_heights[-1]], key=lambda p: p[3])
            if len(top_layer) >= 3:
                center_x = np.mean([p[0] for p in top_layer])
                center_y = np.mean([p[1] for p in top_layer])
                center_z = sorted_heights[-1]
                center_idx = get_vertex_index(center_x, center_y, center_z)
                
                for i in range(len(top_layer)):
                    x1, y1, z1, _, _ = top_layer[i]
                    next_i = (i + 1) % len(top_layer)
                    x2, y2, z2, _, _ = top_layer[next_i]
                    v1 = get_vertex_index(x1, y1, z1)
                    v2 = get_vertex_index(x2, y2, z2)
                    faces.append([center_idx, v2, v1])  # Reverse order for top
        
        return np.array(vertices), np.array(faces)
    
    def write_stl_file(self, filename, vertices, faces):
        """Write STL file in binary format"""
        with open(filename, 'wb') as f:
            # Write header (80 bytes)
            header = b'3D Scanner STL File - Exported from Scanner GUI' + b'\x00' * 35
            f.write(header[:80])
            
            # Write number of facets
            num_facets = len(faces)
            f.write(np.uint32(num_facets).tobytes())
            
            # Write facets
            for face in faces:
                v1 = vertices[face[0]]
                v2 = vertices[face[1]]
                v3 = vertices[face[2]]
                
                # Calculate normal
                edge1 = np.array(v2) - np.array(v1)
                edge2 = np.array(v3) - np.array(v1)
                normal = np.cross(edge1, edge2)
                norm = np.linalg.norm(normal)
                if norm > 0:
                    normal = normal / norm
                else:
                    normal = np.array([0.0, 0.0, 1.0])
                
                # Write normal (3 floats)
                f.write(np.float32(normal).tobytes())
                
                # Write vertices (3 floats each)
                f.write(np.float32(v1).tobytes())
                f.write(np.float32(v2).tobytes())
                f.write(np.float32(v3).tobytes())
                
                # Write attribute byte count (usually 0)
                f.write(np.uint16(0).tobytes())
    
    def write_obj_file(self, filename, vertices, faces):
        """Write OBJ file in text format"""
        with open(filename, 'w') as f:
            f.write("# OBJ file exported from 3D Scanner\n")
            f.write(f"# {len(vertices)} vertices, {len(faces)} faces\n\n")
            
            # Write vertices
            for v in vertices:
                f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
            
            f.write("\n")
            
            # Write faces (OBJ uses 1-based indexing)
            for face in faces:
                f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")
    
    def write_step_file(self, filename, vertices, faces):
        """Write STEP file (ISO 10303-21 format - simplified mesh representation)"""
        from datetime import datetime
        
        # STEP format is complex, so we'll create a simplified version
        # that represents the mesh as a faceted B-rep
        
        with open(filename, 'w', encoding='utf-8') as f:
            f.write("ISO-10303-21;\n")
            f.write("HEADER;\n")
            f.write("FILE_DESCRIPTION(('3D Scanner Export'),'2;1');\n")
            timestamp = datetime.now().strftime('%Y-%m-%dT%H:%M:%S')
            f.write(f"FILE_NAME('scan_export','{timestamp}',('Scanner GUI'),(''),'STP','','');\n")
            f.write("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n")
            f.write("ENDSEC;\n")
            f.write("DATA;\n\n")
            
            ref_idx = 1
            
            # Write direction for coordinate system
            dir_x_ref = f"#{ref_idx}"
            ref_idx += 1
            f.write(f"{dir_x_ref} = DIRECTION('', (1.0, 0.0, 0.0));\n")
            
            dir_z_ref = f"#{ref_idx}"
            ref_idx += 1
            f.write(f"{dir_z_ref} = DIRECTION('', (0.0, 0.0, 1.0));\n")
            
            # Write origin point
            origin_ref = f"#{ref_idx}"
            ref_idx += 1
            f.write(f"{origin_ref} = CARTESIAN_POINT('', (0.0, 0.0, 0.0));\n")
            
            # Write axis placement
            axis_ref = f"#{ref_idx}"
            ref_idx += 1
            f.write(f"{axis_ref} = AXIS2_PLACEMENT_3D('', {origin_ref}, {dir_z_ref}, {dir_x_ref});\n")
            
            # Write vertices as Cartesian points
            vertex_refs = []
            for v in vertices:
                ref = f"#{ref_idx}"
                vertex_refs.append(ref)
                f.write(f"{ref} = CARTESIAN_POINT('', ({v[0]:.6f}, {v[1]:.6f}, {v[2]:.6f}));\n")
                ref_idx += 1
            
            # Write vertex points
            vertex_point_refs = []
            for i, v_ref in enumerate(vertex_refs):
                ref = f"#{ref_idx}"
                vertex_point_refs.append(ref)
                f.write(f"{ref} = VERTEX_POINT('', {vertex_refs[i]});\n")
                ref_idx += 1
            
            # For STEP, we'll create a simplified representation
            # Create a faceted B-rep using triangles
            # Note: Full STEP implementation would require many more entities
            # This is a simplified version that may work with some CAD software
            
            # Write faces (simplified - just reference the triangles)
            face_refs = []
            for i, face in enumerate(faces[:100]):  # Limit to first 100 faces for simplicity
                v1 = vertices[face[0]]
                v2 = vertices[face[1]]
                v3 = vertices[face[2]]
                
                # Calculate normal
                edge1 = np.array(v2) - np.array(v1)
                edge2 = np.array(v3) - np.array(v1)
                normal = np.cross(edge1, edge2)
                norm = np.linalg.norm(normal)
                if norm > 0:
                    normal = normal / norm
                else:
                    normal = np.array([0.0, 0.0, 1.0])
                
                # Create plane for this triangle
                plane_ref = f"#{ref_idx}"
                ref_idx += 1
                f.write(f"{plane_ref} = PLANE('', {axis_ref});\n")
                
                # Create face (simplified)
                face_ref = f"#{ref_idx}"
                ref_idx += 1
                f.write(f"{face_ref} = FACE('', (FACE_BOUND('', EDGE_LOOP('', ()), .T.),), {plane_ref}, .T.);\n")
                face_refs.append(face_ref)
            
            # Create closed shell
            if face_refs:
                shell_ref = f"#{ref_idx}"
                ref_idx += 1
                face_list = ', '.join(face_refs)
                f.write(f"{shell_ref} = CLOSED_SHELL('', ({face_list}));\n")
                
                # Create solid
                solid_ref = f"#{ref_idx}"
                ref_idx += 1
                f.write(f"{solid_ref} = MANIFOLD_SOLID_BREP('', {shell_ref});\n")
            
            f.write("\nENDSEC;\n")
            f.write("END-ISO-10303-21;\n")


    def write_k_file(self, filename, vertices, faces):
        """Write LS-DYNA keyword file with HyperMesh format
        
        Args:
            filename: Output filename
            vertices: Numpy array of vertices (N, 3)
            faces: Numpy array of face indices (M, 3)
        """
        # Get current time for header
        now = datetime.now()
        timestamp = now.strftime("%H:%M:%S %m-%d-%Y")
        
        with open(filename, 'w') as f:
            # Write HyperMesh header
            f.write(f"$$ HM_OUTPUT_DECK created {timestamp} by 3D Scanner GUI\n")
            f.write("$$ Ls-dyna Input Deck Generated by Scanner Export\n")
            f.write("$$ Generated using HyperMesh-Ls-dyna 971 Template Compatible Format\n")
            
            # Write KEYWORD
            f.write("*KEYWORD\n")
            
            # Control cards
            f.write("*CONTROL_TERMINATION\n")
            f.write("$$  ENDTIM    ENDCYC     DTMIN    ENDENG    ENDMAS\n")
            f.write("      10.0                                        \n")
            
            f.write("*CONTROL_TIMESTEP\n")
            f.write("$$  DTINIT    TSSFAC      ISDO    TSLIMT     DT2MS      LCTM     ERODE     MSIST\n")
            f.write("                 0.9                 0.0       0.0                              \n")
            
            f.write("*CONTROL_HOURGLASS\n")
            f.write("$$     IHQ        QH\n")
            f.write("         1      0.05\n")
            
            f.write("*CONTROL_ENERGY\n")
            f.write("$$    HGEN      RWEN    SLNTEN     RYLEN\n")
            f.write("         2         2         2         2\n")
            
            # Database output
            f.write("$$DATABASE_OPTION -- Control Cards for ASCII output\n")
            f.write("*DATABASE_ELOUT\n")
            f.write("       1.0         1\n")
            
            f.write("*DATABASE_GLSTAT\n")
            f.write("       1.0         1\n")
            
            f.write("*DATABASE_MATSUM\n")
            f.write("       1.0         1\n")
            
            f.write("*DATABASE_NODOUT\n")
            f.write("       1.0         1\n")
            
            f.write("*DATABASE_RCFORC\n")
            f.write("       1.0         1\n")
            
            f.write("*DATABASE_BINARY_D3PLOT\n")
            f.write("$$ DT/CYCL      LCDT      BEAM     NPLTC\n")
            f.write("       1.0                                        \n")
            f.write("         0\n")
            
            # Write nodes
            f.write("*NODE\n")
            for node_id, coords in enumerate(vertices, start=1):
                # Format: ID (8 chars), X (16 chars), Y (16 chars), Z (16 chars)
                f.write(f"{node_id:>8}{coords[0]:>16.6f}{coords[1]:>16.6f}{coords[2]:>16.6f}\n")
            
            # Write material with HyperMesh comments
            f.write("*MAT_ELASTIC\n")
            f.write(f"$HMNAME MATS       1{'Scanned_Material':<30}\n")
            f.write("$HWCOLOR MATS       1       2\n")
            f.write("         1      7.85E-09    210000      0.30                              \n")
            f.write("                                                  \n")
            
            # Write section with HyperMesh comments
            f.write("*SECTION_SHELL\n")
            f.write(f"$HMNAME PROPS       1{'Shell_1mm':<30}\n")
            f.write("         1         2                                                  \n")
            f.write("       1.0       1.0       1.0       1.0                    \n")
            
            # Write part with HyperMesh comments
            f.write("*PART\n")
            f.write(f"$HMNAME COMPS       1{'3D_Scanned_Object':<30}\n")
            f.write("$HWCOLOR COMPS       1       3\n")
            f.write("                                                                                \n")
            f.write("         1         1         1                                                  \n")
            
            # Write elements
            # Using ELEMENT_SHELL for triangular elements (4 nodes, with 3rd and 4th node same for triangle)
            f.write("*ELEMENT_SHELL\n")
            for elem_id, face in enumerate(faces, start=1):
                # For triangular shell: use 4-node format with last node = 3rd node
                # Node IDs are 1-based (face indices + 1)
                n1 = face[0] + 1
                n2 = face[1] + 1
                n3 = face[2] + 1
                n4 = n3  # Duplicate last node for triangular element
                f.write(f"{elem_id:>8}       1{n1:>8}{n2:>8}{n3:>8}{n4:>8}\n")
            
            # End file
            f.write("*END\n")
        
        self.log_info(f"LS-DYNA .k file written: {filename}")
        self.log_info(f"  Total nodes: {len(vertices)}")
        self.log_info(f"  Total elements: {len(faces)}")

    def get_cartesian_points(self):
        return None

    def log_info(self, message):
        """Log to info text widget (thread-safe)"""
        if hasattr(self, 'root') and hasattr(self, 'info_text'):
            self.root.after(0, lambda msg=message: self._log_info_safe(msg))
        else:
            print(f"[LOG] {message}")

    def _log_info_safe(self, message):
        """Thread-safe helper"""
        try:
            if hasattr(self, 'info_text'):
                self.info_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
                self.info_text.see(tk.END)
        except Exception as e:
            print(f"Error logging: {e}")

    def log_serial_send(self, command):
        """Log command sent"""
        if hasattr(self, 'serial_log_text'):
            timestamp = time.strftime('%H:%M:%S')
            self.serial_log_text.insert(tk.END, f"[{timestamp}] → SEND: {command.strip()}\n")
            self.serial_log_text.see(tk.END)
            lines = int(self.serial_log_text.index('end-1c').split('.')[0])
            if lines > 1000:
                self.serial_log_text.delete('1.0', f'{lines-1000}.0')

    def log_serial_receive(self, response):
        """Log response received"""
        if hasattr(self, 'serial_log_text'):
            timestamp = time.strftime('%H:%M:%S')
            self.serial_log_text.insert(tk.END, f"[{timestamp}] ← RECV: {response.strip()}\n")
            self.serial_log_text.see(tk.END)
            lines = int(self.serial_log_text.index('end-1c').split('.')[0])
            if lines > 1000:
                self.serial_log_text.delete('1.0', f'{lines-1000}.0')

    def clear_serial_log(self):
        """Clear serial log"""
        if hasattr(self, 'serial_log_text'):
            self.serial_log_text.delete('1.0', tk.END)

def main():
    root = tk.Tk()
    app = ScannerGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()