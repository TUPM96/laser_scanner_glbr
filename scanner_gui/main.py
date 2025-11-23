"""
3D Scanner GUI Application
Python GUI for controlling 3D scanner and visualizing scan data

CORRECTED VERSION - HARDWARE SPECIFICATIONS & CALIBRATION:

ROTATION AXIS (GUI X = GRBL Y):
- Calibration: 0.1mm GRBL = 45° motor rotation
- Formula: angle = GRBL_Y_position × 450
- Full rotation: 360° = 0.8mm GRBL units
- Conversion: Direct value (no scaling needed)

HEIGHT AXIS (GUI Z = GRBL X):
- Full step mode: G1 X0.1 = 45° motor = 1mm actual movement
- M8 lead screw: 1 motor revolution (360°) = 8mm
- Formula: actual_mm = GRBL_X_value × 10
- Conversion when sending: GRBL_X = mm / 10
- Conversion when parsing: mm = GRBL_X × 10

COORDINATE MAPPING:
- format_gcode_command(): GUI X → GRBL Y (direct), GUI Z → GRBL X (÷10)
- parse_grbl_status(): GRBL Y → GUI X (direct), GRBL X → GUI Z (×10)
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
import time
import os

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
        self.current_vl53_distance = None  # Current VL53L0X distance reading in mm
        self.scan_paused = False
        self.scan_start_angle = 0.0  # Starting angle for current rotation

        # Position tracking - CORRECTED CALIBRATION
        # Rotation axis (GRBL Y → GUI X): 0.1mm GRBL = 45°, direct value
        # Height axis (GRBL X → GUI Z): Full step mode, GRBL value × 10 = actual mm
        # NOTE: format_gcode_command swaps X/Y and scales Z when sending
        #       parse_grbl_status swaps back and scales Z × 10 when parsing
        self.current_x_pos = 0.0  # Rotation position in mm (from GRBL Y, direct value)
        self.current_y_pos = 0.0  # Height position in mm (from GRBL X × 10, actual measurement)
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

        # Geometry parameters
        geometry_frame = ttk.LabelFrame(control_frame, text="Thông số hình học", padding="5")
        geometry_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=3)

        ttk.Label(geometry_frame, text="Khoảng cách tâm (cm):").grid(row=0, column=0, sticky=tk.W, pady=1)
        self.center_distance_var = tk.StringVar(value="15.0")
        ttk.Entry(geometry_frame, textvariable=self.center_distance_var, width=8).grid(row=0, column=1, sticky=tk.W, padx=2)

        ttk.Label(geometry_frame, text="Bán kính đĩa (cm):").grid(row=0, column=2, sticky=tk.W, padx=(10,0), pady=1)
        self.disk_radius_var = tk.StringVar(value="5.0")
        ttk.Entry(geometry_frame, textvariable=self.disk_radius_var, width=8).grid(row=0, column=3, sticky=tk.W, padx=2)

        ttk.Label(geometry_frame, text="Số điểm scan/vòng:").grid(row=1, column=0, sticky=tk.W, pady=1)
        self.points_per_revolution_var = tk.StringVar(value="8")
        ttk.Entry(geometry_frame, textvariable=self.points_per_revolution_var, width=8).grid(row=1, column=1, sticky=tk.W, padx=2)

        ttk.Label(geometry_frame, text="Chiều cao tối đa (mm):").grid(row=1, column=2, sticky=tk.W, padx=(10,0), pady=1)
        self.z_travel_var = tk.StringVar(value="100")
        ttk.Entry(geometry_frame, textvariable=self.z_travel_var, width=8).grid(row=1, column=3, sticky=tk.W, padx=2)

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

        # Right panel: VL53L0X display
        right_panel = ttk.LabelFrame(main_test_frame, text="VL53L0X Distance Sensor", padding="10")
        right_panel.grid(row=0, column=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5)

        # Large distance display
        self.vl53_distance_var = tk.StringVar(value="--")
        distance_display = ttk.Label(right_panel, textvariable=self.vl53_distance_var,
                                    font=("Arial", 24, "bold"), foreground="blue")
        distance_display.grid(row=0, column=0, pady=10)

        ttk.Label(right_panel, text="mm", font=("Arial", 12)).grid(row=0, column=1, sticky=tk.W, padx=5)

        # Start/Stop reading button
        self.vl53_read_btn = ttk.Button(right_panel, text="Start Reading",
                                        command=self.toggle_vl53_reading, state=tk.DISABLED)
        self.vl53_read_btn.grid(row=1, column=0, columnspan=2, pady=10, sticky=(tk.W, tk.E))

        # Status
        self.vl53_status_var = tk.StringVar(value="Stopped")
        ttk.Label(right_panel, textvariable=self.vl53_status_var, foreground="gray").grid(
            row=2, column=0, columnspan=2, pady=5)

        # Visual distance bar
        self.distance_canvas = tk.Canvas(right_panel, width=200, height=20, bg="lightgray")
        self.distance_canvas.grid(row=3, column=0, columnspan=2, pady=10)
        self.distance_bar = self.distance_canvas.create_rectangle(0, 0, 0, 20, fill="green")

        # Bottom panel: Position display and controls
        bottom_panel = ttk.Frame(test_tab)
        bottom_panel.grid(row=2, column=0, sticky=(tk.W, tk.E), padx=5, pady=5)

        # Current position display - CORRECTED LABELS
        pos_frame = ttk.LabelFrame(bottom_panel, text="Current Position", padding="5")
        pos_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=5)

        # X position displays angle (from GRBL Y axis after swap)
        ttk.Label(pos_frame, text="X (Góc quay, 0.1mm=45°):").grid(row=0, column=0, padx=5)
        self.test_x_pos_var = tk.StringVar(value="0.0°")
        ttk.Label(pos_frame, textvariable=self.test_x_pos_var, font=("Arial", 10, "bold")).grid(row=0, column=1, padx=5)

        # Z position displays height (from GRBL X axis after swap, M8 lead screw)
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
                self.serial_conn = serial.Serial(port, 115200, timeout=1)
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
                self.log_info("ROTATION (GUI X = GRBL Y):")
                self.log_info("  • 0.1mm GRBL = 45° motor")
                self.log_info("  • angle = GRBL_Y × 450")
                self.log_info("  • 1 full rotation = 0.8mm GRBL")
                self.log_info("")
                self.log_info("HEIGHT (GUI Z = GRBL X):")
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
            self.log_info("Disconnected")

    def send_serial_command(self, command, log=True):
        """Send command to serial and log it"""
        if self.serial_conn:
            try:
                if isinstance(command, str):
                    command_bytes = command.encode()
                else:
                    command_bytes = command

                # Send to serial
                self.serial_conn.write(command_bytes)

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

        # Handle VL53L0X distance reading
        if "VL53L0X" in line.upper() or line.startswith("DISTANCE:"):
            try:
                import re
                if ":" in line:
                    parts = line.split(":")
                    if len(parts) > 1:
                        dist_str = parts[1].strip()
                        dist_str = re.sub(r'[^0-9.]', '', dist_str)
                        if dist_str:
                            distance_mm = float(dist_str)
                            self.update_vl53_display(distance_mm)
                            return

                match = re.search(r'(\d+\.?\d*)\s*(mm|cm)', line, re.IGNORECASE)
                if match:
                    distance = float(match.group(1))
                    unit = match.group(2).lower()
                    if unit == "cm":
                        distance = distance * 10
                    self.update_vl53_display(distance)
                    return
            except Exception as e:
                pass
            return

        # Legacy protocol handlers (for backward compatibility with custom firmware)
        # ... (rest of the protocol handling code remains the same as original)

    def parse_grbl_status(self, status_line):
        """Parse GRBL status report and update position display
        CORRECTED CALIBRATION: 0.1mm = 45 degrees, so angle = position_mm × 450
        Mapping: GRBL X -> GUI Z (height), GRBL Y -> GUI X (angle)
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
            #   GUI X (rotation) → sent as GRBL Y (direct value)
            #   GUI Y (height) → sent as GRBL X (divided by 10)
            # So when parsing, we need to swap back:
            #   positions[0] (GRBL X) = height units → multiply by 10 to get mm
            #   positions[1] (GRBL Y) = rotation (direct mm value)
            print(f"[PARSE] Step 6: Parsing position values...")
            try:
                # GRBL Y (positions[1]) is rotation (GUI X) - direct value
                x_mm = float(positions[1])
                print(f"[PARSE] Step 6: X (rotation from GRBL Y) = {x_mm} mm")
            except ValueError as e:
                print(f"[PARSE] ✗ ERROR parsing X from '{positions[1]}': {e}")
                print(f"{'='*70}\n")
                return

            try:
                # GRBL X (positions[0]) is height (GUI Z) - need to multiply by 10
                # Because: G1 X0.1 = 45° motor = 1mm actual movement
                # Formula: mm_actual = GRBL_X_value × 10
                y_grbl_units = float(positions[0])
                y_mm = y_grbl_units * 10.0  # Convert GRBL units to actual mm
                print(f"[PARSE] Step 6: Y (height from GRBL X) = {y_grbl_units} units = {y_mm:.1f} mm")
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
            self.current_x_pos = x_mm  # Rotation from GRBL Y (direct mm)
            self.current_y_pos = y_mm  # Height from GRBL X (converted to actual mm)
            print(f"[PARSE] Step 7: ✓ Internal state updated")
            print(f"[PARSE] Step 7: X = {x_mm:.3f}mm (rotation from GRBL Y), Y = {y_mm:.1f}mm (height from GRBL X)")

            # Step 8: Calculate angle from X position
            print(f"[PARSE] Step 8: Calculating angle from X position...")
            print(f"[PARSE] Step 8: Formula: {x_mm:.3f} mm × 450 = ? degrees")
            angle = x_mm * 450.0
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
        - X shows rotation angle (from GRBL Y): 0.1mm GRBL = 45°, angle = value × 450
        - Z shows height (from GRBL X): converted to actual mm (GRBL × 10)
        """
        try:
            if hasattr(self, 'test_x_pos_var'):
                # X shows angle (from GRBL Y, calculated as GRBL_Y × 450)
                self.test_x_pos_var.set(f"{self.current_angle:.1f}°")

            if hasattr(self, 'test_z_pos_var'):
                # Z shows height (from GRBL X, already converted to actual mm)
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
        """Update VL53L0X distance display"""
        # Store current distance reading for scan processing
        if distance_mm > 0 and distance_mm < 8190:
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
        - x_move (rotation, GUI X → GRBL Y): Direct value, 0.1mm = 45°
        - y_move (height, GUI Z → GRBL X): Convert mm to GRBL units (÷ 10)
          Example: y_move=2.0mm → G1 X0.2 → motor 90° → 2mm actual movement
        """
        feed_rate_float = max(1.0, float(feed_rate))
        feed_rate = int(feed_rate_float) if feed_rate_float.is_integer() else feed_rate_float

        commands = []
        commands.append("G91\n")

        move_parts = ["G1"]

        # Rotation axis: GUI X → GRBL Y (direct value)
        if abs(x_move) >= 0.1:
            x_str = f"{x_move:.1f}".rstrip('0').rstrip('.')
            move_parts.append(f"Y{x_str}")

        # Height axis: GUI Z → GRBL X (convert mm to GRBL units)
        # Formula: GRBL_X_value = mm / 10
        # Example: 2mm → 0.2, 5mm → 0.5, 10mm → 1.0
        if abs(y_move) >= 0.01:
            y_grbl_units = y_move * 0.1  # Convert mm to GRBL units
            y_str = f"{y_grbl_units:.2f}".rstrip('0').rstrip('.')
            move_parts.append(f"X{y_str}")

        if abs(z_move) >= 0.1:
            z_str = f"{z_move:.1f}".rstrip('0').rstrip('.')
            move_parts.append(f"Z{z_str}")

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

        X axis (rotation): 360° = 360/450 mm = 0.8mm
        Because: 0.1mm = 45°, so 1° = 0.1/45 mm, 360° = 360 × (0.1/45) = 0.8mm

        Y axis (height, M8 lead screw): 1 revolution = 8mm
        """
        return 0.8  # Fixed value for X axis based on calibration

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

    def toggle_vl53_reading(self):
        """Toggle VL53L0X reading"""
        if not self.is_connected:
            return

        if not self.vl53_reading_active:
            self.vl53_reading_active = True
            self.vl53_read_btn.config(text="Stop Reading")
            self.vl53_status_var.set("Reading...")

            reading_thread = threading.Thread(target=self.continuous_vl53_read, daemon=True)
            reading_thread.start()
        else:
            self.vl53_reading_active = False
            self.vl53_read_btn.config(text="Start Reading")
            self.vl53_status_var.set("Stopped")

    def continuous_vl53_read(self):
        """Continuously read from VL53L0X"""
        while self.vl53_reading_active and self.is_connected:
            try:
                if self.serial_conn:
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
            self.log_info(f"X 360° CW (0.8mm): {cmd_str}")

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
            self.log_info(f"X 360° CCW (-0.8mm): {cmd_str}")

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
                return None  # Out of range, skip this point

            # Calculate radius from center
            # Logic from MATLAB: r = centerDistance - distance
            # This gives radius from turntable center to object surface
            radius_from_center = center_distance_cm - distance_cm

            # Filter 4: Remove values around 0 (midThresh in MATLAB)
            # midThreshUpper=0.5, midThreshLower=-0.5
            mid_thresh_upper = 0.5
            mid_thresh_lower = -0.5
            if mid_thresh_lower < radius_from_center < mid_thresh_upper:
                return None  # Too close to center, likely error

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
        z_height = self.current_y_pos  # Height from GRBL X (after swap)
        
        # Calculate 3D point
        point = self.calculate_point_from_scan(angle, self.current_vl53_distance, z_height)
        if point:
            self.scan_data.append(point)
            # Update visualization in main thread (thread-safe)
            self.root.after(0, self.update_visualization)
            self.log_info(f"Point added: angle={angle:.1f}°, dist={self.current_vl53_distance:.1f}mm, z={z_height:.1f}mm, point=({point[0]:.1f}, {point[1]:.1f}, {point[2]:.1f})mm")

    def scan_rotation_loop(self):
        """Main scan loop: rotate X continuously, read sensor, move Z up after each rotation"""
        if not self.is_connected or not self.serial_conn:
            return
        
        try:
            # Get layer height
            # With calibration: G1 X0.2 = motor 90° = 2mm actual movement
            # Layer height = 2mm per layer (can be adjusted)
            layer_height_mm = 2.0  # Fixed 2mm per layer
            
            # Get number of points per revolution
            try:
                points_per_rev = int(self.points_per_revolution_var.get())
                if points_per_rev < 1:
                    points_per_rev = 360
                    self.log_info(f"Invalid points per revolution, using default: {points_per_rev}")
            except:
                points_per_rev = 360
                self.log_info(f"Error reading points per revolution, using default: {points_per_rev}")
            
            # Calculate angle step per point
            angle_step = 360.0 / points_per_rev
            
            # Calculate one revolution distance (0.8mm for 360 degrees)
            one_rev_distance = self.calculate_one_revolution_distance()
            
            # Speed for scan is always 1 (slowest speed for step motor)
            speed = 1.0
            
            self.log_info(f"Scan settings: {points_per_rev} points/vòng, {angle_step:.2f}° per point, speed=F{int(speed)}")
            
            while self.is_scanning and not self.scan_paused:
                # Record starting angle and X position
                self.scan_start_angle = self.current_angle
                start_x_pos = self.current_x_pos  # X position in mm (rotation axis)
                start_z = self.current_y_pos
                
                self.log_info(f"Starting rotation at angle {self.scan_start_angle:.1f}°, X={start_x_pos:.3f}mm, Z={start_z:.2f}mm")
                
                # Start continuous rotation (1 full revolution = 0.8mm)
                commands = self.format_gcode_command(x_move=one_rev_distance, feed_rate=speed)
                self.send_gcode_commands(commands, delay=0.05)
                
                # Monitor rotation and collect data points
                last_angle = self.current_angle
                last_processed_angle = self.scan_start_angle
                angle_changed = False
                points_collected = 0
                next_target_angle = (self.scan_start_angle + angle_step) % 360.0
                
                # Wait for rotation to complete (monitor angle change)
                timeout = time.time() + 60  # 60 second timeout (slower speed needs more time)
                while self.is_scanning and not self.scan_paused:
                    current_angle = self.current_angle
                    
                    # Check if we've collected enough points
                    if points_collected >= points_per_rev:
                        self.log_info(f"Collected all {points_per_rev} points")
                        break
                    
                    # Check if we've completed a full rotation (360 degrees = 0.8mm)
                    # Method 1: Check X position change (more reliable)
                    current_x_pos = self.current_x_pos
                    x_moved = abs(current_x_pos - start_x_pos)
                    
                    # X position wraps around: if start was near 0 and current is near 0.8, or vice versa
                    # Or if we've moved approximately 0.8mm
                    x_completed = (x_moved >= 0.75) or (start_x_pos < 0.1 and current_x_pos >= 0.75) or (start_x_pos >= 0.75 and current_x_pos < 0.1)
                    
                    # Method 2: Check angle difference (backup)
                    angle_diff = current_angle - self.scan_start_angle
                    if angle_diff < 0:
                        angle_diff += 360.0
                    
                    # Also check if we've wrapped around (current angle near 0, start was near 360)
                    wrapped_around = (self.scan_start_angle > 350.0 and current_angle < 10.0 and points_collected > 0)
                    
                    # Stop when X position indicates 1 full rotation OR angle indicates completion
                    if x_completed or angle_diff >= 358.0 or (angle_diff >= 355.0 and points_collected >= points_per_rev * 0.9) or wrapped_around:
                        if x_completed:
                            self.log_info(f"Completed 1 full rotation (X moved {x_moved:.3f}mm: {start_x_pos:.3f} → {current_x_pos:.3f}mm)")
                        elif wrapped_around:
                            self.log_info(f"Completed 1 full rotation (wrapped around: {self.scan_start_angle:.1f}° → {current_angle:.1f}°)")
                        else:
                            self.log_info(f"Completed 1 full rotation ({angle_diff:.1f}°)")
                        break
                    
                    # Calculate angle change from last processed angle
                    angle_change = (current_angle - last_processed_angle) % 360.0
                    if angle_change > 180:
                        angle_change = 360 - angle_change
                    
                    # Check if we've reached the next target angle
                    target_reached = False
                    if angle_change >= angle_step * 0.8:  # 80% of step to account for timing
                        # Check if we're close to target angle
                        angle_to_target = (next_target_angle - current_angle) % 360.0
                        if angle_to_target > 180:
                            angle_to_target = 360 - angle_to_target
                        
                        if angle_to_target <= angle_step * 0.5 or angle_change >= angle_step:
                            target_reached = True
                    
                    if target_reached:
                        # Request sensor reading with timeout
                        sensor_data_received = False
                        try:
                            if self.serial_conn:
                                self.send_serial_command("READ_VL53L0X\n", log=False)

                            # Wait for sensor reading with timeout (max 0.5s)
                            wait_start = time.time()
                            while time.time() - wait_start < 0.5:
                                if self.current_vl53_distance is not None:
                                    sensor_data_received = True
                                    break
                                time.sleep(0.05)

                            # Process point if we have sensor data
                            if sensor_data_received:
                                self.process_scan_data_point()
                                points_collected += 1
                            else:
                                # Timeout - skip this point
                                pass  # Silent skip, no log spam
                        except Exception as e:
                            # Handle write timeout or other errors gracefully
                            if "timeout" in str(e).lower():
                                pass  # Silent skip on timeout
                            else:
                                self.log_info(f"Sensor error at {current_angle:.1f}°: {e}")

                        # Always move to next target angle (whether we got data or not)
                        last_processed_angle = current_angle
                        next_target_angle = (next_target_angle + angle_step) % 360.0
                        angle_changed = True
                    
                    if abs(current_angle - last_angle) > 0.1:  # Angle is changing
                        angle_changed = True
                    last_angle = current_angle
                    
                    if time.time() > timeout:
                        self.log_info(f"Rotation timeout. Collected {points_collected}/{points_per_rev} points")
                        break
                    
                    time.sleep(0.05)  # Check every 50ms
                
                self.log_info(f"Rotation complete. Collected {points_collected}/{points_per_rev} points")
                
                if not self.is_scanning or self.scan_paused:
                    break
                
                # Move Z up by layer height (ONLY ONCE per rotation)
                # Note: y_move in format_gcode_command maps to GRBL X (height axis)
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
        """Update 3D visualization with scan data - surface mesh + point cloud"""
        try:
            self.ax.clear()

            if len(self.scan_data) > 0:
                # Extract x, y, z coordinates
                x_coords = np.array([p[0] for p in self.scan_data])
                y_coords = np.array([p[1] for p in self.scan_data])
                z_coords = np.array([p[2] for p in self.scan_data])

                # Try to create surface mesh if we have enough structured data
                try:
                    # Get unique Z heights (layers)
                    unique_z = np.unique(z_coords)
                    if len(unique_z) >= 2:  # Need at least 2 layers for surface
                        # Group points by layer
                        layers = []
                        for z_val in unique_z:
                            layer_mask = np.abs(z_coords - z_val) < 0.01  # tolerance 0.01cm
                            layer_x = x_coords[layer_mask]
                            layer_y = y_coords[layer_mask]
                            if len(layer_x) > 0:
                                layers.append((layer_x, layer_y, z_val))

                        # Draw surface between consecutive layers
                        if len(layers) >= 2:
                            for i in range(len(layers) - 1):
                                x1, y1, z1 = layers[i]
                                x2, y2, z2 = layers[i + 1]

                                # Create mesh between two layers
                                n_points = min(len(x1), len(x2))
                                if n_points >= 3:
                                    # Draw surface strips
                                    for j in range(n_points - 1):
                                        # Create quad between points
                                        verts = [
                                            [x1[j], y1[j], z1],
                                            [x1[j+1], y1[j+1], z1],
                                            [x2[j+1], y2[j+1], z2],
                                            [x2[j], y2[j], z2]
                                        ]
                                        # Draw filled polygon
                                        self.ax.plot([v[0] for v in verts] + [verts[0][0]],
                                                    [v[1] for v in verts] + [verts[0][1]],
                                                    [v[2] for v in verts] + [verts[0][2]],
                                                    'b-', alpha=0.3, linewidth=0.5)
                except Exception as e:
                    # If surface creation fails, continue with point cloud
                    print(f"[VIZ] Surface mesh failed: {e}, using point cloud only")

                # Always draw point cloud on top
                if len(z_coords) > 0:
                    min_z = min(z_coords)
                    max_z = max(z_coords)
                    if max_z > min_z:
                        # Normalize Z to 0-1 for colormap
                        z_normalized = [(z - min_z) / (max_z - min_z) for z in z_coords]
                        # Use colormap: blue (low) to red (high)
                        self.ax.scatter(x_coords, y_coords, z_coords,
                                       c=z_normalized, cmap='viridis',
                                       s=3, alpha=0.9, edgecolors='none')
                    else:
                        # All points at same height
                        self.ax.scatter(x_coords, y_coords, z_coords,
                                       c='blue', s=3, alpha=0.9, edgecolors='none')
                
                # Set labels
                self.ax.set_xlabel('X (mm)', fontsize=10)
                self.ax.set_ylabel('Y (mm)', fontsize=10)
                self.ax.set_zlabel('Z (mm)', fontsize=10)
                self.ax.set_title(f'3D Scan Point Cloud - {len(self.scan_data)} points', fontsize=11, fontweight='bold')
                
                # Calculate bounds with some padding
                if len(x_coords) > 0 and len(y_coords) > 0 and len(z_coords) > 0:
                    x_range = max(x_coords) - min(x_coords)
                    y_range = max(y_coords) - min(y_coords)
                    z_range = max(z_coords) - min(z_coords)

                    max_range = max(x_range, y_range, z_range) or 1
                    padding = max_range * 0.1  # 10% padding

                    mid_x = (max(x_coords) + min(x_coords)) / 2
                    mid_y = (max(y_coords) + min(y_coords)) / 2
                    mid_z = (max(z_coords) + min(z_coords)) / 2
                    
                    # Set limits with padding
                    self.ax.set_xlim(mid_x - max_range/2 - padding, mid_x + max_range/2 + padding)
                    self.ax.set_ylim(mid_y - max_range/2 - padding, mid_y + max_range/2 + padding)
                    self.ax.set_zlim(mid_z - max_range/2 - padding, mid_z + max_range/2 + padding)

                    # Set equal aspect ratio for better 3D view (true scale)
                    try:
                        self.ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio
                    except:
                        # Fallback for older matplotlib versions
                        pass
                else:
                    # Default view if no data
                    self.ax.set_xlim(-10, 10)
                    self.ax.set_ylim(-10, 10)
                    self.ax.set_zlim(0, 20)
            else:
                # Clear plot if no data
                self.ax.set_xlabel('X (cm)', fontsize=10)
                self.ax.set_ylabel('Y (cm)', fontsize=10)
                self.ax.set_zlabel('Z (cm)', fontsize=10)
                self.ax.set_title('3D Scan Point Cloud - No data', fontsize=11)
                self.ax.set_xlim(-10, 10)
                self.ax.set_ylim(-10, 10)
                self.ax.set_zlim(0, 20)
            
            # Redraw canvas
            self.canvas.draw()
            self.canvas.flush_events()
            
        except Exception as e:
            self.log_info(f"Visualization error: {e}")
            import traceback
            traceback.print_exc()

    def export_stl(self):
        messagebox.showinfo("Export", "STL export not implemented in fixed version")

    def export_k(self):
        messagebox.showinfo("Export", ".k export not implemented in fixed version")

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