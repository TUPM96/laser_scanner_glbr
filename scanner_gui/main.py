"""
3D Scanner GUI Application
Python GUI for controlling 3D scanner and visualizing scan data
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
        
        # Test tab content
        self.setup_test_tab(test_tab)
        
        # Track test points
        self.test_points = []
        self.is_testing = False
        self.test_current_step = 0  # Track current motor X position for test
        self.test_current_step_z = 0  # Track current motor Z position for test
        self.lidar_reading_active = False  # Track if continuous reading is active
        # NEMA 17 motor: 1.8° per step = 200 steps per full revolution (full step mode)
        # Microstepping modes:
        # - Full step (MS1=LOW, MS2=LOW): 200 steps/rev
        # - 1/2 step (MS1=HIGH, MS2=LOW): 400 steps/rev
        # - 1/4 step (MS1=LOW, MS2=HIGH): 800 steps/rev
        # - 1/8 step (MS1=HIGH, MS2=HIGH): 1600 steps/rev
        # - 1/16 step: 3200 steps/rev
        # 
        # If your motor rotates 1.5 revolutions with 1600 steps, it means:
        # - Your driver is likely in 1/8 step mode (1600 steps should = 1 rev)
        # - But something is wrong with the configuration
        # - Try: 1600 / 1.5 = 1067 steps for 1 revolution (calibrated)
        # OR check your driver's microstepping jumpers/settings
        # steps_per_rev will be loaded from firmware on connection
        # Default value will be set from firmware's DEFAULT_STEPS_PER_REV (1600)
        
        # Port selection
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
        
        # Scan parameters - compact layout
        params_frame = ttk.LabelFrame(control_frame, text="Scan Parameters", padding="5")
        params_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=3)
        
        # Row 0: Theta and Z Travel
        # Note: θ Steps (number of measurement points per revolution) is synchronized from firmware
        # It will be loaded automatically when connecting to device
        ttk.Label(params_frame, text="θ Steps:").grid(row=0, column=0, sticky=tk.W, pady=1)
        self.theta_steps_var = tk.StringVar(value="200")  # Default matches firmware DEFAULT_THETA_STEPS_PER_REV
        ttk.Entry(params_frame, textvariable=self.theta_steps_var, width=8).grid(row=0, column=1, sticky=tk.W, padx=2)
        
        ttk.Label(params_frame, text="Z Travel (mm):").grid(row=0, column=2, sticky=tk.W, padx=(10,0), pady=1)
        self.z_travel_var = tk.StringVar(value="200")
        ttk.Entry(params_frame, textvariable=self.z_travel_var, width=8).grid(row=0, column=3, sticky=tk.W, padx=2)
        
        # Row 1: Z Height (Z Steps/mm is hidden - fixed at 200 for vitme phi 8)
        # Z Steps/mm is fixed at 200 (vitme phi 8: 1600 steps/rev / 8mm = 200 steps/mm)
        # It's synchronized from firmware but not shown in GUI since it's hardware-dependent
        self.z_steps_per_mm_var = tk.StringVar(value="200")  # Fixed: vitme phi 8
        
        ttk.Label(params_frame, text="Z Height (mm):").grid(row=1, column=0, sticky=tk.W, pady=1)
        self.z_layer_height_var = tk.StringVar(value="2.0")
        ttk.Entry(params_frame, textvariable=self.z_layer_height_var, width=8).grid(row=1, column=1, sticky=tk.W, padx=2)
        
        # Row 2: Scan Delay and Center Distance
        ttk.Label(params_frame, text="Delay (ms):").grid(row=2, column=0, sticky=tk.W, pady=1)
        self.scan_delay_var = tk.StringVar(value="50")
        ttk.Entry(params_frame, textvariable=self.scan_delay_var, width=8).grid(row=2, column=1, sticky=tk.W, padx=2)
        
        ttk.Label(params_frame, text="Khoảng cách tâm (cm):").grid(row=2, column=2, sticky=tk.W, padx=(10,0), pady=1)
        self.center_distance_var = tk.StringVar(value="15.0")
        ttk.Entry(params_frame, textvariable=self.center_distance_var, width=8).grid(row=2, column=3, sticky=tk.W, padx=2)
        
        # Row 3: Steps per Revolution
        # Note: This value is synchronized from firmware (DEFAULT_STEPS_PER_REV = 1600)
        # It will be loaded automatically when connecting to device
        ttk.Label(params_frame, text="Steps/Rev:").grid(row=3, column=0, sticky=tk.W, pady=1)
        self.steps_per_rev_var = tk.StringVar(value="1600")  # Default matches firmware DEFAULT_STEPS_PER_REV
        ttk.Entry(params_frame, textvariable=self.steps_per_rev_var, width=8).grid(row=3, column=1, sticky=tk.W, padx=2)
        
        # Send config button
        ttk.Button(params_frame, text="Send Config", command=self.send_config).grid(row=4, column=0, columnspan=4, pady=(5,0), sticky=(tk.W, tk.E))
        
        # Geometry parameters - simplified
        geometry_frame = ttk.LabelFrame(control_frame, text="Thông số hình học", padding="5")
        geometry_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=3)
        
        # Row 0: Bán kính đĩa quay
        ttk.Label(geometry_frame, text="Bán kính đĩa (cm):").grid(row=0, column=0, sticky=tk.W, pady=1)
        self.disk_radius_var = tk.StringVar(value="5.0")
        ttk.Entry(geometry_frame, textvariable=self.disk_radius_var, width=8).grid(row=0, column=1, sticky=tk.W, padx=2)
        
        # Note: Khoảng cách tâm đã có trong Scan Parameters (row=2, column=2)
        
        # Scan controls
        scan_frame = ttk.Frame(control_frame)
        scan_frame.grid(row=5, column=0, columnspan=3, pady=5, sticky=(tk.W, tk.E))
        
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
        manual_frame.grid(row=6, column=0, columnspan=3, pady=5, sticky=(tk.W, tk.E))
        
        self.move_to_top_btn = ttk.Button(manual_frame, text="Lên đỉnh", command=self.move_to_top, state=tk.DISABLED)
        self.move_to_top_btn.grid(row=0, column=0, padx=2, sticky=(tk.W, tk.E))
        
        self.home_btn = ttk.Button(manual_frame, text="Về Home", command=self.go_home, state=tk.DISABLED)
        self.home_btn.grid(row=0, column=1, padx=2, sticky=(tk.W, tk.E))
        
        manual_frame.columnconfigure(0, weight=1)
        manual_frame.columnconfigure(1, weight=1)
        scan_frame.columnconfigure(2, weight=1)
        
        # Progress
        progress_frame = ttk.Frame(control_frame)
        progress_frame.grid(row=7, column=0, columnspan=3, pady=2, sticky=(tk.W, tk.E))
        
        ttk.Label(progress_frame, text="Progress:").grid(row=0, column=0, sticky=tk.W)
        self.progress_var = tk.StringVar(value="0%")
        ttk.Label(progress_frame, textvariable=self.progress_var).grid(row=0, column=1, sticky=tk.W, padx=5)
        
        # Current angle display
        ttk.Label(progress_frame, text="Góc quay:").grid(row=1, column=0, sticky=tk.W, pady=(3,0))
        self.current_angle_var = tk.StringVar(value="0.0°")
        ttk.Label(progress_frame, textvariable=self.current_angle_var).grid(row=1, column=1, sticky=tk.W, padx=5, pady=(3,0))
        
        self.progress_bar = ttk.Progressbar(control_frame, mode='determinate')
        self.progress_bar.grid(row=7, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=2)
        
        # Export buttons
        export_frame = ttk.Frame(control_frame)
        export_frame.grid(row=8, column=0, columnspan=3, pady=3, sticky=(tk.W, tk.E))
        
        self.export_stl_btn = ttk.Button(export_frame, text="Export STL", command=self.export_stl, state=tk.NORMAL)
        self.export_stl_btn.grid(row=0, column=0, padx=2, sticky=(tk.W, tk.E))
        
        self.export_k_btn = ttk.Button(export_frame, text="Export .k", command=self.export_k, state=tk.NORMAL)
        self.export_k_btn.grid(row=0, column=1, padx=2, sticky=(tk.W, tk.E))
        
        export_frame.columnconfigure(0, weight=1)
        export_frame.columnconfigure(1, weight=1)
        
        # Data info - compact for 600px height
        info_frame = ttk.LabelFrame(control_frame, text="Scan Info", padding="5")
        info_frame.grid(row=9, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=3)
        
        self.info_text = tk.Text(info_frame, height=3, width=35, font=("Consolas", 8))
        self.info_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar = ttk.Scrollbar(info_frame, orient=tk.VERTICAL, command=self.info_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.info_text.configure(yscrollcommand=scrollbar.set)
        info_frame.rowconfigure(0, weight=1)
        info_frame.columnconfigure(0, weight=1)
        
        # Container frame for canvas and toolbar (use tk.Frame to allow pack)
        canvas_container = tk.Frame(viz_frame)
        canvas_container.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Matplotlib figure - optimized for 1000x600 window
        self.fig = Figure(figsize=(6, 5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_zlabel('Z (cm)')
        self.ax.set_title('3D Scan Point Cloud')
        
        self.canvas = FigureCanvasTkAgg(self.fig, canvas_container)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        # Toolbar (uses pack internally, so put it in a frame that allows pack)
        toolbar = NavigationToolbar2Tk(self.canvas, canvas_container)
        toolbar.update()
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)   # Notebook expands
        main_frame.columnconfigure(2, weight=1)   # Visualization expands
        main_frame.rowconfigure(0, weight=1)
        control_tab.columnconfigure(0, weight=1)
        control_tab.rowconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)
        control_frame.columnconfigure(3, weight=1)
        control_frame.rowconfigure(9, weight=1)  # Info frame expands
        scan_frame.columnconfigure(0, weight=1)
        scan_frame.columnconfigure(1, weight=1)
        viz_frame.columnconfigure(0, weight=1)
        viz_frame.rowconfigure(0, weight=1)
        
        # Bind tab change event to clear test points
        notebook.bind("<<NotebookTabChanged>>", self.on_tab_changed)
    
    def setup_test_tab(self, test_tab):
        """Setup Test tab content"""
        # Test instructions
        info_label = ttk.Label(test_tab, text="Test Mode: Measure 2 points (0° and 180°)", 
                              font=("Arial", 10, "bold"))
        info_label.grid(row=0, column=0, columnspan=3, pady=5, sticky=tk.W)
        
        # Motor control section
        motor_frame = ttk.LabelFrame(test_tab, text="Motor Control", padding="5")
        motor_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        # Motor X (Theta) control
        ttk.Label(motor_frame, text="Motor X (Theta):", font=("Arial", 9, "bold")).grid(row=0, column=0, columnspan=4, sticky=tk.W, pady=2)
        
        # Steps input for X
        ttk.Label(motor_frame, text="Steps:").grid(row=1, column=0, sticky=tk.W, padx=2)
        self.test_steps_var = tk.StringVar(value="1")
        steps_entry = ttk.Entry(motor_frame, textvariable=self.test_steps_var, width=8)
        steps_entry.grid(row=1, column=1, padx=2)
        
        # Direction buttons for X
        self.rotate_cw_btn = ttk.Button(motor_frame, text="→ CW", command=self.rotate_cw, state=tk.DISABLED)
        self.rotate_cw_btn.grid(row=1, column=2, padx=2)
        
        self.rotate_ccw_btn = ttk.Button(motor_frame, text="← CCW", command=self.rotate_ccw, state=tk.DISABLED)
        self.rotate_ccw_btn.grid(row=1, column=3, padx=2)
        
        # Full rotation buttons for X
        self.rotate_x_cw_full_btn = ttk.Button(motor_frame, text="X: 1 vòng CW", command=self.rotate_x_full_cw, state=tk.DISABLED)
        self.rotate_x_cw_full_btn.grid(row=1, column=4, padx=2)
        
        self.rotate_x_ccw_full_btn = ttk.Button(motor_frame, text="X: 1 vòng CCW", command=self.rotate_x_full_ccw, state=tk.DISABLED)
        self.rotate_x_ccw_full_btn.grid(row=1, column=5, padx=2)
        
        # Motor Z control
        ttk.Label(motor_frame, text="Motor Z:", font=("Arial", 9, "bold")).grid(row=2, column=0, columnspan=4, sticky=tk.W, pady=(5,2))
        
        # Steps input for Z
        ttk.Label(motor_frame, text="Steps:").grid(row=3, column=0, sticky=tk.W, padx=2)
        self.test_steps_z_var = tk.StringVar(value="1")
        steps_z_entry = ttk.Entry(motor_frame, textvariable=self.test_steps_z_var, width=8)
        steps_z_entry.grid(row=3, column=1, padx=2)
        
        # Direction buttons for Z
        self.rotate_z_cw_btn = ttk.Button(motor_frame, text="→ CW", command=self.rotate_z_cw, state=tk.DISABLED)
        self.rotate_z_cw_btn.grid(row=3, column=2, padx=2)
        
        self.rotate_z_ccw_btn = ttk.Button(motor_frame, text="← CCW", command=self.rotate_z_ccw, state=tk.DISABLED)
        self.rotate_z_ccw_btn.grid(row=3, column=3, padx=2)
        
        # Full rotation buttons for Z
        self.rotate_z_cw_full_btn = ttk.Button(motor_frame, text="Z: 1 vòng CW", command=self.rotate_z_full_cw, state=tk.DISABLED)
        self.rotate_z_cw_full_btn.grid(row=3, column=4, padx=2)
        
        self.rotate_z_ccw_full_btn = ttk.Button(motor_frame, text="Z: 1 vòng CCW", command=self.rotate_z_full_ccw, state=tk.DISABLED)
        self.rotate_z_ccw_full_btn.grid(row=3, column=5, padx=2)
        
        # Current position display
        ttk.Label(motor_frame, text="X Position:").grid(row=4, column=0, sticky=tk.W, padx=2, pady=2)
        self.test_position_var = tk.StringVar(value="0 steps (0°)")
        ttk.Label(motor_frame, textvariable=self.test_position_var).grid(row=4, column=1, columnspan=2, sticky=tk.W, padx=2)
        
        ttk.Label(motor_frame, text="Z Position:").grid(row=5, column=0, sticky=tk.W, padx=2, pady=2)
        self.test_position_z_var = tk.StringVar(value="0 steps")
        ttk.Label(motor_frame, textvariable=self.test_position_z_var).grid(row=5, column=1, columnspan=2, sticky=tk.W, padx=2)
        
        # LiDAR reading section
        lidar_frame = ttk.LabelFrame(test_tab, text="LiDAR Reading", padding="5")
        lidar_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        # Distance display
        ttk.Label(lidar_frame, text="Distance:").grid(row=0, column=0, sticky=tk.W, padx=2)
        self.lidar_distance_var = tk.StringVar(value="-- cm")
        distance_label = ttk.Label(lidar_frame, textvariable=self.lidar_distance_var, 
                                   font=("Arial", 12, "bold"), foreground="blue")
        distance_label.grid(row=0, column=1, sticky=tk.W, padx=5)
        
        # Start/Stop continuous reading
        self.lidar_read_btn = ttk.Button(lidar_frame, text="Start Reading", 
                                       command=self.toggle_lidar_reading, state=tk.DISABLED)
        self.lidar_read_btn.grid(row=0, column=2, padx=5)
        
        # Reading status
        self.lidar_status_var = tk.StringVar(value="Stopped")
        ttk.Label(lidar_frame, textvariable=self.lidar_status_var, foreground="gray").grid(
            row=1, column=0, columnspan=3, pady=2)
        
        # Test button
        self.test_btn = ttk.Button(test_tab, text="Start Test (0° → 180°)", command=self.start_test, 
                                   state=tk.DISABLED, width=25)
        self.test_btn.grid(row=3, column=0, columnspan=3, pady=5)
        
        # Test status
        self.test_status_label = ttk.Label(test_tab, text="Status: Ready", foreground="blue")
        self.test_status_label.grid(row=5, column=0, columnspan=3, pady=2)
        
        # Test results
        results_frame = ttk.LabelFrame(test_tab, text="Test Results", padding="10")
        results_frame.grid(row=5, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.test_results_text = tk.Text(results_frame, height=8, width=40, font=("Consolas", 9))
        self.test_results_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        test_scrollbar = ttk.Scrollbar(results_frame, orient=tk.VERTICAL, command=self.test_results_text.yview)
        test_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.test_results_text.configure(yscrollcommand=test_scrollbar.set)
        
        results_frame.rowconfigure(0, weight=1)
        results_frame.columnconfigure(0, weight=1)
        
        test_tab.columnconfigure(0, weight=1)
        test_tab.rowconfigure(5, weight=1)
    
    def on_tab_changed(self, event):
        """Handle tab change - clear test points when leaving test tab"""
        # Clear test points when switching tabs
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
                time.sleep(2)  # Wait for Arduino to reset
                self.is_connected = True
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text="Status: Connected", foreground="green")
                if hasattr(self, 'scan_up_btn'):
                    self.scan_up_btn.config(state=tk.NORMAL)
                if hasattr(self, 'scan_down_btn'):
                    self.scan_down_btn.config(state=tk.NORMAL)
                self.test_btn.config(state=tk.NORMAL)
                if hasattr(self, 'rotate_cw_btn'):
                    self.rotate_cw_btn.config(state=tk.NORMAL)
                    self.rotate_ccw_btn.config(state=tk.NORMAL)
                    self.rotate_x_cw_full_btn.config(state=tk.NORMAL)
                    self.rotate_x_ccw_full_btn.config(state=tk.NORMAL)
                if hasattr(self, 'rotate_z_cw_btn'):
                    self.rotate_z_cw_btn.config(state=tk.NORMAL)
                    self.rotate_z_ccw_btn.config(state=tk.NORMAL)
                    self.rotate_z_cw_full_btn.config(state=tk.NORMAL)
                    self.rotate_z_ccw_full_btn.config(state=tk.NORMAL)
                if hasattr(self, 'lidar_read_btn'):
                    self.lidar_read_btn.config(state=tk.NORMAL)
                
                # Start serial reading thread
                self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
                self.serial_thread.start()
                
                self.log_info("Connected to " + port)
                
                # Request current configuration from firmware
                time.sleep(0.5)  # Wait a bit for connection to stabilize
                if self.serial_conn:
                    self.serial_conn.write(b"GET_CONFIG\n")
                    self.log_info("Requesting configuration from device...")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to connect: {str(e)}")
        else:
            self.disconnect()
    
    def disconnect(self):
        """Disconnect from serial port"""
        if self.is_connected:
            if self.is_scanning:
                # Send STOP command to firmware
                if self.serial_conn:
                    self.serial_conn.write(b"STOP\n")
                self.is_scanning = False
            self.is_connected = False
            if self.serial_conn:
                self.serial_conn.close()
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="Status: Disconnected", foreground="red")
            if hasattr(self, 'scan_up_btn'):
                self.scan_up_btn.config(state=tk.DISABLED)
            if hasattr(self, 'scan_down_btn'):
                self.scan_down_btn.config(state=tk.DISABLED)
            self.pause_btn.config(state=tk.DISABLED)
            if hasattr(self, 'resume_btn'):
                self.resume_btn.config(state=tk.DISABLED)
            if hasattr(self, 'test_btn'):
                self.test_btn.config(state=tk.DISABLED)
            if hasattr(self, 'rotate_cw_btn'):
                self.rotate_cw_btn.config(state=tk.DISABLED)
                self.rotate_ccw_btn.config(state=tk.DISABLED)
                self.rotate_x_cw_full_btn.config(state=tk.DISABLED)
                self.rotate_x_ccw_full_btn.config(state=tk.DISABLED)
            if hasattr(self, 'rotate_z_cw_btn'):
                self.rotate_z_cw_btn.config(state=tk.DISABLED)
                self.rotate_z_ccw_btn.config(state=tk.DISABLED)
                self.rotate_z_cw_full_btn.config(state=tk.DISABLED)
                self.rotate_z_ccw_full_btn.config(state=tk.DISABLED)
            if hasattr(self, 'lidar_read_btn'):
                self.lidar_reading_active = False
                self.lidar_read_btn.config(state=tk.DISABLED)
                self.lidar_read_btn.config(text="Start Reading")
                self.lidar_status_var.set("Stopped")
                self.lidar_distance_var.set("-- cm")
            if hasattr(self, 'move_to_top_btn'):
                self.move_to_top_btn.config(state=tk.DISABLED)
            if hasattr(self, 'home_btn'):
                self.home_btn.config(state=tk.DISABLED)
            self.log_info("Disconnected")
    
    def read_serial(self):
        """Read data from serial port in background thread"""
        while self.is_connected and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.process_serial_data(line)
            except Exception as e:
                if self.is_connected:
                    self.log_info(f"Serial error: {str(e)}")
                break
    
    def process_serial_data(self, line):
        """Process incoming serial data"""
        if line.startswith("SCAN_START"):
            # Firmware starts a new scan, so clear data
            self.scan_data = []
            self.current_layer = 0
            self.current_step = 0
            self.progress_bar['value'] = 0
            self.progress_var.set("0% (0 points)")
            # Keep export buttons enabled - they will check if data exists when clicked
            self.log_info("Bắt đầu quét...")
            self.update_visualization()  # Clear visualization
            # Start step-by-step scan thread
            self.step_by_step_active = True
            if self.scan_step_thread is None or not self.scan_step_thread.is_alive():
                self.scan_step_thread = threading.Thread(target=self.send_scan_steps, daemon=True)
                self.scan_step_thread.start()
        elif line.startswith("SCAN_COMPLETE"):
            self.is_scanning = False
            self.step_by_step_active = False  # Stop sending SCAN_STEP commands
            if hasattr(self, 'scan_up_btn'):
                self.scan_up_btn.config(state=tk.NORMAL)
            if hasattr(self, 'scan_down_btn'):
                self.scan_down_btn.config(state=tk.NORMAL)
            self.pause_btn.config(state=tk.DISABLED)
            if hasattr(self, 'resume_btn'):
                self.resume_btn.config(state=tk.DISABLED)
            # Export buttons are always enabled - they check for data when clicked
            self.progress_bar['value'] = 100
            self.progress_var.set("100%")
            self.log_info("Quét hoàn thành")
            self.update_visualization()
        elif line.startswith("SCAN_PAUSED"):
            self.is_scanning = False
            self.pause_btn.config(state=tk.DISABLED)
            if hasattr(self, 'resume_btn'):
                self.resume_btn.config(state=tk.NORMAL)
            self.log_info("Scan đã tạm dừng")
        elif line.startswith("SCAN_RESUMED"):
            self.is_scanning = True
            if hasattr(self, 'resume_btn'):
                self.resume_btn.config(state=tk.DISABLED)
            self.pause_btn.config(state=tk.NORMAL)
            self.log_info("Scan đã tiếp tục")
        elif line.startswith("SCAN_STOPPED"):
            self.is_scanning = False
            if hasattr(self, 'scan_up_btn'):
                self.scan_up_btn.config(state=tk.NORMAL)
            if hasattr(self, 'scan_down_btn'):
                self.scan_down_btn.config(state=tk.NORMAL)
            self.pause_btn.config(state=tk.DISABLED)
            if hasattr(self, 'resume_btn'):
                self.resume_btn.config(state=tk.DISABLED)
            self.log_info("Scan stopped")
        elif line.startswith("CURRENT_CONFIG:"):
            # Parse and load configuration from firmware
            try:
                config_str = line.replace("CURRENT_CONFIG:", "")
                values = config_str.strip().split(',')
                if len(values) >= 5:
                    # θ Steps - synchronized from firmware (single source of truth)
                    self.theta_steps_var.set(values[0])
                    self.log_info(f"θ Steps synchronized from firmware: {values[0]}")
                    self.z_travel_var.set(values[1])
                    # Z Steps/mm - synchronized from firmware (hidden in GUI, fixed for vitme phi 8)
                    self.z_steps_per_mm_var.set(values[2])
                    # No log message since it's hidden from user
                    # z_steps_per_layer is now auto-calculated, but we can calculate z_layer_height from it
                    if len(values) >= 4:
                        z_steps_per_layer = int(values[3])
                        z_steps_per_mm = int(values[2])
                        if z_steps_per_mm > 0:
                            z_layer_height = z_steps_per_layer / z_steps_per_mm
                            self.z_layer_height_var.set(f"{z_layer_height:.2f}")
                            self.log_info(f"Z layer height calculated: {z_steps_per_layer} steps / {z_steps_per_mm} steps/mm = {z_layer_height:.2f} mm")
                    self.scan_delay_var.set(values[4])
                    # Center distance is optional (for backward compatibility)
                    if len(values) >= 6:
                        self.center_distance_var.set(values[5])
                    # Steps per revolution - synchronized from firmware (single source of truth)
                    if len(values) >= 7:
                        self.steps_per_rev_var.set(values[6])
                        self.log_info(f"Steps/Rev synchronized from firmware: {values[6]}")
                    self.log_info("Configuration loaded from device")
            except Exception as e:
                self.log_info(f"Error loading config: {str(e)}")
        elif line.startswith("HOME_COMPLETE"):
            self.log_info("Home position reached")
        elif line.startswith("MOVE_TO_TOP_COMPLETE"):
            self.log_info("Top position reached")
        elif line.startswith("CONFIG_OK"):
            self.log_info("Configuration applied: " + line)
        elif line.startswith("CONFIG_ERROR"):
            self.log_info("Configuration error: " + line)
            messagebox.showerror("Config Error", line)
        elif line.startswith("TEST_POINT:"):
            # Parse test point: TEST_POINT:angle,distance
            try:
                test_str = line.replace("TEST_POINT:", "")
                parts = test_str.strip().split(',')
                if len(parts) == 2:
                    angle = float(parts[0])
                    distance = float(parts[1])
                    self.add_test_point(angle, distance)
            except Exception as e:
                self.log_info(f"Error parsing test point: {str(e)}")
        elif line.startswith("ROTATED:"):
            # Parse rotation confirmation: ROTATED:steps (negative for CCW)
            try:
                steps_str = line.replace("ROTATED:", "")
                steps = int(steps_str.strip())
                theta_steps = int(self.theta_steps_var.get())
                if steps < 0:
                    # Counter-clockwise
                    self.test_current_step = (self.test_current_step - abs(steps) + theta_steps) % theta_steps
                else:
                    # Clockwise
                    self.test_current_step = (self.test_current_step + steps) % theta_steps
                self.update_test_position()
            except Exception as e:
                self.log_info(f"Error parsing rotation: {str(e)}")
        elif line.startswith("LIDAR_DISTANCE:"):
            # Parse continuous lidar reading: LIDAR_DISTANCE:distance
            try:
                dist_str = line.replace("LIDAR_DISTANCE:", "")
                distance = float(dist_str.strip())
                if hasattr(self, 'lidar_distance_var'):
                    self.lidar_distance_var.set(f"{distance:.2f} cm")
                if hasattr(self, 'lidar_status_var'):
                    self.lidar_status_var.set("Reading...")
            except Exception as e:
                if hasattr(self, 'lidar_status_var'):
                    self.lidar_status_var.set("Error reading")
        elif line.startswith("ROTATED_Z:"):
            # Parse Z motor rotation confirmation: ROTATED_Z:steps (negative for CCW)
            try:
                steps_str = line.replace("ROTATED_Z:", "")
                steps = int(steps_str.strip())
                if steps < 0:
                    # Counter-clockwise
                    self.test_current_step_z -= abs(steps)
                else:
                    # Clockwise
                    self.test_current_step_z += steps
                self.update_test_position()
            except Exception as e:
                self.log_info(f"Error parsing Z rotation: {str(e)}")
        elif ',' in line:
            # Parse scan data: "LAYER,STEP,DISTANCE" or "LAYER,STEP,DISTANCE,ANGLE"
            try:
                parts = line.split(',')
                if len(parts) >= 3:
                    layer = int(parts[0])
                    step = int(parts[1])
                    distance = float(parts[2])
                    
                    # Always update progress and current position
                    self.current_layer = layer
                    self.current_step = step
                    
                    # Get angle from firmware if provided, otherwise calculate
                    if len(parts) >= 4:
                        # Angle provided by firmware
                        self.current_angle = float(parts[3])
                    else:
                        # Calculate angle from step
                        try:
                            theta_steps = int(self.theta_steps_var.get())
                            if theta_steps > 0:
                                self.current_angle = (step % theta_steps) * 360.0 / theta_steps
                        except:
                            self.current_angle = 0.0
                    
                    # Update angle display
                    if hasattr(self, 'current_angle_var'):
                        self.current_angle_var.set(f"{self.current_angle:.1f}°")
                    
                    # Skip invalid points (distance = 0 means LiDAR error)
                    # Don't add to scan_data, just skip to next point
                    if distance <= 0 or distance > 1200:
                        # Skip this point - don't add to scan_data, don't display in 3D view
                        # Just return early, don't process this point
                        return
                    
                    # Add valid point to scan_data
                    self.scan_data.append([layer, step, distance])
                    
                    # Update progress
                    theta_steps = int(self.theta_steps_var.get())
                    z_travel = int(self.z_travel_var.get())
                    z_steps_per_mm = int(self.z_steps_per_mm_var.get())
                    z_layer_height = float(self.z_layer_height_var.get())
                    # Auto-calculate z_steps_per_layer from z_layer_height
                    z_steps_per_layer = int(z_layer_height * z_steps_per_mm)
                    z_layers = (z_travel * z_steps_per_mm) // z_steps_per_layer if z_steps_per_layer > 0 else 0
                    total_points = theta_steps * z_layers
                    current_point = layer * theta_steps + step
                    if total_points > 0:
                        progress = (current_point / total_points) * 100
                        self.progress_bar['value'] = progress
                        self.progress_var.set(f"{progress:.1f}% ({len(self.scan_data)} points)")
                    
                    # Export buttons are always enabled - they check for data when clicked
                    
                    # Update visualization more frequently (every 5 points if we have data, every 10 otherwise)
                    update_interval = 5 if len(self.scan_data) > 0 else 10
                    if len(self.scan_data) % update_interval == 0:
                        self.update_visualization()
            except ValueError:
                pass
    
    def start_scan_up(self):
        """Start scanning from bottom to top (upward)"""
        if not self.is_connected:
            messagebox.showerror("Error", "Not connected to device")
            return
        
        # Auto-send config before starting scan to ensure firmware has correct values
        try:
            # Send config silently (without messagebox)
            self._send_config_silent()
            time.sleep(0.2)  # Wait a bit for config to be processed
        except:
            pass  # Continue even if config send fails
        
        self.is_scanning = True
        if hasattr(self, 'scan_up_btn'):
            self.scan_up_btn.config(state=tk.DISABLED)
        if hasattr(self, 'scan_down_btn'):
            self.scan_down_btn.config(state=tk.DISABLED)
        self.pause_btn.config(state=tk.NORMAL)
        if hasattr(self, 'resume_btn'):
            self.resume_btn.config(state=tk.DISABLED)
        # Export buttons remain enabled - they check for data when clicked
        self.progress_bar['value'] = 0
        self.progress_var.set("0%")
        
        if self.serial_conn:
            self.serial_conn.write(b"START_UP\n")
        self.log_info("Bắt đầu quét từ dưới lên...")
    
    def start_scan_down(self):
        """Start scanning from top to bottom (downward)"""
        if not self.is_connected:
            messagebox.showerror("Error", "Not connected to device")
            return
        
        # Auto-send config before starting scan to ensure firmware has correct values
        try:
            # Send config silently (without messagebox)
            self._send_config_silent()
            time.sleep(0.2)  # Wait a bit for config to be processed
        except:
            pass  # Continue even if config send fails
        
        self.is_scanning = True
        if hasattr(self, 'scan_up_btn'):
            self.scan_up_btn.config(state=tk.DISABLED)
        if hasattr(self, 'scan_down_btn'):
            self.scan_down_btn.config(state=tk.DISABLED)
        self.pause_btn.config(state=tk.NORMAL)
        if hasattr(self, 'resume_btn'):
            self.resume_btn.config(state=tk.DISABLED)
        # Export buttons remain enabled - they check for data when clicked
        self.progress_bar['value'] = 0
        self.progress_var.set("0%")
        
        if self.serial_conn:
            self.serial_conn.write(b"START_DOWN\n")
        self.log_info("Bắt đầu quét từ trên xuống...")
    
    def send_scan_steps(self):
        """Send SCAN_STEP commands continuously until scan is complete"""
        while self.step_by_step_active and self.is_scanning and self.is_connected:
            if self.serial_conn:
                try:
                    self.serial_conn.write(b"SCAN_STEP\n")
                    time.sleep(0.05)  # Small delay between steps (50ms)
                except Exception as e:
                    self.log_info(f"Error sending SCAN_STEP: {str(e)}")
                    break
            else:
                break
    
    def pause_scan(self):
        """Pause scanning"""
        self.step_by_step_active = False  # Stop sending SCAN_STEP commands
        if self.serial_conn:
            self.serial_conn.write(b"STOP\n")
        self.pause_btn.config(state=tk.DISABLED)
        self.resume_btn.config(state=tk.NORMAL)
        self.log_info("Tạm dừng quét...")
    
    def resume_scan(self):
        """Resume paused scanning"""
        if not self.is_connected:
            return
        if self.serial_conn:
            self.serial_conn.write(b"RESUME\n")
        # Restart step-by-step scan thread
        self.step_by_step_active = True
        if self.scan_step_thread is None or not self.scan_step_thread.is_alive():
            self.scan_step_thread = threading.Thread(target=self.send_scan_steps, daemon=True)
            self.scan_step_thread.start()
        self.resume_btn.config(state=tk.DISABLED)
        self.pause_btn.config(state=tk.NORMAL)
        self.log_info("Tiếp tục quét...")
    
    def clear_data(self):
        """Clear scan data and reset visualization"""
        self.scan_data = []
        self.current_layer = 0
        self.current_step = 0
        self.current_angle = 0.0  # Current rotation angle in degrees
        self.progress_bar['value'] = 0
        self.progress_var.set("0%")
        # Export buttons remain enabled - they check for data when clicked
        
        # Clear visualization
        self.ax.clear()
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_zlabel('Z (cm)')
        self.ax.set_title('3D Scan Point Cloud')
        self.canvas.draw()
        
        self.log_info("Scan data cleared")
    
    def move_to_top(self):
        """Move Z motor to top position"""
        if not self.is_connected:
            messagebox.showerror("Error", "Not connected to device")
            return
        if self.serial_conn:
            self.serial_conn.write(b"MOVE_TO_TOP\n")
            self.log_info("Moving to top position...")
    
    def go_home(self):
        """Move Z motor to home position"""
        if not self.is_connected:
            messagebox.showerror("Error", "Not connected to device")
            return
        if self.serial_conn:
            self.serial_conn.write(b"HOME\n")
            self.log_info("Returning to home position...")
    
    def send_config(self):
        """Send configuration to firmware"""
        if not self.is_connected:
            messagebox.showerror("Error", "Not connected to device")
            return
        
        try:
            # Get values from GUI
            theta_steps = int(self.theta_steps_var.get())
            z_travel = int(self.z_travel_var.get())
            z_steps_per_mm = int(self.z_steps_per_mm_var.get())
            z_layer_height = float(self.z_layer_height_var.get())
            scan_delay = int(self.scan_delay_var.get())
            center_distance = float(self.center_distance_var.get())
            steps_per_rev = int(self.steps_per_rev_var.get())
            
            # Auto-calculate z_steps_per_layer from z_layer_height
            z_steps_per_layer = int(z_layer_height * z_steps_per_mm)
            
            # Validate theta_steps limits
            MIN_THETA_STEPS = 4
            MAX_THETA_STEPS = 3600
            
            if theta_steps < MIN_THETA_STEPS or theta_steps > MAX_THETA_STEPS:
                messagebox.showerror("Error", f"θ Steps must be between {MIN_THETA_STEPS} and {MAX_THETA_STEPS}")
                return
            
            # Validate that theta_steps doesn't exceed steps_per_rev
            if theta_steps > steps_per_rev:
                messagebox.showerror("Error", f"θ Steps ({theta_steps}) cannot exceed Steps/Rev ({steps_per_rev})")
                return
            
            # Validate other values
            if theta_steps <= 0 or z_travel <= 0 or z_steps_per_mm <= 0 or z_layer_height <= 0 or scan_delay < 0 or center_distance <= 0 or steps_per_rev <= 0:
                messagebox.showerror("Error", "Invalid configuration values")
                return
            
            # Send config command: CONFIG,theta_steps,z_travel,z_steps_per_mm,z_steps_per_layer,scan_delay,center_distance,steps_per_rev
            # Note: z_steps_per_layer is auto-calculated from z_layer_height * z_steps_per_mm
            config_cmd = f"CONFIG,{theta_steps},{z_travel},{z_steps_per_mm},{z_steps_per_layer},{scan_delay},{center_distance},{steps_per_rev}\n"
            
            if self.serial_conn:
                self.serial_conn.write(config_cmd.encode())
                self.log_info(f"Config sent: θ={theta_steps}, Z={z_travel}mm, {z_steps_per_mm}steps/mm, Z height={z_layer_height}mm (auto: {z_steps_per_layer}steps/layer), delay={scan_delay}ms, center={center_distance}cm")
                messagebox.showinfo("Success", "Configuration sent to device")
        except ValueError:
            messagebox.showerror("Error", "Invalid input values. Please enter numbers only.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send config: {str(e)}")
    
    def update_visualization(self):
        """Update 3D visualization with current scan data and test points"""
        self.ax.clear()
        
        # Get cartesian points from scan data
        points = None
        if self.scan_data:
            points = self.get_cartesian_points()
        
        # Plot scan points with mesh/surface
        if points is not None and len(points) > 0:
            x = points[:, 0]
            y = points[:, 1]
            z = points[:, 2]
            
            # Try to create mesh from structured scan data
            try:
                # Reconstruct layer and step structure from scan_data
                layers_dict = {}
                point_indices = {}  # Map (layer, step) -> index in points array
                for i, data in enumerate(self.scan_data):
                    layer = data[0]
                    step = data[1]
                    if layer not in layers_dict:
                        layers_dict[layer] = []
                    layers_dict[layer].append(step)
                    point_indices[(layer, step)] = i
                
                # Create mesh by connecting adjacent points
                if len(layers_dict) > 0:
                    # Get theta_steps to know the structure
                    try:
                        theta_steps = int(self.theta_steps_var.get())
                    except:
                        theta_steps = 200  # Default
                    
                    # Check if we have enough structured data
                    has_structure = len(layers_dict) > 1
                    if has_structure:
                        # Check if first layer has multiple steps
                        first_layer = min(layers_dict.keys())
                        has_structure = has_structure and len(layers_dict[first_layer]) > 1
                    
                    if has_structure:
                        # Create triangles based on scan structure
                        triangles = []
                        layers = sorted(layers_dict.keys())
                        
                        for layer_idx in range(len(layers) - 1):
                            layer1 = layers[layer_idx]
                            layer2 = layers[layer_idx + 1]
                            
                            # Get all steps for both layers
                            steps1 = sorted(layers_dict[layer1])
                            steps2 = sorted(layers_dict[layer2])
                            
                            # Create quads (two triangles) between adjacent points
                            for i in range(len(steps1) - 1):
                                step1_curr = steps1[i]
                                step1_next = steps1[i + 1]
                                
                                # Find corresponding steps in layer2
                                # Try to find step at same position or closest
                                step2_curr = None
                                step2_next = None
                                
                                # Find closest step in layer2
                                for step2 in steps2:
                                    if step2_curr is None or abs(step2 - step1_curr) < abs(step2_curr - step1_curr):
                                        step2_curr = step2
                                    if step2_next is None or abs(step2 - step1_next) < abs(step2_next - step1_next):
                                        step2_next = step2
                                
                                # Create triangles if we have all 4 points
                                if (layer1, step1_curr) in point_indices and \
                                   (layer1, step1_next) in point_indices and \
                                   (layer2, step2_curr) in point_indices and \
                                   (layer2, step2_next) in point_indices:
                                    
                                    idx1 = point_indices[(layer1, step1_curr)]
                                    idx2 = point_indices[(layer1, step1_next)]
                                    idx3 = point_indices[(layer2, step2_curr)]
                                    idx4 = point_indices[(layer2, step2_next)]
                                    
                                    # Validate indices are within bounds
                                    max_idx = len(points) - 1
                                    if idx1 <= max_idx and idx2 <= max_idx and idx3 <= max_idx and idx4 <= max_idx:
                                        # Triangle 1: (idx1, idx2, idx3)
                                        triangles.append([idx1, idx2, idx3])
                                        # Triangle 2: (idx2, idx4, idx3)
                                        triangles.append([idx2, idx4, idx3])
                            
                            # Also connect points within same layer (circular connection)
                            if len(steps1) > 2:
                                # Connect last step to first step in layer1
                                step1_last = steps1[-1]
                                step1_first = steps1[0]
                                
                                # Find corresponding steps in layer2
                                step2_last = None
                                step2_first = None
                                for step2 in steps2:
                                    if step2_last is None or abs(step2 - step1_last) < abs(step2_last - step1_last):
                                        step2_last = step2
                                    if step2_first is None or abs(step2 - step1_first) < abs(step2_first - step1_first):
                                        step2_first = step2
                                
                                if (layer1, step1_last) in point_indices and \
                                   (layer1, step1_first) in point_indices and \
                                   (layer2, step2_last) in point_indices and \
                                   (layer2, step2_first) in point_indices:
                                    
                                    idx1 = point_indices[(layer1, step1_last)]
                                    idx2 = point_indices[(layer1, step1_first)]
                                    idx3 = point_indices[(layer2, step2_last)]
                                    idx4 = point_indices[(layer2, step2_first)]
                                    
                                    # Validate indices are within bounds
                                    max_idx = len(points) - 1
                                    if idx1 <= max_idx and idx2 <= max_idx and idx3 <= max_idx and idx4 <= max_idx:
                                        triangles.append([idx1, idx2, idx3])
                                        triangles.append([idx2, idx4, idx3])
                        
                        if triangles:
                            # Use structured mesh
                            triangles = np.array(triangles)
                            self.ax.plot_trisurf(x, y, z, triangles=triangles,
                                                cmap='viridis', alpha=0.7, edgecolor='none',
                                                linewidth=0, antialiased=True, label='Scan surface')
                            
                            # Also show points for reference
                            self.ax.scatter(x, y, z, c=z, cmap='viridis', s=2, alpha=0.3, 
                                          edgecolors='none', zorder=1)
                        else:
                            # Fallback to Delaunay if structured mesh fails
                            from scipy.spatial import Delaunay
                            tri = Delaunay(np.column_stack([x, y]))
                            self.ax.plot_trisurf(x, y, z, triangles=tri.simplices, 
                                                cmap='viridis', alpha=0.7, edgecolor='none',
                                                linewidth=0, antialiased=True, label='Scan surface')
                            self.ax.scatter(x, y, z, c=z, cmap='viridis', s=2, alpha=0.3, 
                                          edgecolors='none', zorder=1)
                    else:
                        # Not enough structure, use Delaunay triangulation
                        from scipy.spatial import Delaunay
                        tri = Delaunay(np.column_stack([x, y]))
                        self.ax.plot_trisurf(x, y, z, triangles=tri.simplices, 
                                            cmap='viridis', alpha=0.7, edgecolor='none',
                                            linewidth=0, antialiased=True, label='Scan surface')
                        self.ax.scatter(x, y, z, c=z, cmap='viridis', s=2, alpha=0.3, 
                                      edgecolors='none', zorder=1)
                else:
                    # Not enough data, just show points
                    self.ax.scatter(x, y, z, c=z, cmap='viridis', s=1, alpha=0.6, label='Scan points')
            except Exception as e:
                # Fallback to scatter if mesh creation fails
                import traceback
                print(f"Mesh creation error: {e}")
                print(traceback.format_exc())
                self.ax.scatter(x, y, z, c=z, cmap='viridis', s=1, alpha=0.6, label='Scan points')
        
        # Plot test points
        if self.test_points and len(self.test_points) > 0:
            test_x = [p[0] for p in self.test_points]
            test_y = [p[1] for p in self.test_points]
            test_z = [p[2] for p in self.test_points]
            
            self.ax.scatter(test_x, test_y, test_z, c='red', s=100, marker='o', 
                          edgecolors='black', linewidths=2, label='Test points', zorder=5)
            
            # Draw line between test points if 2 points
            if len(self.test_points) == 2:
                self.ax.plot([test_x[0], test_x[1]], [test_y[0], test_y[1]], 
                           [test_z[0], test_z[1]], 'r--', linewidth=2, alpha=0.5)
        
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_zlabel('Z (cm)')
        total_points = len(points) if points is not None else 0
        test_count = len(self.test_points) if self.test_points else 0
        self.ax.set_title(f'3D View ({total_points} scan points, {test_count} test points)')
        
        if total_points > 0 or test_count > 0:
            self.ax.legend()
        
        self.canvas.draw()
    
    def start_test(self):
        """Start test mode: measure point at 0°, rotate 180°, measure again"""
        if not self.is_connected:
            messagebox.showerror("Error", "Not connected to device")
            return
        
        if self.is_testing:
            return
        
        self.is_testing = True
        self.test_btn.config(state=tk.DISABLED)
        self.test_status_label.config(text="Status: Testing...", foreground="orange")
        self.test_points = []
        self.test_results_text.delete(1.0, tk.END)
        self.test_results_text.insert(tk.END, "Starting test...\n")
        
        # Start test in background thread
        test_thread = threading.Thread(target=self.run_test, daemon=True)
        test_thread.start()
    
    def run_test(self):
        """Run test sequence"""
        try:
            # Measure first point at current position (0°)
            self.test_results_text.insert(tk.END, "Measuring point 1 at 0°...\n")
            self.test_results_text.see(tk.END)
            
            if self.serial_conn:
                self.serial_conn.write(b"TEST_POINT\n")
            
            # Wait for response (will be handled by process_serial_data)
            time.sleep(1.5)
            
            # Rotate 180 degrees
            theta_steps = int(self.theta_steps_var.get())
            steps_180 = theta_steps // 2
            
            self.test_results_text.insert(tk.END, f"Rotating {steps_180} steps (180°)...\n")
            self.test_results_text.see(tk.END)
            
            if self.serial_conn:
                self.serial_conn.write(f"ROTATE,{steps_180}\n".encode())
            
            time.sleep(2.5)  # Wait for rotation to complete
            
            # Measure second point at 180°
            self.test_results_text.insert(tk.END, "Measuring point 2 at 180°...\n")
            self.test_results_text.see(tk.END)
            
            if self.serial_conn:
                self.serial_conn.write(b"TEST_POINT\n")
            
            time.sleep(1.5)  # Wait for response
            
            self.test_results_text.insert(tk.END, "Test completed!\n")
            self.test_results_text.see(tk.END)
            
        except Exception as e:
            self.test_results_text.insert(tk.END, f"Error: {str(e)}\n")
            self.test_results_text.see(tk.END)
        finally:
            self.is_testing = False
            self.test_btn.config(state=tk.NORMAL)
            self.test_status_label.config(text="Status: Ready", foreground="green")
    
    def add_test_point(self, angle, distance):
        """Add a test point to visualization"""
        # Convert angle (degrees) and distance to cartesian
        center_distance = float(self.center_distance_var.get())
        r = center_distance - distance
        angle_rad = np.radians(angle)
        
        x = r * np.cos(angle_rad)
        y = r * np.sin(angle_rad)
        z = 0  # Test points at z=0
        
        self.test_points.append([x, y, z])
        
        # Update results text
        self.test_results_text.insert(tk.END, f"Point {len(self.test_points)}: Angle={angle}°, Distance={distance:.2f}cm, R={r:.2f}cm\n")
        self.test_results_text.insert(tk.END, f"  Position: X={x:.2f}cm, Y={y:.2f}cm, Z={z:.2f}cm\n")
        self.test_results_text.see(tk.END)
        
        # Update visualization
        self.update_visualization()
        
        # If we have 2 points, calculate distance
        if len(self.test_points) == 2:
            p1 = np.array(self.test_points[0])
            p2 = np.array(self.test_points[1])
            dist = np.linalg.norm(p2 - p1)
            self.test_results_text.insert(tk.END, f"\nDistance between points: {dist:.2f}cm\n")
            self.test_results_text.see(tk.END)
    
    def rotate_cw(self):
        """Rotate motor clockwise"""
        if not self.is_connected:
            return
        
        try:
            steps = int(self.test_steps_var.get())
            if steps <= 0:
                messagebox.showerror("Error", "Steps must be greater than 0")
                return
            
            if self.serial_conn:
                self.serial_conn.write(f"ROTATE,{steps}\n".encode())
                self.test_results_text.insert(tk.END, f"Rotating CW {steps} steps...\n")
                self.test_results_text.see(tk.END)
        except ValueError:
            messagebox.showerror("Error", "Invalid steps value")
        except Exception as e:
            self.test_results_text.insert(tk.END, f"Error: {str(e)}\n")
            self.test_results_text.see(tk.END)
    
    def rotate_ccw(self):
        """Rotate motor counter-clockwise"""
        if not self.is_connected:
            return
        
        try:
            steps = int(self.test_steps_var.get())
            if steps <= 0:
                messagebox.showerror("Error", "Steps must be greater than 0")
                return
            
            # For CCW, we rotate in reverse direction
            if self.serial_conn:
                self.serial_conn.write(f"ROTATE_CCW,{steps}\n".encode())
                self.test_results_text.insert(tk.END, f"Rotating CCW {steps} steps...\n")
                self.test_results_text.see(tk.END)
        except ValueError:
            messagebox.showerror("Error", "Invalid steps value")
        except Exception as e:
            self.test_results_text.insert(tk.END, f"Error: {str(e)}\n")
            self.test_results_text.see(tk.END)
    
    def update_test_position(self):
        """Update test position display"""
        if hasattr(self, 'test_position_var') and hasattr(self, 'theta_steps_var'):
            try:
                theta_steps = int(self.theta_steps_var.get())
                angle = (self.test_current_step % theta_steps) * 360.0 / theta_steps
                self.test_position_var.set(f"{self.test_current_step} steps ({angle:.1f}°)")
            except:
                pass
        
        # Update Z position
        if hasattr(self, 'test_position_z_var'):
            try:
                self.test_position_z_var.set(f"{self.test_current_step_z} steps")
            except:
                pass
    
    def rotate_x_full_cw(self):
        """Rotate motor X one full revolution clockwise"""
        if not self.is_connected:
            return
        try:
            steps = int(self.steps_per_rev_var.get())
            if steps <= 0:
                messagebox.showerror("Error", "Steps per revolution must be greater than 0")
                return
            if self.serial_conn:
                self.serial_conn.write(f"ROTATE,{steps}\n".encode())
                self.test_results_text.insert(tk.END, f"Rotating X motor 1 full revolution CW ({steps} steps)...\n")
                self.test_results_text.see(tk.END)
        except ValueError:
            messagebox.showerror("Error", "Invalid steps per revolution value")
        except Exception as e:
            self.test_results_text.insert(tk.END, f"Error: {str(e)}\n")
            self.test_results_text.see(tk.END)
    
    def rotate_x_full_ccw(self):
        """Rotate motor X one full revolution counter-clockwise"""
        if not self.is_connected:
            return
        try:
            steps = int(self.steps_per_rev_var.get())
            if steps <= 0:
                messagebox.showerror("Error", "Steps per revolution must be greater than 0")
                return
            if self.serial_conn:
                self.serial_conn.write(f"ROTATE_CCW,{steps}\n".encode())
                self.test_results_text.insert(tk.END, f"Rotating X motor 1 full revolution CCW ({steps} steps)...\n")
                self.test_results_text.see(tk.END)
        except ValueError:
            messagebox.showerror("Error", "Invalid steps per revolution value")
        except Exception as e:
            self.test_results_text.insert(tk.END, f"Error: {str(e)}\n")
            self.test_results_text.see(tk.END)
    
    def rotate_z_cw(self):
        """Rotate motor Z clockwise"""
        if not self.is_connected:
            return
        try:
            steps = int(self.test_steps_z_var.get())
            if steps <= 0:
                messagebox.showerror("Error", "Steps must be greater than 0")
                return
            if self.serial_conn:
                self.serial_conn.write(f"ROTATE_Z,{steps}\n".encode())
                self.test_results_text.insert(tk.END, f"Rotating Z motor CW {steps} steps...\n")
                self.test_results_text.see(tk.END)
        except ValueError:
            messagebox.showerror("Error", "Invalid steps value")
        except Exception as e:
            self.test_results_text.insert(tk.END, f"Error: {str(e)}\n")
            self.test_results_text.see(tk.END)
    
    def rotate_z_ccw(self):
        """Rotate motor Z counter-clockwise"""
        if not self.is_connected:
            return
        try:
            steps = int(self.test_steps_z_var.get())
            if steps <= 0:
                messagebox.showerror("Error", "Steps must be greater than 0")
                return
            if self.serial_conn:
                self.serial_conn.write(f"ROTATE_Z_CCW,{steps}\n".encode())
                self.test_results_text.insert(tk.END, f"Rotating Z motor CCW {steps} steps...\n")
                self.test_results_text.see(tk.END)
        except ValueError:
            messagebox.showerror("Error", "Invalid steps value")
        except Exception as e:
            self.test_results_text.insert(tk.END, f"Error: {str(e)}\n")
            self.test_results_text.see(tk.END)
    
    def rotate_z_full_cw(self):
        """Rotate motor Z one full revolution clockwise"""
        if not self.is_connected:
            return
        try:
            steps = int(self.steps_per_rev_var.get())
            if steps <= 0:
                messagebox.showerror("Error", "Steps per revolution must be greater than 0")
                return
            if self.serial_conn:
                self.serial_conn.write(f"ROTATE_Z,{steps}\n".encode())
                self.test_results_text.insert(tk.END, f"Rotating Z motor 1 full revolution CW ({steps} steps)...\n")
                self.test_results_text.see(tk.END)
        except ValueError:
            messagebox.showerror("Error", "Invalid steps per revolution value")
        except Exception as e:
            self.test_results_text.insert(tk.END, f"Error: {str(e)}\n")
            self.test_results_text.see(tk.END)
    
    def rotate_z_full_ccw(self):
        """Rotate motor Z one full revolution counter-clockwise"""
        if not self.is_connected:
            return
        try:
            steps = int(self.steps_per_rev_var.get())
            if steps <= 0:
                messagebox.showerror("Error", "Steps per revolution must be greater than 0")
                return
            if self.serial_conn:
                self.serial_conn.write(f"ROTATE_Z_CCW,{steps}\n".encode())
                self.test_results_text.insert(tk.END, f"Rotating Z motor 1 full revolution CCW ({steps} steps)...\n")
                self.test_results_text.see(tk.END)
        except ValueError:
            messagebox.showerror("Error", "Invalid steps per revolution value")
        except Exception as e:
            self.test_results_text.insert(tk.END, f"Error: {str(e)}\n")
            self.test_results_text.see(tk.END)
    
    def toggle_lidar_reading(self):
        """Toggle continuous LiDAR reading"""
        if not self.is_connected:
            return
        
        if not self.lidar_reading_active:
            # Start continuous reading
            self.lidar_reading_active = True
            self.lidar_read_btn.config(text="Stop Reading")
            self.lidar_status_var.set("Starting...")
            self.lidar_distance_var.set("-- cm")
            
            # Start reading thread
            reading_thread = threading.Thread(target=self.continuous_lidar_read, daemon=True)
            reading_thread.start()
        else:
            # Stop continuous reading
            self.lidar_reading_active = False
            self.lidar_read_btn.config(text="Start Reading")
            self.lidar_status_var.set("Stopped")
    
    def continuous_lidar_read(self):
        """Continuously read from LiDAR"""
        while self.lidar_reading_active and self.is_connected:
            try:
                if self.serial_conn:
                    self.serial_conn.write(b"READ_LIDAR\n")
                time.sleep(0.1)  # Read every 100ms
            except Exception as e:
                if self.lidar_reading_active:
                    self.lidar_status_var.set(f"Error: {str(e)}")
                break
    
    def export_stl(self):
        """Export scan data to STL file"""
        if not self.scan_data:
            messagebox.showwarning("Warning", "No scan data to export")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".stl",
            filetypes=[("STL files", "*.stl"), ("All files", "*.*")]
        )
        
        if filename:
            try:
                self.generate_stl(filename)
                messagebox.showinfo("Success", f"STL file saved to {filename}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to export STL: {str(e)}")
    
    def get_cartesian_points(self):
        """Convert scan data to cartesian coordinates"""
        if not self.scan_data:
            return None
        
        # Convert scan data to points
        data = np.array(self.scan_data)
        layers = data[:, 0].astype(int)
        steps = data[:, 1].astype(int)
        distances = data[:, 2]
        
        theta_steps = int(self.theta_steps_var.get())
        theta = (steps / theta_steps) * 2 * np.pi
        center_distance = float(self.center_distance_var.get())  # cm - distance from lidar to center of turntable
        r = center_distance - distances
        # z_layer_height is in mm, convert to cm for consistency with X, Y coordinates (which are in cm)
        # Note: z_layer_height represents the actual physical height moved per layer
        z_layer_height_mm = float(self.z_layer_height_var.get())
        z_layer_height_cm = z_layer_height_mm / 10.0  # Convert mm to cm
        
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        # Calculate Z: layer number * height per layer
        # Layer 0 = bottom, each layer moves up by z_layer_height
        z = layers * z_layer_height_cm
        
        # Filter out invalid points (distance = 0 means LiDAR error)
        # Skip points with distance = 0 or invalid values
        valid_distance = (distances > 0) & (distances <= 1200)  # Valid distance range
        
        # Calculate filter thresholds from geometry parameters
        disk_radius = float(self.disk_radius_var.get())
        center_distance = float(self.center_distance_var.get())
        
        # Auto-calculate filter ranges:
        # - Min bán kính: 0 (tâm đĩa)
        # - Max bán kính: bán kính đĩa (hoặc lớn hơn một chút để an toàn)
        min_radius = 0
        max_radius = disk_radius * 1.2  # Cho phép vật thể lớn hơn đĩa một chút
        
        # - Min khoảng cách: khoảng cách từ lidar đến điểm gần nhất (cạnh trong đĩa)
        # - Max khoảng cách: khoảng cách từ lidar đến điểm xa nhất (cạnh ngoài + vật thể cao)
        min_distance = max(0, center_distance - disk_radius - 2)  # Điểm gần nhất
        max_distance = center_distance + disk_radius + 50  # Điểm xa nhất (cho phép vật thể cao)
        
        # Apply filter: valid distance AND within radius/distance ranges
        valid = valid_distance & (r >= min_radius) & (r <= max_radius) & (distances >= min_distance) & (distances <= max_distance)
        points = np.column_stack([x[valid], y[valid], z[valid]])
        
        return points
    
    def generate_stl(self, filename):
        """Generate STL file from scan data"""
        from stl_generator import generate_stl_from_points
        
        points = self.get_cartesian_points()
        if points is None or len(points) == 0:
            raise ValueError("No valid points to export")
        
        generate_stl_from_points(points, filename)
    
    def export_k(self):
        """Export scan data to .k file (point cloud format)"""
        if not self.scan_data:
            messagebox.showwarning("Warning", "No scan data to export")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".k",
            filetypes=[("Point Cloud files", "*.k"), ("Text files", "*.txt"), ("All files", "*.*")]
        )
        
        if filename:
            try:
                self.generate_k(filename)
                messagebox.showinfo("Success", f"Point cloud file saved to {filename}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to export .k file: {str(e)}")
    
    def generate_k(self, filename):
        """Generate .k file (HyperMesh Key file format) from scan data"""
        points = self.get_cartesian_points()
        if points is None or len(points) == 0:
            raise ValueError("No valid points to export")
        
        # Get triangles from mesh structure (same logic as visualization)
        triangles = []
        layers_dict = {}  # Initialize outside try block
        if self.scan_data:
            try:
                # Reconstruct layer and step structure from scan_data
                point_indices = {}  # Map (layer, step) -> index in points array
                valid_indices = []  # Map from scan_data index to points index
                
                # First, map scan_data to points
                points_list = []
                for i, data in enumerate(self.scan_data):
                    layer = data[0]
                    step = data[1]
                    distance = data[2]
                    angle = data[3]
                    
                    # Skip invalid points (distance <= 0 or > 1200)
                    if distance <= 0 or distance > 1200:
                        continue
                    
                    # Calculate cartesian coordinates
                    center_distance = float(self.center_distance_var.get())
                    # z_layer_height is in mm, convert to cm for consistency with X, Y coordinates (which are in cm)
                    z_layer_height_mm = float(self.z_layer_height_var.get())
                    z_layer_height_cm = z_layer_height_mm / 10.0  # Convert mm to cm
                    
                    r = distance
                    theta_rad = angle * np.pi / 180.0
                    x = r * np.cos(theta_rad)
                    y = r * np.sin(theta_rad)
                    z = layer * z_layer_height_cm
                    
                    # Apply filters
                    disk_radius = float(self.disk_radius_var.get())
                    min_radius = max(0.0, center_distance - disk_radius - 5.0)
                    max_radius = center_distance + disk_radius + 50.0
                    min_distance = max(0.0, center_distance - disk_radius - 5.0)
                    max_distance = center_distance + disk_radius + 50.0
                    
                    r_point = np.sqrt(x*x + y*y)
                    if r_point < min_radius or r_point > max_radius or distance < min_distance or distance > max_distance:
                        continue
                    
                    points_list.append([x, y, z])
                    point_idx = len(points_list) - 1
                    
                    if layer not in layers_dict:
                        layers_dict[layer] = []
                    layers_dict[layer].append(step)
                    point_indices[(layer, step)] = point_idx
                    valid_indices.append(i)
            except:
                pass  # If structure reconstruction fails, just export nodes
        
        # Generate triangles if we have structured data
        if len(layers_dict) > 1:
            try:
                theta_steps = int(self.theta_steps_var.get())
            except:
                theta_steps = 200
            
            try:
                layers = sorted(layers_dict.keys())
                for layer_idx in range(len(layers) - 1):
                    layer1 = layers[layer_idx]
                    layer2 = layers[layer_idx + 1]
                    
                    steps1 = sorted(layers_dict[layer1])
                    steps2 = sorted(layers_dict[layer2])
                    
                    for i in range(len(steps1) - 1):
                        step1_curr = steps1[i]
                        step1_next = steps1[i + 1]
                        
                        step2_curr = None
                        step2_next = None
                        
                        for step2 in steps2:
                            if step2_curr is None or abs(step2 - step1_curr) < abs(step2_curr - step1_curr):
                                step2_curr = step2
                            if step2_next is None or abs(step2 - step1_next) < abs(step2_next - step1_next):
                                step2_next = step2
                        
                        if (layer1, step1_curr) in point_indices and \
                           (layer1, step1_next) in point_indices and \
                           (layer2, step2_curr) in point_indices and \
                           (layer2, step2_next) in point_indices:
                            
                            idx1 = point_indices[(layer1, step1_curr)]
                            idx2 = point_indices[(layer1, step1_next)]
                            idx3 = point_indices[(layer2, step2_curr)]
                            idx4 = point_indices[(layer2, step2_next)]
                            
                            max_idx = len(points_list) - 1
                            if idx1 <= max_idx and idx2 <= max_idx and idx3 <= max_idx and idx4 <= max_idx:
                                triangles.append([idx1, idx2, idx3])
                                triangles.append([idx2, idx4, idx3])
            except:
                pass
        
        # Use points_list if available, otherwise use points
        if len(points_list) > 0:
            export_points = np.array(points_list)
        else:
            export_points = points
        
        # Write HyperMesh/LS-DYNA .k file format
        with open(filename, 'w') as f:
            # Write header
            f.write("*KEYWORD\n")
            f.write("$# Generated by 3D Scanner\n")
            f.write("$# Total nodes: {}\n".format(len(export_points)))
            f.write("$# Coordinates in cm\n")
            
            # Write nodes section (LS-DYNA format)
            f.write("*NODE\n")
            # Format: node_id, x, y, z (comma-separated, no parentheses)
            for i, point in enumerate(export_points):
                node_id = i + 1  # HyperMesh node IDs start from 1
                f.write("{},{:.6f},{:.6f},{:.6f}\n".format(
                    node_id, point[0], point[1], point[2]))
            
            # Write elements (triangles) if available
            if len(triangles) > 0:
                f.write("*ELEMENT_SHELL\n")
                f.write("$# Generated triangles from scan mesh\n")
                f.write("$# Total elements: {}\n".format(len(triangles)))
                
                # Write elements: element_id, node1, node2, node3, node4 (comma-separated)
                # For shell elements, we need 4 nodes (repeat last node for triangle)
                for i, tri in enumerate(triangles):
                    elem_id = i + 1  # HyperMesh element IDs start from 1
                    # Convert to 1-based indexing for HyperMesh
                    node1 = tri[0] + 1
                    node2 = tri[1] + 1
                    node3 = tri[2] + 1
                    node4 = tri[2] + 1  # Repeat last node for triangle
                    # Format: element_id, node1, node2, node3, node4
                    f.write("{},{},{},{},{}\n".format(
                        elem_id, node1, node2, node3, node4))
            
            f.write("*END\n")
    
    def log_info(self, message):
        """Log message to info text widget"""
        self.info_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
        self.info_text.see(tk.END)

def main():
    root = tk.Tk()
    app = ScannerGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()

