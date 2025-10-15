import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading, time, queue, os, csv, random, math
import serial, serial.tools.list_ports
import numpy as np
import cv2

# --- optional aruco ---
try:
    from cv2 import aruco
    HAS_ARUCO = True
except Exception:
    HAS_ARUCO = False

# --- optional Pillow for preview ---
try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except Exception:
    HAS_PIL = False

REALTIME_RESET      = b'\x18'
REALTIME_FEED_HOLD  = b'!'
REALTIME_CYCLE      = b'~'
REALTIME_STATUS     = b'?'
REALTIME_JOG_CANCEL = b'\x85'

# ===================== Serial/GLBR (X only) =====================
class GlbrClient:
    def __init__(self):
        self.ser = None
        self.rx_thread = None
        self.rx_running = False
        self.log_q = queue.Queue()
        self.state = "DISCONNECTED"
        self.last_status_line = ""

    def open(self, port, baud):
        self.close()
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
        time.sleep(0.2)
        self.ser.write(b"\r\n\r\n"); self.ser.flush()
        time.sleep(0.5)
        while self.ser.in_waiting:
            _ = self.ser.read(self.ser.in_waiting)
        self.send_gcode("G21"); self.send_gcode("G91"); self.send_gcode("G4 P0")
        self.rx_running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        self.state = "CONNECTED"

    def close(self):
        self.rx_running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1)
            self.rx_thread = None
        if self.ser:
            try: self.ser.close()
            except Exception: pass
            self.ser = None
        self.state = "DISCONNECTED"

    def _rx_loop(self):
        buf = b""
        while self.rx_running:
            try:
                if self.ser is None: break
                data = self.ser.read(1024)
                if data:
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        s = line.decode(errors="ignore").strip()
                        if s:
                            self.log_q.put(s)
                            if s.startswith("<") and s.endswith(">"):
                                self.last_status_line = s
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.log_q.put(f"[ERR] RX: {e}")
                time.sleep(0.2)

    def is_open(self): return self.ser is not None and self.ser.is_open

    def send_bytes(self, bts):
        if not self.is_open(): raise RuntimeError("Chưa kết nối serial")
        self.ser.write(bts); self.ser.flush()

    def send_gcode(self, line):
        self.send_bytes((line.strip()+"\n").encode())

    # realtime
    def status(self):      self.send_bytes(REALTIME_STATUS)      if self.is_open() else None
    def feed_hold(self):   self.send_bytes(REALTIME_FEED_HOLD)   if self.is_open() else None
    def cycle_start(self): self.send_bytes(REALTIME_CYCLE)       if self.is_open() else None
    def reset(self):       self.send_bytes(REALTIME_RESET)       if self.is_open() else None
    def jog_cancel(self):  self.send_bytes(REALTIME_JOG_CANCEL)  if self.is_open() else None

    # X axis
    def move_x(self, dx_mm, feed_mm_min): self.send_gcode(f"G1 X{dx_mm:.4f} F{feed_mm_min:.2f}")
    def jog_x_continuous(self, feed_mm_min, big=1_000_000): self.send_gcode(f"$J=G91 G21 X{big:.3f} F{feed_mm_min:.3f}")

    # ---- D12/D13 (LED/LASER) ----
    def _set_spindle_dir(self, want_high: bool, smax: int = 1000):
        self.send_gcode(f"S{smax}")
        if want_high: self.send_gcode("M4")
        else:         self.send_gcode("M3")
        self.send_gcode("M5")

    def laser_on(self, invert_d13: bool, smax: int = 1000):
        self.send_gcode("M8")
        self._set_spindle_dir(want_high=(not invert_d13), smax=smax)

    def laser_off(self, invert_d13: bool, smax: int = 1000):
        self.send_gcode("M9")
        self._set_spindle_dir(want_high=(invert_d13), smax=smax)

# ===================== Helpers =====================
def parse_grbl_status(line: str) -> dict:
    info = {"state":"UNKNOWN","fields":{}}
    if not (line.startswith("<") and line.endswith(">")): return info
    body = line[1:-1]; parts = body.split("|")
    if not parts: return info
    info["state"] = parts[0]
    for p in parts[1:]:
        if ":" in p:
            k,v = p.split(":",1); info["fields"][k]=v
        else:
            info["fields"][p]=True
    return info

# ===================== Camera / Calib worker =====================
class CalibWorker:
    def __init__(self):
        self.cap = None
        self.running = False
        self.frame_size = None
        # pattern params
        self.pattern_type = "Chessboard"
        self.cols = 9
        self.rows = 6
        self.square_mm = 25.0
        # ChArUco params
        self.aruco_dict_name = "DICT_5X5_50"
        self.charuco_square_mm = 25.0
        self.charuco_marker_mm = 18.0
        # collected points
        self.objpoints = []
        self.imgpoints = []
        # HSV default
        self.hsv_low = np.array([0,120,120], dtype=np.uint8)
        self.hsv_high= np.array([15,255,255], dtype=np.uint8)

    def start(self, cam_index=0, w=1280, h=720):
        self.cap = cv2.VideoCapture(cam_index)
        if w and h:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if not self.cap.isOpened():
            raise RuntimeError("Không mở được camera")
        self.running = True

    def stop(self):
        self.running = False
        if self.cap:
            try: self.cap.release()
            except Exception: pass
            self.cap = None

    def read(self):
        if not self.cap: return None
        ok, frame = self.cap.read()
        if not ok: return None
        if self.frame_size is None:
            self.frame_size = (frame.shape[1], frame.shape[0])
        return frame

    def _build_objp_grid(self):
        objp = np.zeros((self.cols*self.rows, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        objp *= self.square_mm
        return objp

    def detect_pattern(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        overlay = frame.copy()
        objp = None
        info = {}
        if self.pattern_type == "Chessboard":
            flags = (cv2.CALIB_CB_ADAPTIVE_THRESH +
                     cv2.CALIB_CB_FAST_CHECK +
                     cv2.CALIB_CB_NORMALIZE_IMAGE)
            ok, corners = cv2.findChessboardCorners(gray, (self.cols, self.rows), flags)
            if ok:
                corners2 = cv2.cornerSubPix(
                    gray, corners, (11,11), (-1,-1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                )
                cv2.drawChessboardCorners(overlay, (self.cols, self.rows), corners2, ok)
                objp = self._build_objp_grid()
                return True, corners2, overlay, objp, info
            return False, None, overlay, None, info
        elif self.pattern_type in ("Circles","Asym Circles"):
            pattern = (self.cols, self.rows)
            flags = cv2.CALIB_CB_SYMMETRIC_GRID
            if self.pattern_type == "Asym Circles":
                flags = cv2.CALIB_CB_ASYMMETRIC_GRID
            ok, centers = cv2.findCirclesGrid(gray, pattern, flags=flags)
            if ok:
                cv2.drawChessboardCorners(overlay, pattern, centers, ok)
                objp = self._build_objp_grid()
                return True, centers, overlay, objp, info
            return False, None, overlay, None, info
        elif self.pattern_type == "Charuco":
            if not HAS_ARUCO:
                info["error"] = "Cần opencv-contrib-python cho ChArUco"
                return False, None, overlay, None, info
            dict_map = {k:getattr(aruco,k) for k in dir(aruco) if k.startswith("DICT_")}
            dict_id = dict_map.get(self.aruco_dict_name, aruco.DICT_5X5_50)
            ad = aruco.getPredefinedDictionary(dict_id)
            cb = aruco.CharucoBoard_create(self.cols, self.rows,
                                           self.charuco_square_mm, self.charuco_marker_mm, ad)
            corners, ids, _ = aruco.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), ad)
            overlay = aruco.drawDetectedMarkers(overlay, corners, ids) if corners is not None else overlay
            if ids is None or len(ids)==0:
                return False, None, overlay, None, info
            ret, ch_corners, ch_ids = aruco.interpolateCornersCharuco(corners, ids, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), cb)
            if ret and ch_corners is not None and ch_ids is not None and len(ch_corners)>=4:
                aruco.drawDetectedCornersCharuco(overlay, ch_corners, ch_ids)
                objp = cb.chessboardCorners
                info["charuco_ids"] = ch_ids
                return True, ch_corners, overlay, objp, info
            return False, None, overlay, None, info
        else:
            return False, None, overlay, None, {"error":"Pattern không hỗ trợ"}

    def add_frame(self, frame):
        ok, corners, overlay, objp, info = self.detect_pattern(frame)
        if not ok: return False, overlay, info
        if corners.ndim==2: corners = corners.reshape(-1,1,2)
        self.imgpoints.append(corners.astype(np.float32))
        if objp is not None and len(objp)>=len(corners):
            self.objpoints.append(objp[:len(corners)].astype(np.float32))
        else:
            self.objpoints.append(self._build_objp_grid().astype(np.float32))
        return True, overlay, info

    def clear_frames(self): self.objpoints.clear(); self.imgpoints.clear()

    def calibrate(self):
        if len(self.objpoints) < 5:
            raise RuntimeError("Cần ≥5 khung hợp lệ để calib")
        if self.frame_size is None:
            raise RuntimeError("Chưa có kích thước khung hình")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, self.frame_size, None, None
        )
        if not ret: raise RuntimeError("Calibrate không hội tụ")
        return mtx, dist, rvecs, tvecs

    def save_npz(self, path, mtx, dist, rvecs, tvecs):
        np.savez(path,
                 camera_matrix=mtx, dist_coeffs=dist,
                 rvecs=rvecs, tvecs=tvecs,
                 frame_width=self.frame_size[0], frame_height=self.frame_size[1],
                 pattern_type=self.pattern_type,
                 cols=self.cols, rows=self.rows, square_mm=self.square_mm,
                 aruco_dict=self.aruco_dict_name,
                 charuco_square_mm=self.charuco_square_mm,
                 charuco_marker_mm=self.charuco_marker_mm,
                 hsv_low=self.hsv_low, hsv_high=self.hsv_high)
        return path

# ============ RANSAC (thuần Python) ============
def ransac_line_py(points_xy, iters=300, tol=1.5, min_inliers=30):
    pts = list(points_xy)
    n = len(pts)
    if n < 2:
        return False, 0.0, 0.0, [False]*n
    best_a = 0.0; best_b = 0.0; best_mask = [False]*n; best_cnt = 0
    for _ in range(iters):
        i, j = random.sample(range(n), 2)
        x1, y1 = pts[i]; x2, y2 = pts[j]
        if abs(x2 - x1) < 1e-6: continue
        a = (y2 - y1) / (x2 - x1); b = y1 - a * x1
        denom = math.sqrt(a*a + 1.0)
        mask = []; cnt = 0
        for (x, y) in pts:
            d = abs(y - (a*x + b)) / denom
            is_in = d < tol
            mask.append(is_in); cnt += 1 if is_in else 0
        if cnt > best_cnt:
            best_cnt = cnt; best_a, best_b, best_mask = a, b, mask
    if best_cnt < max(min_inliers, 2):
        return False, 0.0, 0.0, [False]*n
    xs, ys = [], []
    for k, (x, y) in enumerate(pts):
        if best_mask[k]: xs.append(float(x)); ys.append(float(y))
    m = len(xs)
    if m >= 2:
        sx = sum(xs); sy = sum(ys)
        sxx = sum(x*x for x in xs); sxy = sum(x*y for (x,y) in zip(xs,ys))
        det = (m*sxx - sx*sx)
        if abs(det) > 1e-12:
            a_ref = (m*sxy - sx*sy) / det
            b_ref = (sy - a_ref*sx) / m
            best_a, best_b = a_ref, b_ref
    return True, float(best_a), float(best_b), best_mask

# ===================== GUI =====================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("3D Scanner")
        self.geometry("1180x840")
        self.resizable(False, False)
        self.client = GlbrClient()
        self.cam = CalibWorker()
        self.status_poll_interval_ms = 250

        # scan state
        self.scan_running = False
        self.scan_thread = None
        self.scan_data = []  # list of dict

        # quay monitor
        self.spin_prev_gray = None
        self.spin_prev_pts = None
        self.spin_prev_ts = None
        self.spin_ema_deg_per_s = 0.0
        self.spin_total_angle = 0.0

        # auto calib state
        self.auto_running = False
        self.auto_thread = None

        # mesh image (dải 2D) và mô phỏng bàn xoay (top-view)
        self.mesh_img = None
        self.mesh_h = 400
        self.mesh_w = 0
        self.sim_img = None  # ảnh mô phỏng top-view
        self.table_radius_mm = 60.0   # default

        # tham số tái tạo thô
        self.recon_cx = None
        self.recon_y0 = None
        self.recon_mm_per_px_x = 0.10
        self.recon_mm_per_px_y = 0.10
        self.use_laser2_also = True

        # 3D viewer state
        self.var_view_3d = None
        self.cnv_right = None
        self._v3d_pts = []
        self._v3d_yaw = 0.0
        self._v3d_pitch = 0.0
        self._v3d_zoom = 1.0
        self._v3d_last = None
        self._sim_tk = None
        self.tk_prev = None
        self.tk_prev_scan = None

        self._build_ui()
        self._refresh_ports()
        self.after(100, self._tick)

    def _sanitize_feed(self, f): return max(1.0, float(f))

    def _build_ui(self):
        nb = ttk.Notebook(self); nb.place(x=10, y=10, width=1160, height=820)

        # ===== Tab 1: Điều khiển =====
        ctrl = ttk.Frame(nb); nb.add(ctrl, text="Điều khiển X + Laser")

        conn = ttk.LabelFrame(ctrl, text="Kết nối"); conn.place(x=10, y=10, width=1130, height=100)
        ttk.Label(conn, text="Port:").place(x=10, y=10)
        self.cmb_port = ttk.Combobox(conn, width=28, state="readonly"); self.cmb_port.place(x=60, y=8)
        ttk.Button(conn, text="Quét cổng", command=self._refresh_ports).place(x=280, y=7, width=90)
        ttk.Label(conn, text="Baud:").place(x=390, y=10)
        self.cmb_baud = ttk.Combobox(conn, values=["115200","250000","57600","9600"], width=12, state="readonly")
        self.cmb_baud.current(0); self.cmb_baud.place(x=440, y=8)
        ttk.Button(conn, text="Kết nối", command=self._on_connect).place(x=590, y=7, width=100)
        ttk.Button(conn, text="Ngắt", command=self._on_disconnect).place(x=700, y=7, width=80)
        ttk.Button(conn, text="Reset (Ctrl-X)", command=self._on_reset).place(x=790, y=7, width=150)
        self.lbl_state = ttk.Label(conn, text="State: DISCONNECTED", font=("Segoe UI",10,"bold"))
        self.lbl_state.place(x=10, y=40)

        xf = ttk.LabelFrame(ctrl, text="X + tốc độ (quét 3D)"); xf.place(x=10, y=120, width=1130, height=170)
        ttk.Label(xf, text="Feed (mm/min):").place(x=10, y=10)
        self.ent_feed = ttk.Entry(xf, width=10); self.ent_feed.insert(0,"5"); self.ent_feed.place(x=120, y=8)
        ttk.Button(xf, text="Tốc độ quét chậm (5)", command=lambda: (self.ent_feed.delete(0,'end'), self.ent_feed.insert(0,"5"))).place(x=200, y=7, width=170)
        ttk.Label(xf, text="Khoảng dịch (mm):").place(x=390, y=10)
        self.ent_dx = ttk.Entry(xf, width=10); self.ent_dx.insert(0,"10"); self.ent_dx.place(x=510,y=8)
        self.btn_jog_start = ttk.Button(xf, text="Jog X liên tục", command=self._on_jog_start); self.btn_jog_start.place(x=10, y=50, width=140)
        self.btn_jog_stop  = ttk.Button(xf, text="Dừng Jog", command=self._on_jog_stop); self.btn_jog_stop.place(x=160, y=50, width=120)
        self.btn_x_plus    = ttk.Button(xf, text="X +Khoảng", command=lambda: self._on_move(+1)); self.btn_x_plus.place(x=290, y=50, width=120)
        self.btn_x_minus   = ttk.Button(xf, text="X -Khoảng", command=lambda: self._on_move(-1)); self.btn_x_minus.place(x=420, y=50, width=120)
        self.btn_hold      = ttk.Button(xf, text="Feed Hold (!)", command=self._on_hold); self.btn_hold.place(x=550, y=50, width=120)
        self.btn_resume    = ttk.Button(xf, text="Resume (~)", command=self._on_resume); self.btn_resume.place(x=680, y=50, width=120)
        ttk.Button(xf, text="Hỏi trạng thái (?)", command=self._on_status_once).place(x=810, y=50, width=120)

        laserf = ttk.LabelFrame(ctrl, text="Laser/LED (D12 + D13)"); laserf.place(x=10, y=300, width=1130, height=110)
        ttk.Button(laserf, text="BẬT LASER (M8 + D13)", command=self._on_laser_on).place(x=20, y=20, width=200)
        ttk.Button(laserf, text="TẮT LASER (M9 + D13)", command=self._on_laser_off).place(x=230, y=20, width=200)
        ttk.Label(laserf, text="• D12 = M8/M9   • D13 = M3/M4 (DIR) — cần S>0 để cập nhật").place(x=460, y=24)
        self.var_inv_d13 = tk.BooleanVar(value=True)
        ttk.Checkbutton(laserf, text="Invert D13 (LED active-LOW)", variable=self.var_inv_d13).place(x=20, y=65)

        logf = ttk.LabelFrame(ctrl, text="Log / Console"); logf.place(x=10, y=420, width=1130, height=300)
        self.txt = tk.Text(logf, height=14, wrap="none"); self.txt.place(x=10, y=10, width=1100, height=250)
        yscroll = ttk.Scrollbar(logf, orient="vertical", command=self.txt.yview); yscroll.place(x=1115, y=10, height=250)
        self.txt.configure(yscrollcommand=yscroll.set)

        # ===== Tab 2: Calib =====
        calib = ttk.Frame(nb); nb.add(calib, text="Calib (Pattern → .npz)")
        cam_ctrl = ttk.LabelFrame(calib, text="Camera"); cam_ctrl.place(x=10, y=10, width=1130, height=80)
        ttk.Label(cam_ctrl, text="Index:").place(x=10, y=10)
        self.ent_cam_idx = ttk.Entry(cam_ctrl, width=5); self.ent_cam_idx.insert(0,"0"); self.ent_cam_idx.place(x=60,y=8)
        ttk.Button(cam_ctrl, text="Start", command=self._cam_start).place(x=120, y=7, width=80)
        ttk.Button(cam_ctrl, text="Stop",  command=self._cam_stop ).place(x=210, y=7, width=80)

        patf = ttk.LabelFrame(calib, text="Pattern (giống Horus: set số ô)"); patf.place(x=10, y=100, width=700, height=140)
        ttk.Label(patf, text="Loại:").place(x=10, y=10)
        self.cmb_pat = ttk.Combobox(patf, values=["Chessboard","Circles","Asym Circles","Charuco"], state="readonly", width=18)
        self.cmb_pat.current(0); self.cmb_pat.place(x=60, y=8)
        ttk.Label(patf, text="Cols x Rows:").place(x=10, y=45)
        self.ent_cols = ttk.Entry(patf, width=5); self.ent_cols.insert(0,"9");  self.ent_cols.place(x=100, y=43)
        self.ent_rows = ttk.Entry(patf, width=5); self.ent_rows.insert(0,"6");  self.ent_rows.place(x=140, y=43)
        ttk.Label(patf, text="Ô (mm):").place(x=190, y=45)
        self.ent_sq = ttk.Entry(patf, width=7); self.ent_sq.insert(0,"25.0"); self.ent_sq.place(x=240, y=43)

        ttk.Label(patf, text="Aruco dict:").place(x=10, y=80)
        self.cmb_dict = ttk.Combobox(patf, values=[
            "DICT_4X4_50","DICT_5X5_50","DICT_6X6_50","DICT_ARUCO_ORIGINAL"
        ], state="readonly", width=18)
        self.cmb_dict.current(1); self.cmb_dict.place(x=90, y=78)
        ttk.Label(patf, text="Charuco square/mm:").place(x=260, y=80)
        self.ent_ch_sq = ttk.Entry(patf, width=7); self.ent_ch_sq.insert(0,"25.0"); self.ent_ch_sq.place(x=400, y=78)
        ttk.Label(patf, text="marker/mm:").place(x=460, y=80)
        self.ent_ch_mk = ttk.Entry(patf, width=7); self.ent_ch_mk.insert(0,"18.0"); self.ent_ch_mk.place(x=530, y=78)

        actf = ttk.LabelFrame(calib, text="Thu khung & Calib"); actf.place(x=720, y=100, width=420, height=140)
        ttk.Button(actf, text="Nhận diện + Thêm khung (manual)", command=self._calib_add).place(x=10, y=10, width=220)
        ttk.Button(actf, text="Xoá danh sách khung", command=self._calib_clear).place(x=240, y=10, width=160)
        ttk.Button(actf, text="Calibrate & Lưu .npz", command=self._calib_save).place(x=10, y=50, width=390)
        self.lbl_frames = ttk.Label(actf, text="Frames: 0"); self.lbl_frames.place(x=10, y=90)

        autof = ttk.LabelFrame(calib, text="Auto quay & thu khung (bước trục X như Horus)"); autof.place(x=10, y=250, width=1130, height=100)
        ttk.Label(autof, text="Số khung cần (OK):").place(x=10, y=10)
        self.ent_auto_frames = ttk.Entry(autof, width=7); self.ent_auto_frames.insert(0,"24"); self.ent_auto_frames.place(x=140, y=8)
        ttk.Label(autof, text="Bước X mỗi lần (mm):").place(x=210, y=10)
        self.ent_auto_dx = ttk.Entry(autof, width=8); self.ent_auto_dx.insert(0,"2.0"); self.ent_auto_dx.place(x=350, y=8)
        ttk.Label(autof, text="Feed (mm/min):").place(x=430, y=10)
        self.ent_auto_feed = ttk.Entry(autof, width=8); self.ent_auto_feed.insert(0,"5"); self.ent_auto_feed.place(x=530, y=8)
        ttk.Label(autof, text="Delay sau bước (ms):").place(x=610, y=10)
        self.ent_auto_delay = ttk.Entry(autof, width=8); self.ent_auto_delay.insert(0,"250"); self.ent_auto_delay.place(x=760, y=8)
        self.btn_auto_start = ttk.Button(autof, text="Bắt đầu AUTO", command=self._auto_start); self.btn_auto_start.place(x=860, y=8, width=120)
        self.btn_auto_stop  = ttk.Button(autof, text="Dừng AUTO", command=self._auto_stop); self.btn_auto_stop.place(x=990, y=8, width=120)

        prevf = ttk.LabelFrame(calib, text="Preview + Overlay pattern"); prevf.place(x=10, y=360, width=1130, height=420)
        self.lbl_prev = ttk.Label(prevf, text="(Preview ở đây)"); self.lbl_prev.place(x=10, y=10, width=1100, height=380)

        # ===== Tab 3: Scan 3D =====
        scan = ttk.Frame(nb); nb.add(scan, text="Scan 3D (2 Laser + RANSAC)")
        cam2 = ttk.LabelFrame(scan, text="Camera"); cam2.place(x=10, y=10, width=1130, height=100)
        ttk.Button(cam2, text="Start cam", command=self._cam_start).place(x=10, y=10, width=100)
        ttk.Button(cam2, text="Stop cam",  command=self._cam_stop).place(x=120, y=10, width=100)
        self.lbl_spin = ttk.Label(cam2, text="Bàn quay: (chưa đo)", font=("Segoe UI",9,"bold"))
        self.lbl_spin.place(x=240, y=12)
        self.var_auto_angle = tk.BooleanVar(value=False)
        ttk.Checkbutton(cam2, text="Ước lượng góc tự động", variable=self.var_auto_angle).place(x=240, y=40)

        laserf = ttk.LabelFrame(scan, text="Laser HSV (2 kênh)"); laserf.place(x=10, y=110, width=740, height=135)
        ttk.Label(laserf, text="Laser1 H:[").place(x=10, y=10)
        self.l1_hl = tk.IntVar(value=0);   self.l1_hh = tk.IntVar(value=15)
        self.l1_sl = tk.IntVar(value=120); self.l1_sh = tk.IntVar(value=255)
        self.l1_vl = tk.IntVar(value=120); self.l1_vh = tk.IntVar(value=255)
        ttk.Entry(laserf, width=4, textvariable=self.l1_hl).place(x=70,y=8); ttk.Label(laserf, text=",").place(x=100,y=10)
        ttk.Entry(laserf, width=4, textvariable=self.l1_hh).place(x=110,y=8); ttk.Label(laserf, text="]  S:[").place(x=145,y=10)
        ttk.Entry(laserf, width=4, textvariable=self.l1_sl).place(x=185,y=8); ttk.Label(laserf, text=",").place(x=215,y=10)
        ttk.Entry(laserf, width=4, textvariable=self.l1_sh).place(x=225,y=8); ttk.Label(laserf, text="]  V:[").place(x=260,y=10)
        ttk.Entry(laserf, width=4, textvariable=self.l1_vl).place(x=300,y=8); ttk.Label(laserf, text=",").place(x=330,y=10)
        ttk.Entry(laserf, width=4, textvariable=self.l1_vh).place(x=340,y=8); ttk.Label(laserf, text="]").place(x=370,y=10)

        ttk.Label(laserf, text="Laser2 H:[").place(x=10, y=40)
        self.l2_hl = tk.IntVar(value=160); self.l2_hh = tk.IntVar(value=179)
        self.l2_sl = tk.IntVar(value=120); self.l2_sh = tk.IntVar(value=255)
        self.l2_vl = tk.IntVar(value=120); self.l2_vh = tk.IntVar(value=255)
        ttk.Entry(laserf, width=4, textvariable=self.l2_hl).place(x=70,y=38); ttk.Label(laserf, text=",").place(x=100,y=40)
        ttk.Entry(laserf, width=4, textvariable=self.l2_hh).place(x=110,y=38); ttk.Label(laserf, text="]  S:[").place(x=145,y=40)
        ttk.Entry(laserf, width=4, textvariable=self.l2_sl).place(x=185,y=40); ttk.Label(laserf, text=",").place(x=215,y=40)
        ttk.Entry(laserf, width=4, textvariable=self.l2_sh).place(x=225,y=40); ttk.Label(laserf, text="]  V:[").place(x=260,y=40)
        ttk.Entry(laserf, width=4, textvariable=self.l2_vl).place(x=300,y=40); ttk.Label(laserf, text=",").place(x=330,y=40)
        ttk.Entry(laserf, width=4, textvariable=self.l2_vh).place(x=340,y=40); ttk.Label(laserf, text="]").place(x=370,y=40)

        scctl = ttk.LabelFrame(scan, text="Điều khiển quét"); scctl.place(x=760, y=110, width=380, height=175)
        ttk.Label(scctl, text="Feed quét (mm/min):").place(x=10,y=10)
        self.ent_scan_feed = ttk.Entry(scctl, width=8); self.ent_scan_feed.insert(0,"5"); self.ent_scan_feed.place(x=150,y=8)
        ttk.Label(scctl, text="Bước X (mm/khung):").place(x=10,y=40)
        self.ent_scan_dx = ttk.Entry(scctl, width=8); self.ent_scan_dx.insert(0,"0.5"); self.ent_scan_dx.place(x=150,y=38)
        ttk.Label(scctl, text="Số khung:").place(x=10,y=70)
        self.ent_scan_frames = ttk.Entry(scctl, width=8); self.ent_scan_frames.insert(0,"400"); self.ent_scan_frames.place(x=150,y=68)
        self.var_turntable = tk.BooleanVar(value=True)
        ttk.Checkbutton(scctl, text="Đế quay Horus (không dùng X)", variable=self.var_turntable).place(x=10, y=100)
        ttk.Label(scctl, text="Góc/khung (deg):").place(x=200, y=100)
        self.ent_deg_per_frame = ttk.Entry(scctl, width=8); self.ent_deg_per_frame.insert(0,"1.0"); self.ent_deg_per_frame.place(x=290, y=98)
        ttk.Button(scctl, text="Bắt đầu quét", command=self._scan_start).place(x=240,y=8, width=120)
        ttk.Button(scctl, text="Dừng quét",   command=self._scan_stop ).place(x=240,y=40, width=120)

        # Tái tạo thô (tham số sang 3D & mô phỏng)
        reconf = ttk.LabelFrame(scan, text="Tái tạo thô (top-view & xuất 3D)"); reconf.place(x=760, y=290, width=380, height=80)
        ttk.Label(reconf, text="Bán kính bàn (mm):").place(x=10, y=10)
        self.ent_table_r = ttk.Entry(reconf, width=7); self.ent_table_r.insert(0, f"{self.table_radius_mm:g}")
        self.ent_table_r.place(x=130,y=8)
        ttk.Label(reconf, text="cx (px):").place(x=190, y=10)
        self.ent_cx = ttk.Entry(reconf, width=7); self.ent_cx.insert(0, "")
        self.ent_cx.place(x=240,y=8)
        ttk.Label(reconf, text="mm/px X,Y:").place(x=10, y=40)
        self.ent_mmpx_x = ttk.Entry(reconf, width=7); self.ent_mmpx_x.insert(0, f"{self.recon_mm_per_px_x:g}")
        self.ent_mmpx_x.place(x=90, y=38)
        self.ent_mmpx_y = ttk.Entry(reconf, width=7); self.ent_mmpx_y.insert(0, f"{self.recon_mm_per_px_y:g}")
        self.ent_mmpx_y.place(x=140, y=38)
        ttk.Label(reconf, text="y0 (px):").place(x=190, y=40)
        self.ent_y0 = ttk.Entry(reconf, width=7); self.ent_y0.insert(0, "")
        self.ent_y0.place(x=240, y=38)

        exportf = ttk.LabelFrame(scan, text="Xuất dữ liệu quét"); exportf.place(x=10, y=290, width=740, height=80)
        ttk.Button(exportf, text="Lưu .npz", command=self._scan_save_npz).place(x=20, y=15, width=120)
        ttk.Button(exportf, text="Lưu .csv", command=self._scan_save_csv).place(x=150, y=15, width=120)
        ttk.Button(exportf, text="Xuất .PLY/.OBJ", command=self._export_ply_obj).place(x=280, y=15, width=140)
        self.lbl_scan_info = ttk.Label(exportf, text="Frames quét: 0  |  Tổng điểm: 0")
        self.lbl_scan_info.place(x=440, y=18)

        # Preview + Mô phỏng/3D
        prev3d = ttk.LabelFrame(scan, text="Preview camera  |  Top-view / 3D viewer (xoay bằng chuột)")
        prev3d.place(x=10, y=380, width=1130, height=400)
        self.lbl_prev_scan = ttk.Label(prev3d, text="(Preview quét ở đây)")
        self.lbl_prev_scan.place(x=10, y=10, width=540, height=360)

        # vùng phải: Canvas + công tắc 3D
        self.var_view_3d = tk.BooleanVar(value=False)
        ttk.Checkbutton(prev3d, text="Bật 3D viewer (drag để xoay, cuộn để zoom)",
                        variable=self.var_view_3d, command=self._refresh_right_view).place(x=580, y=10)
        self.cnv_right = tk.Canvas(prev3d, width=540, height=330, bg="#111111", highlightthickness=0)
        self.cnv_right.place(x=580, y=40)

        # bind chuột cho 3D viewer
        self.cnv_right.bind("<Button-1>", self._v3d_on_down)
        self.cnv_right.bind("<B1-Motion>", self._v3d_on_drag)
        self.cnv_right.bind("<ButtonRelease-1>", self._v3d_on_up)
        self.cnv_right.bind("<MouseWheel>", self._v3d_on_wheel)      # Win/Mac
        self.cnv_right.bind("<Button-4>", self._v3d_on_wheel_linux)  # Linux up
        self.cnv_right.bind("<Button-5>", self._v3d_on_wheel_linux)  # Linux down

    # ---------- Control Tab handlers ----------
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cmb_port["values"] = ports
        if ports: self.cmb_port.current(0)

    def _on_connect(self):
        port = self.cmb_port.get().strip()
        baud = int(self.cmb_baud.get())
        if not port:
            messagebox.showwarning("Thiếu thông tin","Hãy chọn cổng COM")
            return
        try:
            self.client.open(port, baud)
            self._log(f"[INFO] Connected {port} @ {baud}")
        except Exception as e:
            messagebox.showerror("Kết nối", str(e))

    def _on_disconnect(self):
        self.client.close(); self._log("[INFO] Disconnected")

    def _on_reset(self):
        try: self.client.reset(); self._log("[REALTIME] Ctrl-X")
        except Exception as e: messagebox.showerror("Lỗi", str(e))

    def _on_hold(self):
        try: self.client.feed_hold(); self._log("[REALTIME] ! Hold")
        except Exception as e: messagebox.showerror("Lỗi", str(e))

    def _on_resume(self):
        try: self.client.cycle_start(); self._log("[REALTIME] ~ Resume")
        except Exception as e: messagebox.showerror("Lỗi", str(e))

    def _on_status_once(self):
        try: self.client.status()
        except Exception as e: messagebox.showerror("Lỗi", str(e))

    def _on_jog_start(self):
        try:
            feed = self._sanitize_feed(self.ent_feed.get())
            self.ent_feed.delete(0,'end'); self.ent_feed.insert(0, f"{feed:g}")
            self.client.jog_x_continuous(feed_mm_min=feed)
            self._log(f"[$J] Jog X @ F={feed} mm/min")
        except Exception as e:
            messagebox.showerror("Lỗi", str(e))

    def _on_jog_stop(self):
        try:
            self.client.jog_cancel(); self._log("[JOG] Cancel (0x85)")
            self.client.status()
        except Exception as e:
            messagebox.showerror("Lỗi", str(e))

    def _on_move(self, sign):
        try:
            feed = self._sanitize_feed(self.ent_feed.get())
            dx = float(self.ent_dx.get()) * sign
            self.ent_feed.delete(0,'end'); self.ent_feed.insert(0, f"{feed:g}")
            self.client.move_x(dx_mm=dx, feed_mm_min=feed)
            self._log(f"[G1] X{dx} @ F={feed}")
        except Exception as e:
            messagebox.showerror("Lỗi", str(e))

    def _on_laser_on(self):
        try:
            inv = bool(self.var_inv_d13.get())
            self.client.laser_on(invert_d13=inv, smax=1000)
            self._log(f"[LASER] ON -> D12=M8; D13={'LOW' if inv else 'HIGH'} (invert={inv})")
        except Exception as e:
            messagebox.showerror("Lỗi", str(e))

    def _on_laser_off(self):
        try:
            inv = bool(self.var_inv_d13.get())
            self.client.laser_off(invert_d13=inv, smax=1000)
            self._log(f"[LASER] OFF -> D12=M9; D13={'HIGH' if inv else 'LOW'} (invert={inv})")
        except Exception as e:
            messagebox.showerror("Lỗi", str(e))

    # ---------- Calib Tab handlers ----------
    def _cam_start(self):
        try:
            idx = int(self.ent_cam_idx.get())
            self.cam.start(idx)
            # reset quay monitor
            self.spin_prev_gray = None
            self.spin_prev_pts = None
            self.spin_prev_ts = None
            self.spin_ema_deg_per_s = 0.0
            self.spin_total_angle = 0.0
            self._log(f"[CAM] Start index {idx}")
        except Exception as e:
            messagebox.showerror("Camera", str(e))

    def _cam_stop(self):
        self.cam.stop(); self._log("[CAM] Stopped")

    def _calib_add(self):
        if not self.cam.running:
            messagebox.showwarning("Camera","Hãy Start camera trước"); return
        self._update_pattern_params()
        frame = self.cam.read()
        if frame is None:
            messagebox.showwarning("Camera", "Không có frame"); return
        ok, overlay, info = self.cam.add_frame(frame)
        if ok:
            self._log("[CALIB] Thêm khung OK")
            self.lbl_frames.config(text=f"Frames: {len(self.cam.objpoints)}")
        else:
            if "error" in info: self._log(f"[CALIB] Lỗi: {info['error']}")
            else: self._log("[CALIB] Không nhận diện được pattern")
        self._push_preview(overlay if overlay is not None else frame)

    def _calib_clear(self):
        self.cam.clear_frames()
        self.lbl_frames.config(text="Frames: 0")
        self._log("[CALIB] Đã xoá danh sách khung")

    def _calib_save(self):
        try:
            self._update_pattern_params()
            mtx, dist, rvecs, tvecs = self.cam.calibrate()
            path = filedialog.asksaveasfilename(defaultextension=".npz",
                                                filetypes=[("NumPy config","*.npz")],
                                                initialfile="camera_config.npz")
            if not path: return
            self.cam.save_npz(path, mtx, dist, rvecs, tvecs)
            self._log(f"[CALIB] Lưu config: {path}")
            messagebox.showinfo("Calibration", f"Đã lưu:\n{path}")
        except Exception as e:
            messagebox.showerror("Calibration", str(e))

    def _update_pattern_params(self):
        self.cam.pattern_type = self.cmb_pat.get()
        try:
            self.cam.cols = int(self.ent_cols.get())
            self.cam.rows = int(self.ent_rows.get())
            self.cam.square_mm = float(self.ent_sq.get())
            self.cam.aruco_dict_name = self.cmb_dict.get()
            self.cam.charuco_square_mm = float(self.ent_ch_sq.get())
            self.cam.charuco_marker_mm = float(self.ent_ch_mk.get())
        except: pass

    # ----- AUTO collect like Horus -----
    def _auto_start(self):
        if self.auto_running: return
        if not (self.client.is_open() and self.cam.running):
            messagebox.showwarning("AUTO", "Cần KẾT NỐI GLBR và START camera trước."); return
        try:
            self.auto_target = int(self.ent_auto_frames.get())
            self.auto_dx = float(self.ent_auto_dx.get())
            self.auto_feed = self._sanitize_feed(self.ent_auto_feed.get())
            self.auto_delay_ms = int(self.ent_auto_delay.get())
        except:
            messagebox.showwarning("AUTO", "Thông số AUTO không hợp lệ."); return
        self.auto_running = True
        self.auto_thread = threading.Thread(target=self._auto_worker, daemon=True)
        self.auto_thread.start()
        self._log("[AUTO] Bắt đầu thu khung…")

    def _auto_stop(self):
        self.auto_running = False
        self._log("[AUTO] Yêu cầu dừng.")

    def _auto_worker(self):
        ok_frames = 0; attempts = 0; retry_sleep = 0.25
        while self.auto_running and ok_frames < self.auto_target:
            try: self.client.move_x(self.auto_dx, self.auto_feed)
            except Exception as e:
                attempts += 1
                self._log(f"[AUTO] Lỗi move X (attempt {attempts}): {e}. Sẽ thử lại…")
                time.sleep(retry_sleep); continue
            time.sleep(max(self.auto_delay_ms, 0) / 1000.0)
            frame = self.cam.read()
            if frame is None:
                self._log("[AUTO] Không có frame camera, bỏ qua lần này."); continue
            self._update_pattern_params()
            ok, overlay, info = self.cam.add_frame(frame)
            if ok:
                ok_frames += 1
                self._log(f"[AUTO] +1 khung OK ({ok_frames}/{self.auto_target})")
                self.lbl_frames.after(0, lambda v=ok_frames: self.lbl_frames.config(text=f"Frames: {v}"))
                self._push_preview(overlay)
            else:
                if "error" in info: self._log(f"[AUTO] Lỗi nhận diện: {info['error']}")
                else: self._log("[AUTO] Không nhận diện được pattern, tiếp tục bước tiếp theo.")
        self.auto_running = False
        self._log("[AUTO] Hoàn thành." if ok_frames >= self.auto_target else "[AUTO] Đã dừng.")

    # ---------- Scan 3D handlers ----------
    def _scan_start(self):
        if self.scan_running: return
        if not self.cam.running:
            messagebox.showwarning("Scan", "Cần START camera trước."); return
        if (not self.client.is_open()) and (not self.var_turntable.get()):
            messagebox.showwarning("Scan", "Cần KẾT NỐI GLBR nếu không dùng chế độ quay đế."); return
        try:
            self.scan_feed = self._sanitize_feed(self.ent_scan_feed.get())
            self.scan_dx = float(self.ent_scan_dx.get())
            self.scan_frames_target = int(self.ent_scan_frames.get())
            self.scan_turntable = bool(self.var_turntable.get())
            self.scan_deg_per_frame = float(self.ent_deg_per_frame.get())
        except:
            messagebox.showwarning("Scan", "Thông số quét không hợp lệ."); return

        self.scan_data = []
        # reset mesh (dải ảnh)
        h_hint = (self.cam.frame_size[1] if self.cam.frame_size else 720)
        self._mesh_reset(total_steps=self.scan_frames_target, frame_h_hint=h_hint)
        # reset mô phỏng
        self._sim_reset()
        # tự điền cx & y0 nếu trống, khi có frame đầu tiên
        self.recon_cx = None
        self.recon_y0 = None
        if self.var_auto_angle.get(): self.spin_total_angle = 0.0
        self.scan_running = True
        self.scan_thread = threading.Thread(target=self._scan_worker, daemon=True)
        self.scan_thread.start()
        self._log(f"[SCAN] Bắt đầu quét 3D – {'QUAY ĐẾ' if self.scan_turntable else 'DỊCH X'}…")

    def _scan_stop(self):
        self.scan_running = False
        self._log("[SCAN] Yêu cầu dừng.")

    def _scan_worker(self):
        acc_x = 0.0
        for i in range(self.scan_frames_target):
            if not self.scan_running: break
            if self.scan_turntable:
                time.sleep(0.05)
                angle_deg = (self.spin_total_angle if self.var_auto_angle.get() else i * self.scan_deg_per_frame)
            else:
                try: self.client.move_x(self.scan_dx, self.scan_feed)
                except Exception as e:
                    self._log(f"[SCAN] Lỗi move X: {e}"); break
                acc_x += self.scan_dx
                time.sleep(0.05)
                angle_deg = None

            frame = self.cam.read()
            if frame is None:
                self._log("[SCAN] Không có frame"); continue

            # khởi tạo tham số tái tạo thô lần đầu
            if self.recon_cx is None:
                fh, fw = frame.shape[:2]
                self.recon_cx = fw/2.0
                self.recon_y0 = float(fh)  # đáy khung
                try:
                    if self.ent_cx.get().strip(): self.recon_cx = float(self.ent_cx.get())
                    if self.ent_y0.get().strip(): self.recon_y0 = float(self.ent_y0.get())
                except: pass

            overlay, l1_pts, l2_pts = self._process_two_lasers(frame)

            # cập nhật dải mesh (2D)
            self._mesh_add_frame(i, l1_pts, l2_pts, frame.shape[:2])
            # cập nhật mô phỏng bàn xoay + vật (top-view)
            self._sim_add_points(angle_deg, l1_pts, l2_pts, frame.shape[:2])
            # hiển thị vùng phải theo chế độ
            self._refresh_right_view()

            # lưu bản ghi
            rec = {"step_idx": i, "x_pos": acc_x, "ts": time.time(),
                   "laser1": l1_pts, "laser2": l2_pts}
            if self.scan_turntable: rec["angle_deg"] = float(angle_deg)
            self.scan_data.append(rec)

            # nếu đang bật 3D viewer -> dựng nhanh point cloud tạm
            if self.var_view_3d.get():
                pc_live = self._collect_pointcloud_mm()
                self._v3d_set_points(pc_live)

            # preview + label
            self._push_scan_preview(overlay)
            tot_pts = (0 if l1_pts is None else len(l1_pts)) + (0 if l2_pts is None else len(l2_pts))
            if self.scan_turntable:
                self.lbl_scan_info.after(0, lambda s=i+1, t=tot_pts, a=(angle_deg if angle_deg is not None else float("nan")):
                    self.lbl_scan_info.config(text=f"Frames: {s} | Điểm: {t} | Góc≈{(a if a!=a else a):.1f}°"))
            else:
                self.lbl_scan_info.after(0, lambda s=i+1, t=tot_pts, x=acc_x:
                    self.lbl_scan_info.config(text=f"Frames: {s} | Điểm: {t} | X={x:.3f}mm"))

        self.scan_running = False
        self._log("[SCAN] Kết thúc quét.")
        # cập nhật 3D cuối cùng khi kết thúc
        if self.var_view_3d.get():
            self._v3d_set_points(self._collect_pointcloud_mm())
        # đề nghị xuất ngay
        if self.scan_data and messagebox.askyesno("Xuất điểm 3D", "Xuất .PLY/.OBJ ngay bây giờ?"):
            try: self._export_ply_obj()
            except Exception as e: messagebox.showerror("Xuất 3D", str(e))

    def _process_two_lasers(self, frame_bgr):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        def clamp(v, lo, hi): return max(lo, min(hi, int(v)))
        def mask_from_vars(hl, hh, sl, sh, vl, vh):
            low  = (clamp(hl,0,179), clamp(sl,0,255), clamp(vl,0,255))
            high = (clamp(hh,0,179), clamp(sh,0,255), clamp(vh,0,255))
            return cv2.inRange(hsv, low, high)
        m1 = mask_from_vars(self.l1_hl.get(), self.l1_hh.get(), self.l1_sl.get(), self.l1_sh.get(), self.l1_vl.get(), self.l1_vh.get())
        m2 = mask_from_vars(self.l2_hl.get(), self.l2_hh.get(), self.l2_sl.get(), self.l2_sh.get(), self.l2_vl.get(), self.l2_vh.get())
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        m1 = cv2.morphologyEx(m1, cv2.MORPH_OPEN, k, iterations=1); m1 = cv2.dilate(m1, k, iterations=1)
        m2 = cv2.morphologyEx(m2, cv2.MORPH_OPEN, k, iterations=1); m2 = cv2.dilate(m2, k, iterations=1)
        overlay = frame_bgr.copy()
        def nz_points(mask):
            nz = cv2.findNonZero(mask)
            if nz is None: return []
            return [(int(p[0][0]), int(p[0][1])) for p in nz]
        pts1 = nz_points(m1); pts2 = nz_points(m2)
        out1 = None; out2 = None
        if len(pts1) > 0:
            ok, a, b, imask = ransac_line_py(pts1, iters=250, tol=1.5, min_inliers=30)
            if ok:
                h, w = overlay.shape[:2]; x0, x1 = 0, w-1
                y0 = int(a*x0 + b); y1 = int(a*x1 + b)
                cv2.line(overlay, (x0, y0), (x1, y1), (0,255,0), 1)
                out1 = []; step = max(1, len(pts1)//200)
                for idx in range(0, len(pts1), step):
                    if imask[idx]:
                        x,y = pts1[idx]; out1.append((x,y))
                        cv2.circle(overlay, (x,y), 1, (0,200,0), -1)
        if len(pts2) > 0:
            ok, a, b, imask = ransac_line_py(pts2, iters=250, tol=1.5, min_inliers=30)
            if ok:
                h, w = overlay.shape[:2]; x0, x1 = 0, w-1
                y0 = int(a*x0 + b); y1 = int(a*x1 + b)
                cv2.line(overlay, (x0, y0), (x1, y1), (255,0,0), 1)
                out2 = []; step = max(1, len(pts2)//200)
                for idx in range(0, len(pts2), step):
                    if imask[idx]:
                        x,y = pts2[idx]; out2.append((x,y))
                        cv2.circle(overlay, (x,y), 1, (200,0,0), -1)
        return overlay, out1, out2

    # ========== Mesh dải & Mô phỏng top-view ==========
    def _mesh_reset(self, total_steps: int, frame_h_hint: int):
        self.mesh_w = max(1, int(total_steps))
        self.mesh_h = max(100, int(min(1000, frame_h_hint)))
        self.mesh_img = np.zeros((self.mesh_h, self.mesh_w, 3), dtype=np.uint8)

    def _mesh_add_frame(self, step_idx: int, l1_pts, l2_pts, frame_hw):
        if self.mesh_img is None: return
        Hmesh, Wmesh = self.mesh_img.shape[:2]
        if step_idx < 0 or step_idx >= Wmesh: return
        Hsrc = frame_hw[0]
        def map_y(y): return int(min(Hmesh-1, max(0, y * (Hmesh-1) / max(1, Hsrc-1))))
        if l1_pts:
            for (x,y) in l1_pts:
                self.mesh_img[map_y(y), step_idx] = (0,255,0)
        if l2_pts:
            for (x,y) in l2_pts:
                self.mesh_img[map_y(y), step_idx] = (255,0,0)

    def _push_scan_mesh(self):
        # (không còn dùng label riêng cho dải; vùng phải hiển thị top-view/3D)
        pass

    def _sim_reset(self):
        self.sim_img = None

    def _sim_add_points(self, angle_deg, l1_pts, l2_pts, frame_hw):
        if not HAS_PIL: return
        # tích lũy điểm vào ảnh mô phỏng (top-view XZ)
        W, H = 540, 360
        if self.sim_img is None:
            self.sim_img = np.zeros((H, W, 3), dtype=np.uint8)
        # đọc tham số
        try:
            self.table_radius_mm = float(self.ent_table_r.get() or self.table_radius_mm)
        except: pass
        try:
            self.recon_mm_per_px_x = float(self.ent_mmpx_x.get() or self.recon_mm_per_px_x)
            self.recon_mm_per_px_y = float(self.ent_mmpx_y.get() or self.recon_mm_per_px_y)
        except: pass
        if self.recon_cx is None: return  # chờ frame đầu để có kích thước
        try:
            if self.ent_cx.get().strip(): self.recon_cx = float(self.ent_cx.get())
            if self.ent_y0.get().strip(): self.recon_y0 = float(self.ent_y0.get())
        except: pass
        # scale mm->px trong ảnh mô phỏng
        Rpx = int(min(W, H)//2 - 8)
        mm2px = Rpx / max(1e-6, self.table_radius_mm)
        cx = W//2; cz = H//2
        # vẽ nền: bàn xoay
        self.sim_img[:] = (20,20,20)
        cv2.circle(self.sim_img, (cx, cz), Rpx, (60,60,60), thickness=2)
        cv2.circle(self.sim_img, (cx, cz), 2, (80,80,80), -1)
        def add_list(pts, color_bgr):
            if not pts: return
            if angle_deg is None: return
            th = math.radians(angle_deg)
            c, s = math.cos(th), math.sin(th)
            for (xp, yp) in pts:
                r_mm = (float(xp) - float(self.recon_cx)) * self.recon_mm_per_px_x
                if abs(r_mm) > self.table_radius_mm: continue
                X = r_mm * c
                Z = r_mm * s
                xpix = int(cx + X * mm2px)
                zpix = int(cz + Z * mm2px)
                if 0 <= xpix < W and 0 <= zpix < H:
                    self.sim_img[zpix, xpix] = color_bgr
        add_list(l1_pts, (0,255,0))
        if self.use_laser2_also:
            add_list(l2_pts, (255,0,0))

    def _push_simulation(self):
        # nếu đang 3D thì không vẽ top-view
        if self.var_view_3d.get():
            return
        self.cnv_right.delete("all")
        if self.sim_img is None or not HAS_PIL:
            self.cnv_right.create_text(270,165, fill="#aaaaaa", text="(Cài Pillow để xem mô phỏng)")
            return
        show = cv2.cvtColor(self.sim_img, cv2.COLOR_BGR2RGB)
        im = Image.fromarray(show)
        self._sim_tk = ImageTk.PhotoImage(im)
        self.cnv_right.create_image(0, 0, image=self._sim_tk, anchor="nw")

    # --------- 3D viewer (Canvas) ----------
    def _refresh_right_view(self):
        if self.var_view_3d.get():
            # bật 3D: dựng từ dữ liệu hiện có
            self._v3d_set_points(self._collect_pointcloud_mm())
        else:
            # tắt 3D: về top-view
            self._push_simulation()

    def _v3d_set_points(self, pc_xyz_mm):
        self._v3d_pts = list(pc_xyz_mm or [])
        self._v3d_redraw()

    def _v3d_project(self, X, Y, Z, W, H):
        # rotate bởi pitch (quanh trục X) rồi yaw (quanh trục Y)
        cp = math.cos(self._v3d_pitch); sp = math.sin(self._v3d_pitch)
        cy = math.cos(self._v3d_yaw);   sy = math.sin(self._v3d_yaw)
        # Rx pitch
        x1 = X
        y1 =  Y*cp - Z*sp
        z1 =  Y*sp + Z*cp
        # Ry yaw
        x2 =  x1*cy + z1*sy
        y2 =  y1
        z2 = -x1*sy + z1*cy

        # phối cảnh đơn giản (camera nhìn -Z, đặt ở z_cam)
        z_cam = 200.0   # mm
        f = 350.0       # px
        denom = (z_cam - z2)
        if denom <= 1e-3:
            return None
        u = f * (x2 / denom)
        v = f * (y2 / denom)

        # scale/zoom & tịnh tiến vào giữa canvas
        s = self._v3d_zoom
        cx, cy2 = W/2.0, H/2.0
        return int(cx + u*s), int(cy2 - v*s)

    def _v3d_redraw(self):
        if not self.var_view_3d.get():
            return
        self.cnv_right.delete("all")
        W = int(self.cnv_right.winfo_width())
        H = int(self.cnv_right.winfo_height())
        self.cnv_right.create_rectangle(0,0,W,H, fill="#111111", outline="")
        if not self._v3d_pts:
            self.cnv_right.create_text(W//2, H//2, fill="#888888",
                text="(Chưa có point cloud)\nBật 3D sau vài khung hoặc dừng quét để xem.", justify="center")
            return
        self.cnv_right.create_text(8, 8, anchor="nw", fill="#aaaaaa",
                                   text=f"yaw={math.degrees(self._v3d_yaw):.1f}°, pitch={math.degrees(self._v3d_pitch):.1f}°, zoom={self._v3d_zoom:.2f}")
        r = 1
        for (X,Y,Z) in self._v3d_pts:
            p = self._v3d_project(X, Y, Z, W, H)
            if p is None: continue
            x,y = p
            self.cnv_right.create_rectangle(x-r, y-r, x+r, y+r, outline="", fill="#3ddc97")

    def _v3d_on_down(self, ev):
        if not self.var_view_3d.get(): return
        self._v3d_last = (ev.x, ev.y)

    def _v3d_on_drag(self, ev):
        if not self.var_view_3d.get() or self._v3d_last is None: return
        x0,y0 = self._v3d_last
        dx = ev.x - x0
        dy = ev.y - y0
        self._v3d_yaw   += dx * 0.01
        self._v3d_pitch += dy * 0.01
        self._v3d_pitch = max(-math.pi/2+0.01, min(math.pi/2-0.01, self._v3d_pitch))
        self._v3d_last = (ev.x, ev.y)
        self._v3d_redraw()

    def _v3d_on_up(self, ev):
        self._v3d_last = None

    def _v3d_on_wheel(self, ev):
        if not self.var_view_3d.get(): return
        if ev.delta > 0: self._v3d_zoom *= 1.1
        else:            self._v3d_zoom /= 1.1
        self._v3d_zoom = max(0.1, min(10.0, self._v3d_zoom))
        self._v3d_redraw()

    def _v3d_on_wheel_linux(self, ev):
        if not self.var_view_3d.get(): return
        if ev.num == 4: self._v3d_zoom *= 1.1
        elif ev.num == 5: self._v3d_zoom /= 1.1
        self._v3d_zoom = max(0.1, min(10.0, self._v3d_zoom))
        self._v3d_redraw()

    # ========== Export ==========
    def _scan_save_npz(self):
        if not self.scan_data:
            messagebox.showwarning("Lưu", "Chưa có dữ liệu quét."); return
        path = filedialog.asksaveasfilename(defaultextension=".npz",
                                            filetypes=[("NumPy","*.npz")],
                                            initialfile="scan_raw_ransac.npz")
        if not path: return
        steps = np.array([d["x_pos"] for d in self.scan_data], dtype=np.float32)
        tss   = np.array([d["ts"] for d in self.scan_data], dtype=np.float64)
        def arr_or_empty_list(lst):
            if not lst: return np.empty((0,2), dtype=np.float32)
            a = np.array(lst, dtype=np.float32)
            return a if a.size else np.empty((0,2), dtype=np.float32)
        l1 = [arr_or_empty_list(d["laser1"]) for d in self.scan_data]
        l2 = [arr_or_empty_list(d["laser2"]) for d in self.scan_data]
        angles = np.array([ (d.get("angle_deg", np.nan)) for d in self.scan_data ], dtype=np.float32)
        np.savez(path, steps=steps, tss=tss, angles=angles,
                 laser1=np.array(l1, dtype=object), laser2=np.array(l2, dtype=object))
        self._log(f"[SCAN] Đã lưu: {path}")

    def _scan_save_csv(self):
        if not self.scan_data:
            messagebox.showwarning("Lưu", "Chưa có dữ liệu quét."); return
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                            filetypes=[("CSV","*.csv")],
                                            initialfile="scan_raw_ransac.csv")
        if not path: return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["frame_idx","x_pos_mm","ts","angle_deg","laser_id","x_pix","y_pix"])
            for d in self.scan_data:
                i = d["step_idx"]; xp = d["x_pos"]; ts = d["ts"]; ang = d.get("angle_deg", float("nan"))
                for lid, arr in [(1, d["laser1"]), (2, d["laser2"])]:
                    if not arr: continue
                    for (x,y) in arr:
                        w.writerow([i, f"{xp:.5f}", f"{ts:.6f}",
                                    ("" if (ang!=ang) else f"{ang:.5f}"),
                                    lid, int(x), int(y)])
        self._log(f"[SCAN] Đã lưu: {path}")

    def _collect_pointcloud_mm(self):
        """Chuyển dữ liệu quét thành point cloud (mm) theo phép chiếu trụ tròn thô."""
        pc = []  # list of (X,Y,Z)
        if not self.scan_data: return pc
        # đọc tham số
        try: self.table_radius_mm = float(self.ent_table_r.get() or self.table_radius_mm)
        except: pass
        try:
            self.recon_mm_per_px_x = float(self.ent_mmpx_x.get() or self.recon_mm_per_px_x)
            self.recon_mm_per_px_y = float(self.ent_mmpx_y.get() or self.recon_mm_per_px_y)
        except: pass
        if self.recon_cx is None or self.recon_y0 is None:
            # fallback nếu cần
            self.recon_cx = self.recon_cx or 0.0
            self.recon_y0 = self.recon_y0 or 0.0
        for d in self.scan_data:
            ang = d.get("angle_deg", None)
            if ang is None: continue  # chỉ dựng với chế độ quay đế
            th = math.radians(float(ang))
            c, s = math.cos(th), math.sin(th)
            def add_pts(pts):
                if not pts: return
                for (xp, yp) in pts:
                    r_mm = (float(xp) - float(self.recon_cx)) * self.recon_mm_per_px_x
                    if abs(r_mm) > self.table_radius_mm: continue
                    X = r_mm * c
                    Z = r_mm * s
                    Y = (float(self.recon_y0) - float(yp)) * self.recon_mm_per_px_y  # trục dọc lên trên
                    pc.append((X, Y, Z))
            add_pts(d.get("laser1"))
            if self.use_laser2_also: add_pts(d.get("laser2"))
        return pc

    def _export_ply_obj(self):
        if not self.scan_data:
            messagebox.showwarning("Xuất 3D", "Chưa có dữ liệu quét."); return
        pc = self._collect_pointcloud_mm()
        if not pc:
            messagebox.showwarning("Xuất 3D", "Không có điểm hợp lệ (kiểm tra góc/khung và tham số tái tạo).")
            return
        base = filedialog.asksaveasfilename(defaultextension=".ply",
                                            filetypes=[("Point Cloud (*.ply)","*.ply"), ("OBJ (*.obj)","*.obj")],
                                            initialfile="scan_points.ply")
        if not base: return
        root, ext = os.path.splitext(base)
        ply_path = root + ".ply"
        obj_path = root + ".obj"
        # --- PLY ASCII ---
        with open(ply_path, "w") as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {len(pc)}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("end_header\n")
            for (x,y,z) in pc:
                f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
        # --- OBJ (chỉ vertex) ---
        with open(obj_path, "w") as f:
            f.write("# simple point cloud as vertices only\n")
            for (x,y,z) in pc:
                f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        self._log(f"[EXPORT] Đã lưu: {ply_path} và {obj_path}")
        messagebox.showinfo("Xuất 3D", f"Đã lưu:\n{ply_path}\n{obj_path}\n\nMở bằng MeshLab/CloudCompare/Blender để chỉnh mesh (Poisson/ball pivot…).")

    # ---------- UI helpers ----------
    def _log(self, s: str):
        self.txt.insert("end", s+"\n"); self.txt.see("end")

    def _set_state_label(self, state: str):
        self.lbl_state.config(text=f"State: {state}")
        color = {"Idle":"#2ecc71","Run":"#3498db","Jog":"#9b59b6","Hold":"#f1c40f","Alarm":"#e74c3c","Door":"#e67e22","Sleep":"#95a5a6"}.get(state,"#555")
        self.lbl_state.config(foreground=color)

    def _push_preview(self, frame):
        if frame is None: return
        if not HAS_PIL:
            self.lbl_prev.configure(text="(Cài Pillow để xem preview)")
            return
        show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h,w,_ = show.shape
        maxw,maxh = 1100,380
        scale = min(maxw/w, maxh/h)
        show = cv2.resize(show, (int(w*scale), int(h*scale)))
        im = Image.fromarray(show)
        self.tk_prev = ImageTk.PhotoImage(image=im)
        self.lbl_prev.configure(image=self.tk_prev)

    def _push_scan_preview(self, frame):
        if frame is None: return
        if not HAS_PIL:
            self.lbl_prev_scan.configure(text="(Cài Pillow để xem preview)")
            return
        show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h,w,_ = show.shape
        maxw,maxh = 540,360
        scale = min(maxw/w, maxh/h)
        show = cv2.resize(show, (int(w*scale), int(h*scale)))
        im = Image.fromarray(show)
        self.tk_prev_scan = ImageTk.PhotoImage(image=im)
        self.lbl_prev_scan.configure(image=self.tk_prev_scan)

    # --------- Giám sát quay đế (optical flow + affine) ---------
    def _update_spin_monitor(self, frame_bgr):
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        now = time.time()
        if self.spin_prev_gray is None:
            self.spin_prev_gray = gray; self.spin_prev_ts = now
            self.spin_prev_pts = cv2.goodFeaturesToTrack(gray, maxCorners=200, qualityLevel=0.01, minDistance=8)
            return
        if self.spin_prev_pts is None:
            self.spin_prev_pts = cv2.goodFeaturesToTrack(self.spin_prev_gray, maxCorners=200, qualityLevel=0.01, minDistance=8)
            if self.spin_prev_pts is None:
                self.spin_prev_gray = gray; self.spin_prev_ts = now; return
        nxt_pts, st, err = cv2.calcOpticalFlowPyrLK(self.spin_prev_gray, gray, self.spin_prev_pts, None)
        if nxt_pts is None or st is None:
            self.spin_prev_gray = gray; self.spin_prev_ts = now
            self.spin_prev_pts = cv2.goodFeaturesToTrack(gray, maxCorners=200, qualityLevel=0.01, minDistance=8)
            return
        p0 = self.spin_prev_pts[st==1]; p1 = nxt_pts[st==1]
        if len(p0) >= 10 and len(p1) >= 10:
            M, inliers = cv2.estimateAffinePartial2D(p0.reshape(-1,1,2), p1.reshape(-1,1,2),
                                                     method=cv2.RANSAC, ransacReprojThreshold=3.0)
            if M is not None:
                a, b = M[0,0], M[0,1]
                angle_rad = math.atan2(b, a)
                dt = max(1e-3, (now - (self.spin_prev_ts or now)))
                omega_deg_s = (angle_rad * 180.0 / math.pi) / dt
                self.spin_ema_deg_per_s = 0.8*self.spin_ema_deg_per_s + 0.2*abs(omega_deg_s)
                if self.var_auto_angle.get():
                    self.spin_total_angle += (angle_rad * 180.0 / math.pi)
        spinning = self.spin_ema_deg_per_s > 0.5
        txt = f"Bàn quay: {'ĐANG QUAY' if spinning else 'ĐỨNG YÊN'} | ω≈{self.spin_ema_deg_per_s:.2f} deg/s"
        self.lbl_spin.config(text=txt, foreground=("#2ecc71" if spinning else "#e74c3c"))
        self.spin_prev_gray = gray; self.spin_prev_ts = now
        self.spin_prev_pts = cv2.goodFeaturesToTrack(gray, maxCorners=200, qualityLevel=0.01, minDistance=8)

    def _tick(self):
        # đọc log từ GRBL
        while True:
            try: line = self.client.log_q.get_nowait()
            except queue.Empty: break
            else: self._log(line)
        # cập nhật state
        if self.client.last_status_line:
            info = parse_grbl_status(self.client.last_status_line)
            self._set_state_label(info.get("state","UNKNOWN"))
        if self.client.is_open():
            try: self.client.status()
            except: pass
        # live preview + monitor quay
        if self.cam.running:
            frame = self.cam.read()
            if frame is not None:
                ok, corners, overlay, _, info = self.cam.detect_pattern(frame)
                self._push_preview(overlay if ok else frame)
                if not self.scan_running:
                    self._push_scan_preview(frame)
                self._update_spin_monitor(frame)
        self.after(self.status_poll_interval_ms, self._tick)

    def ent_feed_get(self):
        try: return float(self.ent_feed.get())
        except: return 5.0

if __name__ == "__main__":
    app = App()
    app.mainloop()
