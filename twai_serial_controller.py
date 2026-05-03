import threading
import time
import math
import serial
import serial.tools.list_ports
from collections import deque

# ── Constants (khớp với các controller khác trong project) ──────────────────
AXIS_STATE_IDLE               = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
CLOSED_LOOP_CONTROL           = AXIS_STATE_CLOSED_LOOP_CONTROL
IDLE                          = AXIS_STATE_IDLE

gear_ratio = 100.0   # hệ số hộp số (giống trajectory_controller.py)
DEG2RAD    = math.pi / 180


# ── Helper ──────────────────────────────────────────────────────────────────
def list_serial_ports():
    """Trả danh sách các COM port khả dụng."""
    return [p.device for p in serial.tools.list_ports.comports()]


# ── Main Controller Class ────────────────────────────────────────────────────
class TWAIController(threading.Thread):
    """
    Thread controller điều khiển 2 motor ODrive qua ESP32 TWAI bridge.
    API tương thích với trajectory_controller.ODriveThread để tái sử dụng
    guicontroller.py với thay đổi tối thiểu.
    """

    def __init__(self, serial_port: str = "COM3", baudrate: int = 115200):
        super().__init__(daemon=True)

        # ── Serial config ────────────────────────────────────────────────
        self.serial_port = serial_port
        self.baudrate    = baudrate
        self.ser: serial.Serial | None = None

        # ── State flags (GUI reads these) ────────────────────────────────
        self.connected          = False
        self.closed_loop_control = False
        self.isOffset           = False
        self.error              = False
        self.esp32_ready        = False   # True sau khi nhận "READY:"
        self.status_message     = "Chưa kết nối"

        # ── Threading primitives ─────────────────────────────────────────
        self.data_lock   = threading.Lock()
        self._stop_event  = threading.Event()
        self._estop_event = threading.Event()

        # ── Physics / Kinematic (giống các controller cũ) ────────────────
        self.start_pos       = 0.0        # degrees – vị trí gốc khi offset
        self.gear_ratio      = gear_ratio

        # ── Offset (revolutions, raw ODrive value tại thời điểm set_offset) ─
        self.offset_rev = [0.0, 0.0]      # offset cho motor 0 và 1

        # ── Position state (degrees) ─────────────────────────────────────
        self.pos  = [0.0, 0.0]           # vị trí hiện tại (degrees)
        self.pos_set = [0.0, 0.0]        # setpoint (degrees)
        self.vel_set = [0.0, 0.0]        # feedforward velocity (deg/s)
        self._last_pos_set = [0.0, 0.0]
        self._last_set_ts = time.perf_counter()
        self._setpoint_dirty = False

        # ── Raw position từ ESP32 (revolutions) ──────────────────────────
        self._raw_pos = [0.0, 0.0]
        self.torque_set = [0.0, 0.0]
        self.use_torque_commands = True

        # ── Control params (dùng cho GUI control panel) ──────────────────
        self.Kp             = 10.0
        self.Kd             = 5.0
        self.ctrl_bandwidth = 2000
        self.enc_bandwidth  = 50
        self.max_torque     = 0.15       # kept for GUI compatibility
        self.ext_load       = 0.0
        self.hanger_distance = 0.6
        self.coul_friction  = 0.0
        self.visc_friction  = 27.6
        self.window_size    = 25
        self.poly_order     = 2

        # ── Data buffer (GUI reads for plotting) ─────────────────────────
        # Tuple: (timestamp, pos0_deg, pos1_deg, pos0_set_deg, pos1_set_deg)
        self.data: deque = deque(maxlen=800)

        # ── Serial receive line buffer ───────────────────────────────────
        self._line_buf = ""

    # ════════════════════════════════════════════════════════════════════════
    # Connection
    # ════════════════════════════════════════════════════════════════════════

    def connect(self):
        """Mở cổng Serial. Gọi từ run() hoặc từ GUI thread."""
        try:
            print(f"[TWAI] Kết nối tới {self.serial_port} @ {self.baudrate}...")
            self.ser = serial.Serial(
                self.serial_port,
                self.baudrate,
                timeout=0.02
            )
            self.connected     = True
            self.error         = False
            self.esp32_ready   = False
            self.status_message = f"Đã kết nối {self.serial_port}, đang chờ ESP32..."
            print(f"[TWAI] Kết nối Serial thành công.")
        except Exception as e:
            self.connected      = False
            self.error          = True
            self.status_message = f"Lỗi kết nối: {e}"
            print(f"[TWAI] Lỗi Serial: {e}")

    def disconnect(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.connected = False

    # ════════════════════════════════════════════════════════════════════════
    # State machine (GUI compatibility)
    # ════════════════════════════════════════════════════════════════════════

    def get_state(self):
        if not self.connected:
            return None
        return CLOSED_LOOP_CONTROL if self.closed_loop_control else IDLE

    def _send_simple_cmd(self, cmd: str):
        if not self.ser or not self.ser.is_open:
            return
        try:
            self.ser.write((cmd + "\n").encode("ascii"))
        except Exception as e:
            print(f"[TWAI] Lỗi ghi Serial ({cmd}): {e}")
            self.connected = False

    def enter_closed_loop(self):
        """Kích hoạt chế độ gửi lệnh position và yêu cầu bridge vào CLOSED_LOOP."""
        if not self.esp32_ready:
            print("[TWAI] ESP32 chưa READY, không thể vào Closed Loop.")
            return
        self._send_simple_cmd("CLEAR")
        self._send_simple_cmd("CLOSE")
        self.closed_loop_control = True
        self.status_message = "Closed Loop đang chạy"
        print("[TWAI] Đã vào CLOSED_LOOP_CONTROL.")

    def return_IDLE(self):
        self._send_simple_cmd("IDLE")
        self.closed_loop_control = False
        self.status_message = "IDLE"
        print("[TWAI] Trở về IDLE.")

    def is_controlable(self):
        return (
            self.connected
            and self.esp32_ready
            and self.closed_loop_control
            and self.isOffset
            and not self._estop_event.is_set()
            and self._setpoint_dirty
        )

    def emergency_stop(self):
        self._estop_event.set()
        self._send_simple_cmd("IDLE")
        self.status_message = "ESTOP!"
        print("[TWAI] EMERGENCY STOP!")

    def reset(self):
        self._estop_event.clear()
        self.isOffset = False
        self.return_IDLE()
        self._send_simple_cmd("CLEAR")
        self.status_message = "Reset xong"
        print("[TWAI] Reset.")

    def stop(self):
        self._stop_event.set()
        self.disconnect()

    # ════════════════════════════════════════════════════════════════════════
    # Offset (lấy vị trí hiện tại làm gốc)
    # ════════════════════════════════════════════════════════════════════════

    def set_offset(self):
        """
        Lưu raw revolutions hiện tại làm offset tham chiếu, giống
        trajectory_controller.py dùng axis.encoder.pos_estimate.
        """
        with self.data_lock:
            self.offset_rev[0] = self._raw_pos[0]
            self.offset_rev[1] = self._raw_pos[1]
            self.isOffset = True
        print(f"[TWAI] Offset set: motor0={self.offset_rev[0]:.4f} rev, "
              f"motor1={self.offset_rev[1]:.4f} rev")

    # ════════════════════════════════════════════════════════════════════════
    # Unit conversion helpers
    # ════════════════════════════════════════════════════════════════════════

    def _rev_to_deg(self, rev: float, motor_id: int) -> float:
        """ODrive raw rev → degrees (với offset và start_pos)."""
        return (rev - self.offset_rev[motor_id]) * 360.0 / gear_ratio + self.start_pos

    def _deg_to_rev(self, deg: float, motor_id: int) -> float:
        """Degrees → ODrive raw rev (ngược lại với _rev_to_deg)."""
        return (deg - self.start_pos) * gear_ratio / 360.0 + self.offset_rev[motor_id]

    # ════════════════════════════════════════════════════════════════════════
    # Serial Protocol
    # ════════════════════════════════════════════════════════════════════════

    def _send_position(self, motor_id: int, pos_rev: float, vel_rev: float = 0.0):
        """Gửi lệnh setPosition cho motor (đơn vị: revolutions, rev/s)."""
        if not self.ser or not self.ser.is_open:
            return
        cmd = f"P{motor_id}:{pos_rev:.6f},{vel_rev:.6f}\n"
        try:
            self.ser.write(cmd.encode("ascii"))
        except Exception as e:
            print(f"[TWAI] Lỗi ghi Serial: {e}")
            self.connected = False

    def _process_serial(self):
        """Đọc và parse các dòng từ ESP32."""
        if not self.ser or not self.ser.is_open:
            return
        try:
            waiting = self.ser.in_waiting
            if waiting > 0:
                raw = self.ser.read(waiting).decode("ascii", errors="replace")
                self._line_buf += raw

                while "\n" in self._line_buf:
                    line, self._line_buf = self._line_buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    self._parse_line(line)
        except Exception as e:
            print(f"[TWAI] Lỗi đọc Serial: {e}")
            self.connected = False

    def _parse_line(self, line: str):
        """Xử lý một dòng text từ ESP32."""
        if line.startswith("FB,"):
            # Format: FB,<pos0>,<pos1>
            parts = line[3:].split(",")
            if len(parts) >= 2:
                try:
                    raw0 = float(parts[0])
                    raw1 = float(parts[1])
                    with self.data_lock:
                        self._raw_pos[0] = raw0
                        self._raw_pos[1] = raw1
                        # Convert sang degrees
                        self.pos[0] = self._rev_to_deg(raw0, 0)
                        self.pos[1] = self._rev_to_deg(raw1, 1)
                        # Lưu data để GUI plot
                        self.data.append((
                            time.time(),
                            self.pos[0], self.pos[1],
                            self.pos_set[0], self.pos_set[1]
                        ))
                except ValueError:
                    pass

        elif line.startswith("READY"):
            self.esp32_ready = True
            self.status_message = "ESP32 READY — có thể vào Closed Loop"
            print(f"[TWAI] {line}")

        elif line.startswith("STATUS"):
            self.status_message = line
            print(f"[TWAI] {line}")

        elif line.startswith("WARN"):
            self.status_message = line
            print(f"[TWAI] {line}")

        elif line.startswith("INFO"):
            self.status_message = line
            print(f"[TWAI] {line}")

        elif line.startswith("ERROR"):
            self.error = True
            self.status_message = line
            print(f"[TWAI] {line}")

    # ════════════════════════════════════════════════════════════════════════
    # GUI-facing control methods
    # ════════════════════════════════════════════════════════════════════════

    def set_target(self, motor_id: int, pos_deg: float):
        """
        Đặt setpoint cho một motor (đơn vị: degrees).
        Nếu đang Closed Loop, lệnh sẽ được gửi ngay trong vòng lặp kế tiếp.
        """
        with self.data_lock:
            now = time.perf_counter()
            dt = max(now - self._last_set_ts, 1e-3)
            self.vel_set[motor_id] = (pos_deg - self._last_pos_set[motor_id]) / dt
            self.pos_set[motor_id] = pos_deg
            self._last_pos_set[motor_id] = pos_deg
            self._last_set_ts = now
            self._setpoint_dirty = True

    def set_both_targets(self, pos0_deg: float, pos1_deg: float):
        """Đặt setpoint cho cả 2 motor cùng lúc."""
        with self.data_lock:
            now = time.perf_counter()
            dt = max(now - self._last_set_ts, 1e-3)
            self.vel_set[0] = (pos0_deg - self._last_pos_set[0]) / dt
            self.vel_set[1] = (pos1_deg - self._last_pos_set[1]) / dt
            self.pos_set[0] = pos0_deg
            self.pos_set[1] = pos1_deg
            self._last_pos_set[0] = pos0_deg
            self._last_pos_set[1] = pos1_deg
            self._last_set_ts = now
            self._setpoint_dirty = True

    def get_data(self):
        """Trả snapshot của data buffer cho GUI plot."""
        with self.data_lock:
            return list(self.data)

    def get_pos(self):
        """Trả vị trí hiện tại (degrees) của cả 2 motor."""
        with self.data_lock:
            return self.pos[0], self.pos[1]

    # ── Compatibility shims cho guicontroller.py ────────────────────────────
    def update_ctrlElms(self, *ctrlElms):
        """
        Cập nhật control elements từ GUI.
        ctrlElms = (target_deg_m0, target_deg_m1, Kp, Kd, ctrl_bw, enc_bw)
        """
        with self.data_lock:
            if len(ctrlElms) >= 2:
                now = time.perf_counter()
                dt = max(now - self._last_set_ts, 1e-3)
                new0 = float(ctrlElms[0])
                new1 = float(ctrlElms[1])
                self.vel_set[0] = (new0 - self._last_pos_set[0]) / dt
                self.vel_set[1] = (new1 - self._last_pos_set[1]) / dt
                self.pos_set[0] = new0
                self.pos_set[1] = new1
                self._last_pos_set[0] = new0
                self._last_pos_set[1] = new1
                self._last_set_ts = now
                self._setpoint_dirty = True
            if len(ctrlElms) >= 4:
                self.Kp = float(ctrlElms[2])
                self.Kd = float(ctrlElms[3])
            if len(ctrlElms) >= 6:
                self.ctrl_bandwidth = float(ctrlElms[4])
                self.enc_bandwidth  = float(ctrlElms[5])

    def update_loadParms(self, *loadParms):
        """Cập nhật load parameters từ GUI (giữ tương thích)."""
        with self.data_lock:
            if len(loadParms) >= 1: self.ext_load        = float(loadParms[0])
            if len(loadParms) >= 2: self.hanger_distance  = float(loadParms[1])
            if len(loadParms) >= 3: self.coul_friction    = float(loadParms[2])
            if len(loadParms) >= 4: self.visc_friction     = float(loadParms[3])
            if len(loadParms) >= 5: self.max_torque        = float(loadParms[4])

    def clear_error(self):
        self.error = False

    # ════════════════════════════════════════════════════════════════════════
    # Main Thread Loop
    # ════════════════════════════════════════════════════════════════════════

    def run(self):
        """Vòng lặp chính: ~100Hz — đọc Serial và gửi lệnh position."""
        print("[TWAI] Thread khởi động.")

        while not self._stop_event.is_set():
            t_start = time.perf_counter()

            # ── Kết nối nếu chưa kết nối ─────────────────────────────────
            if not self.connected:
                self.connect()
                if not self.connected:
                    self._stop_event.wait(0.5)
                    continue

            # ── ESTOP: không làm gì, chỉ chờ ─────────────────────────────
            if self._estop_event.is_set():
                self._stop_event.wait(0.05)
                continue

            try:
                # 1. Đọc và parse phản hồi từ ESP32
                self._process_serial()

                # Poll trạng thái bridge định kỳ
                if int(time.time() * 10) % 20 == 0:
                    self._send_simple_cmd("STATUS")

                # 2. Gửi lệnh position nếu đang Closed Loop
                if self.is_controlable():
                    with self.data_lock:
                        p0_set = self.pos_set[0]
                        p1_set = self.pos_set[1]
                    # Convert degrees → revolutions rồi gửi
                    rev0 = self._deg_to_rev(p0_set, 0)
                    rev1 = self._deg_to_rev(p1_set, 1)
                    vel0 = self.vel_set[0] * gear_ratio / 360.0
                    vel1 = self.vel_set[1] * gear_ratio / 360.0
                    self._send_position(0, rev0, vel0)
                    self._send_position(1, rev1, vel1)
                    self._setpoint_dirty = False

            except Exception as e:
                print(f"[TWAI] Lỗi vòng lặp: {e}")
                self.connected = False
                self.error     = True
                self._stop_event.wait(1.0)

            # ── Giữ ~100Hz ────────────────────────────────────────────────
            elapsed = time.perf_counter() - t_start
            sleep_t = 0.01 - elapsed
            if sleep_t > 0:
                self._stop_event.wait(sleep_t)

        print("[TWAI] Thread dừng.")
