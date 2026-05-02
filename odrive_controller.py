import threading
import time
import odrive
import math
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    CONTROL_MODE_TORQUE_CONTROL,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE
)
from collections import deque

CLOSED_LOOP_CONTROL = AXIS_STATE_CLOSED_LOOP_CONTROL
IDLE = AXIS_STATE_IDLE
DEG2RAD = math.pi / 180

gear_ratio = 100.0
g = 9.81


class ODriveThread(threading.Thread):
    def __init__(self):
        super().__init__()

        # running state
        self.connected = False
        self.closed_loop_control = False
        self.isOffset = False
        self.error = False
        self.data_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._estop_event = threading.Event()

        # Cờ hiệu an toàn khi chạy chế độ đo lường tự động
        self.is_calibrating_friction = False

        # Driver components
        self.odrv = None
        self.axis = None
        self.max_torque = 0.15  # min limit
        self.tor_coef = 0.708282
        self.Kt = 8.27 / 270
        self.offset = 0.0

        # Physic components
        self.start_pos = -90

        self.link_mass = 1.125
        self.link_length = 0.7
        self.center_distance = 0.7 / 2 - 0.04  # 0.31
        self.motor_inertia = 0.000643
        self.const_inertia = 1 / 12 * self.link_mass * (self.link_length ** 2) + self.link_mass * (
                self.center_distance ** 2) + gear_ratio ** 2 * self.motor_inertia  # 6.584

        self.hanger_mass = 0.26
        self.ext_load = 0.0
        self.hanger_distance = 0.7 - 0.04 - 0.06  # 0.6

        self.m = self.link_mass + self.hanger_mass  # 1.385
        self.lc = (self.center_distance * self.link_mass + self.hanger_distance * self.hanger_mass) / self.m  # 0.36444
        self.Ic = self.const_inertia + self.hanger_mass * (self.hanger_distance ** 2)  # 6.6776

        self.coul_friction = 0.0
        self.visc_friction = 0.00276 * gear_ratio ** 2

        # inputs
        self.Kp = 10
        self.Kd = 5
        self.torque_set = 0.0
        self.pos_set = self.start_pos
        self.vel_set = 0.0
        self.acc_set = 0.0
        self.time_set = 5.0
        self.ctrl_bandwidth = 2000
        self.enc_bandwidth = 50

        # show
        self.pos = 0.0
        self.vel = 0.0
        self.pre_vel = 0.0
        self.acc = 0.0
        self.pre_acc = 0.0
        self.jerk = 0.0
        self.pre_jerk = 0.0
        self.preT = 0.0
        self.data = deque(maxlen=800)

    def connect(self):
        try:
            print("Connecting to Odrive...")
            self.odrv = odrive.find_any(timeout=10)
            if self.odrv:
                print(f"Connected to Odrive: {self.odrv.serial_number}")
                self.axis = self.odrv.axis0
                self.connected = True
                self.error = False
            else:
                print("ODrive not found")
        except Exception as e:
            self.error = True
            print(f"Connection failed: {e}")

    def clear_error(self):
        if self.axis:
            self.axis.controller.error = 0
            self.axis.encoder.error = 0
            self.axis.motor.error = 0
            self.axis.error = 0
            self.error = False
            print("Errors cleared")

    def enter_closed_loop(self):
        self.clear_error()
        if self.axis:
            self.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            self.axis.requested_state = CLOSED_LOOP_CONTROL
            self.closed_loop_control = True
            print("Entered CLOSED_LOOP_CONTROL mode.")

    def return_IDLE(self):
        if self.axis:
            self.axis.controller.input_torque = 0
            self.axis.requested_state = IDLE
            self.closed_loop_control = False

    def is_controlable(self):
        return self.connected and self.closed_loop_control and self.isOffset and not self._estop_event.is_set()

    def update_ctrlElms(self, *ctrlElms):
        try:
            with self.data_lock:
                self.pos_set = ctrlElms[0]
                self.time_set = ctrlElms[1]
                self.Kp = ctrlElms[2]
                self.Kd = ctrlElms[3]
                self.ctrl_bandwidth = ctrlElms[4]
                self.enc_bandwidth = ctrlElms[5]
                # [SỬA LỖI] Phải check self.axis có tồn tại không trước khi nạp config, đề phòng GUI gửi lệnh lúc chưa cắm cáp
                if self.axis:
                    self.axis.motor.config.current_control_bandwidth = self.ctrl_bandwidth
                    self.axis.encoder.config.bandwidth = self.enc_bandwidth
        except Exception as e:
            print("Elements update error:", e)

    def update_loadParms(self, *loadParms):
        try:
            with self.data_lock:
                self.ext_load = loadParms[0]
                self.hanger_distance = loadParms[1]
                self.coul_friction = loadParms[2]
                self.visc_friction = 0.00276 * gear_ratio ** 2 + loadParms[3]
                self.m = self.link_mass + self.hanger_mass + self.ext_load
                self.lc = (self.center_distance * self.link_mass + self.hanger_distance * (
                        self.hanger_mass + self.ext_load)) / self.m
                self.Ic = self.const_inertia + (self.hanger_mass + self.ext_load) * (self.hanger_distance ** 2)
                self.max_torque = loadParms[4]
        except Exception as e:
            print("Parameter update error:", e)

    def get_state(self):
        if self.connected and self.axis: return self.axis.current_state
        return None

    def emergency_stop(self):
        self._estop_event.set()
        self.torque_set = 0.0
        try:
            if self.connected and self.axis is not None:
                self.axis.controller.input_torque = 0
        except Exception:
            pass

    def get_data(self):
        with self.data_lock:
            return list(self.data)

    def set_offset(self):
        try:
            if self.axis:
                self.offset = self.axis.encoder.pos_estimate
                self.isOffset = True
        except Exception:
            pass

    def reset(self):
        self.return_IDLE()
        self.isOffset = False
        self._estop_event.clear()

    def stop(self):
        self._stop_event.set()
        try:
            if self.axis is not None:
                self.axis.controller.input_torque = 0
                time.sleep(0.05)
                self.axis.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass

    # =========================================================================
    # KHỐI HÀM ĐO MA SÁT KHÔ TỰ ĐỘNG (Đã tích hợp toán học tách ma sát nhớt)
    # =========================================================================
    def get_friction_torque_at_angle(self, target_angle, tolerance=2.0, expected_direction=1):
        with self.data_lock:
            valid_samples = []
            for row in self.data:
                pos = row[1]
                vel = row[2]
                tor = row[8]

                if abs(pos - target_angle) <= tolerance:
                    if (expected_direction > 0 and vel > 0) or (expected_direction < 0 and vel < 0):
                        valid_samples.append(tor)

        if not valid_samples:
            return None

        return sum(valid_samples) / len(valid_samples)

    def auto_measure_friction(self, target_angle=0.0, test_vel=0.05):
        self.is_calibrating_friction = True
        tau_len = None
        tau_xuong = None
        coulomb_fric_pure = None
        try:
            self.axis.controller.config.control_mode = 2
            self.axis.controller.config.input_mode = 1
            self.axis.requested_state = 8
            time.sleep(0.2)

            # LƯỢT 1: ĐI LÊN
            self.axis.controller.input_vel = -0.5
            time.sleep(2)

            self.axis.controller.input_vel = test_vel
            time.sleep(4)
            self.axis.controller.input_vel = 0.0
            time.sleep(0.5)

            tau_len = self.get_friction_torque_at_angle(target_angle, expected_direction=1)

            # LƯỢT 2: ĐI XUỐNG
            self.axis.controller.input_vel = 0.5
            time.sleep(2)

            self.axis.controller.input_vel = -test_vel
            time.sleep(4)
            self.axis.controller.input_vel = 0.0
            time.sleep(0.5)

            tau_xuong = self.get_friction_torque_at_angle(target_angle, expected_direction=-1)

            # TÍNH TOÁN & LƯU BIẾN TỰ ĐỘNG
            if tau_len is not None and tau_xuong is not None:
                tong_ma_sat = (tau_len - tau_xuong) / 2.0

                # Chuyển đổi vận tốc từ vòng/s sang rad/s để ráp vào phương trình vật lý
                vel_rad_s = test_vel * 2.0 * math.pi

                # Tính lượng ma sát nhớt đang pha tạp bên trong (tau = D * q_dot)
                tau_viscous = self.visc_friction * vel_rad_s

                # Trừ đi để cô lập Ma sát khô (Coulomb)
                coulomb_fric_pure = abs(tong_ma_sat) - tau_viscous

                # Ghi thẳng vào bộ nhớ để vòng lặp CTC (dynamic_calculation) lấy xài luôn
                with self.data_lock:
                    # Ép hàm max(0, ...) để đề phòng D quá lớn làm ma sát khô bị tính ra số ÂM vô lý
                    self.coul_friction = max(0.0, coulomb_fric_pure)

        except Exception as e:
            print(f"Lỗi đo lường: {e}")
        finally:
            self.axis.controller.input_vel = 0.0
            self.is_calibrating_friction = False

        # Vẫn nôn ra 2 giá trị gốc để cho file main.py lấy lưu vào CSV vẽ biểu đồ
        return tau_len, tau_xuong, coulomb_fric_pure

    # =========================================================================
    # VÒNG LẶP ĐIỀU KHIỂN ĐỘNG LỰC HỌC (CTC) 100Hz
    # =========================================================================
    def dynamic_calculation(self):
        m = self.m
        lc = self.lc
        Ic = self.Ic

        q = self.pos * DEG2RAD
        qdot = self.vel * DEG2RAD
        q_d = self.pos_set * DEG2RAD
        qdot_d = self.vel_set * DEG2RAD
        qddot_d = self.acc_set * DEG2RAD

        Kp = self.Kp
        Kd = self.Kd
        ep = q - q_d
        ev = qdot - qdot_d
        D = self.visc_friction

        direction = math.copysign(1.0, qdot) if abs(qdot) > 0.001 else 0.0

        tor = Ic * (qddot_d - (Kp * ep + Kd * ev)) + m * g * lc * math.cos(
            q) + D * qdot + self.coul_friction * direction
        tor = tor / self.tor_coef / gear_ratio

        self.torque_set = max(min(tor, self.max_torque), -self.max_torque)

    def run(self):
        self.preT = time.perf_counter()

        while not self._stop_event.is_set():
            t_start = time.perf_counter()
            try:
                if not self.connected:
                    self.connect()
                    if not self.connected:
                        time.sleep(0.1)
                        continue

                if self._estop_event.is_set():
                    self._stop_event.wait(0.1)
                    continue

                # Data collect
                with self.data_lock:
                    t = time.perf_counter()
                    deltaT = t - self.preT

                    if deltaT <= 0:
                        deltaT = 0.0001

                    self.pos = (self.axis.encoder.pos_estimate - self.offset) * 360 / gear_ratio + self.start_pos
                    self.vel = self.axis.encoder.vel_estimate * 360 / gear_ratio

                    raw_acc = (self.vel - self.pre_vel) / deltaT
                    self.acc = 0.1 * raw_acc + 0.9 * self.pre_acc

                    raw_jerk = (self.acc - self.pre_acc) / deltaT
                    self.jerk = 0.05 * raw_jerk + 0.95 * self.pre_jerk

                    self.pre_vel = self.vel
                    self.pre_acc = self.acc
                    self.pre_jerk = self.jerk
                    self.preT = t

                    tor_set = self.axis.motor.current_control.Iq_setpoint * self.Kt
                    self.data.append((time.time(), self.pos, self.vel, self.pos_set, self.vel_set, self.acc,
                                      self.acc_set, self.jerk, tor_set))

                # Chặn không cho CTC giành lệnh lúc đang chạy bài test quét ma sát
                if self.is_controlable() and not self.is_calibrating_friction:
                    with self.data_lock:
                        self.dynamic_calculation()
                        self.axis.controller.input_torque = self.torque_set
                elif not self.is_calibrating_friction:
                    self.pos_set = self.start_pos
                    self.torque_set = 0.0

            except Exception as e:
                print("ODrive error:", e)
                self.connected = False
                self.closed_loop_control = False
                self.error = True
                time.sleep(1)

            t_end = time.perf_counter()
            t_sleep = 0.01 - (t_end - t_start)
            if t_sleep > 0:
                self._stop_event.wait(timeout=t_sleep)