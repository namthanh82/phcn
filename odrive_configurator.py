"""ODrive configuration helpers.

Use this module to configure each ODrive board separately, assign CAN node IDs,
enable cyclic feedback, and optionally run full calibration on axis0.
"""

import argparse
import time

import odrive
from odrive.enums import (
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_IDLE,
    ControlMode,
    InputMode,
)


def find_odrive(serial_number=None, timeout=10):
    if serial_number:
        print(f"Searching for ODrive serial={serial_number}...")
        return odrive.find_any(serial_number=serial_number, timeout=timeout)

    print("Searching for ODrive...")
    return odrive.find_any(timeout=timeout)


def configure_odrive_axis(
    axis,
    node_id,
    control_mode=ControlMode.TORQUE_CONTROL,
    input_mode=InputMode.PASSTHROUGH,
    bandwidth=20,
    encoder_rate_ms=10,
    heartbeat_rate_ms=100,
    cpr=4096,
):
    try:
        print(f"Configuring axis with Node ID {node_id}...")

        # Encoder MT6835
        axis.encoder.config.mode = 0  # 0 indicates INCREMENTAL encoder
        axis.encoder.config.cpr = cpr
        # Lỗi 4 (NO_RESPONSE) thường do không bắt được xung index hoặc mất kênh
        # Bắt đầu kiểm tra xem có phải do INDEX không, hãy thử tắt INDEX đi
        axis.encoder.config.use_index = False
        axis.encoder.config.pre_calibrated = False 
        axis.encoder.config.ignore_illegal_hall_state = True
        print(f"  - Encoder: INCREMENTAL, {cpr} CPR, use_index=False, pre_calibrated=False")

        axis.controller.config.control_mode = control_mode
        print(f"  - Control mode: {control_mode}")

        axis.controller.config.input_mode = input_mode
        print(f"  - Input mode: {input_mode}")

        axis.controller.config.input_filter_bandwidth = bandwidth
        print(f"  - Bandwidth: {bandwidth}")

        axis.config.can.node_id = node_id
        print(f"  - CAN Node ID: {node_id}")

        axis.config.can.encoder_rate_ms = encoder_rate_ms
        axis.config.can.heartbeat_rate_ms = heartbeat_rate_ms
        print(f"  - Encoder rate: {encoder_rate_ms} ms")
        print(f"  - Heartbeat rate: {heartbeat_rate_ms} ms")

        print(f"Axis {node_id} configured successfully!\n")
        return True

    except Exception as e:
        print(f"Error configuring axis {node_id}: {e}")
        return False


def run_full_calibration(axis, timeout_s=120):
    try:
        print("Starting full calibration sequence...")
        axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

        start = time.time()
        while time.time() - start < timeout_s:
            if axis.current_state == AXIS_STATE_IDLE and axis.error == 0:
                print("Calibration finished successfully.")
                return True

            if axis.error != 0:
                print(f"Calibration failed with axis error: {axis.error}")
                print(f"  -> Motor error: {axis.motor.error}")
                print(f"  -> Encoder error: {axis.encoder.error}")
                
                # Gợi ý lỗi phổ biến
                if axis.encoder.error == 2:
                    print("  [!] Loi 2: Sai so CPR hoac Pole Pairs (So cap cuc motor chua dung).")
                elif axis.encoder.error == 32 or axis.encoder.error == 16:
                    print("  [!] Loi: Khong tim thay xung Z (Index) hoac ban bi nhieu tin hieu.")
                


                return False

            time.sleep(0.5)

        print("Calibration timed out.")
        return False

    except Exception as e:
        print(f"Error during calibration: {e}")
        return False


def configure_odrive_board(
    serial_number=None,
    node_id=0,
    auto_calibrate=True,
    control_mode=ControlMode.TORQUE_CONTROL,
    input_mode=InputMode.PASSTHROUGH,
    bandwidth=20,
    encoder_rate_ms=10,
    heartbeat_rate_ms=100,
    cpr=16384,
):
    try:
        odrv = find_odrive(serial_number=serial_number)
        print(f"ODrive found: {odrv}\n")

        print("Clearing any previous errors...")
        try:
            odrv.clear_errors()
        except:
            # Fallback cho firmware cu
            odrv.axis0.error = 0
            odrv.axis0.motor.error = 0
            odrv.axis0.encoder.error = 0
            odrv.axis0.controller.error = 0

        axis = odrv.axis0
        ok = configure_odrive_axis(
            axis,
            node_id=node_id,
            control_mode=control_mode,
            input_mode=input_mode,
            bandwidth=bandwidth,
            encoder_rate_ms=encoder_rate_ms,
            heartbeat_rate_ms=heartbeat_rate_ms,
            cpr=cpr,
        )
        if not ok:
            return None

        if auto_calibrate:
            if not run_full_calibration(axis):
                return None

        print("=" * 50)
        print("Saving configuration...")
        try:
            odrv.save_configuration()
        except type(odrv).__module__.count == "odrive.fibre":
            # ignore disconnect error on save_configuration from Fibre Object
            pass
        except Exception as save_err:
            if "disconnected" in str(save_err).lower() or type(save_err).__name__ == "ChannelBrokenException":
                pass
            else:
                raise save_err
        print("Configuration saved!")

        print("\nRebooting ODrive...")
        time.sleep(1)
        try:
            odrv.reboot()
        except:
            pass
        print("ODrive rebooted successfully!\n")

        return odrv

    except Exception as e:
        print(f"Error: {e}")
        print("Make sure ODrive is connected via USB and powered on.")
        return None


def configure_two_odrives(
    dev0_serial=None,
    dev1_serial=None,
    dev0_node_id=0,
    dev1_node_id=1,
    auto_calibrate=True,
    cpr=16384,
):
    print("Configuring dev0...")
    dev0 = configure_odrive_board(
        serial_number=dev0_serial,
        node_id=dev0_node_id,
        auto_calibrate=auto_calibrate,
        cpr=cpr,
    )
    if dev0 is None:
        return None, None

    print("Configuring dev1...")
    dev1 = configure_odrive_board(
        serial_number=dev1_serial,
        node_id=dev1_node_id,
        auto_calibrate=auto_calibrate,
        cpr=cpr,
    )
    if dev1 is None:
        return dev0, None

    return dev0, dev1


def verify_odrive_config(odrv=None):
    try:
        if odrv is None:
            odrv = find_odrive()

        print("Current ODrive Configuration:")
        print("=" * 50)

        for i in range(2):
            try:
                axis = getattr(odrv, f"axis{i}")
                print(f"\nAxis {i}:")
                print(f"  Control Mode: {axis.controller.config.control_mode}")
                print(f"  Input Mode: {axis.controller.config.input_mode}")
                print(f"  CAN Node ID: {axis.config.can.node_id}")
                print(f"  Encoder Rate: {axis.config.can.encoder_rate_ms} ms")
                print(f"  Heartbeat Rate: {axis.config.can.heartbeat_rate_ms} ms")
            except Exception:
                pass

        print("\n" + "=" * 50 + "\n")
        return True

    except Exception as e:
        print(f"Error verifying config: {e}")
        return False


def parse_args():
    parser = argparse.ArgumentParser(description="Configure ODrive boards for CAN + cyclic feedback + calibration")
    parser.add_argument("--dev0-serial", default=None, help="Serial number of dev0 ODrive")
    parser.add_argument("--dev1-serial", default=None, help="Serial number of dev1 ODrive")
    parser.add_argument("--dev0-node", type=int, default=0, help="CAN node ID for dev0 axis0")
    parser.add_argument("--dev1-node", type=int, default=1, help="CAN node ID for dev1 axis0")
    parser.add_argument("--cpr", type=int, default=16384, help="Encoder CPR (Counts Per Revolution)")
    parser.add_argument("--no-calibrate", action="store_true", help="Skip full calibration sequence")
    parser.add_argument("--single", action="store_true", help="Configure only one ODrive")
    return parser.parse_args()


if __name__ == "__main__":
    print("ODrive Configurator\n")
    args = parse_args()
    auto_calibrate = not args.no_calibrate

    if args.single:
        odrv = configure_odrive_board(
            serial_number=args.dev0_serial,
            node_id=args.dev0_node,
            auto_calibrate=auto_calibrate,
            cpr=args.cpr,
        )
        if odrv:
            verify_odrive_config(odrv)
    else:
        dev0, dev1 = configure_two_odrives(
            dev0_serial=args.dev0_serial,
            dev1_serial=args.dev1_serial,
            dev0_node_id=args.dev0_node,
            dev1_node_id=args.dev1_node,
            auto_calibrate=auto_calibrate,
            cpr=args.cpr,
        )
        if dev0:
            verify_odrive_config(dev0)
        if dev1:
            verify_odrive_config(dev1)
