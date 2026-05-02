# TWAI Bridge Tasks

## Goal
Keep TWAI/CAN as an optional transport path while the main CTC torque logic stays in Python.

## Tasks
- Read and define the exact CAN frame format for the target motor driver.
- Confirm whether the protocol supports torque, current, speed, and position commands.
- Implement TWAI startup with `twai_start()`.
- Implement CAN transmit with `twai_transmit()`.
- Implement CAN receive with `twai_receive()`.
- Monitor alerts with `twai_read_alerts()`.
- Recover from bus-off with `twai_initiate_recovery()`.
- Add clear Python-to-ESP32 command framing.
- Add clear feedback formatting from ESP32 back to Python.

## Notes
- Keep the firmware transparent and small.
- Avoid two independent command sources at once.
- Add raw torque support only if the protocol is fully confirmed.
- If the library only supports position frames, keep TWAI as a bridge and not the torque brain.
