# TODO — Project Work Plan

## Goal
Build a stable motor control system that uses true CTC torque control for desired-motion tracking, with CAN/TWAI support where appropriate.

## Current Focus
### 1. Make the Python CTC torque backend stable
- Keep `trajectory_controller.py` as the primary CTC controller.
- Ensure the ODrive backend connects reliably.
- Confirm the correct axis is selected.
- Keep `control_mode = torque control`.
- Keep `input_mode = passthrough`.
- Make GUI/backend APIs consistent for single-axis control first.

### 2. Clean up the GUI flow
- Make the GUI clearly show torque/CTC mode.
- Disable controls that are not valid for the active backend.
- Fix any button logic that assumes dual-axis behavior.
- Keep PSO/tuning tools compatible with the active backend.

### 3. Decide how TWAI will be used
- Determine whether TWAI is only a bridge or part of the active control path.
- If used as a bridge, keep it transparent and simple.
- Avoid running two independent command sources at once.

## TWAI Bridge Tasks
### Option A — Raw TWAI transport for ODrive or motor protocol
- Read and define the exact CAN frame format.
- Implement `twai_start()` initialization.
- Implement transmit with `twai_transmit()`.
- Implement receive with `twai_receive()`.
- Handle alerts with `twai_read_alerts()`.
- Recover from bus-off with `twai_initiate_recovery()`.

### Option B — Keep ESP32 only as a simple bridge
- Forward commands from Python to CAN.
- Return feedback from CAN to Python.
- Keep the firmware as small as possible.

## Later Expansion
- Extend single-axis control to dual-axis control.
- Add synchronized motion commands.
- Add trajectory presets in the GUI.
- Add raw TWAI torque support only if the protocol is fully confirmed.

## Safety Checks
- Verify hardware wiring first.
- Confirm ODrive calibration.
- Clear errors before control.
- Test one layer at a time.
- Use low torque limits for first motion tests.
