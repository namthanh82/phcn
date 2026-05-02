# Debugging Rules

- Verify ODrive connection before enabling control.
- Confirm the correct axis is being controlled.
- Clear ODrive errors before entering closed loop.
- Check backend compatibility with the GUI before tuning parameters.
- Test one layer at a time: ODrive -> Python backend -> GUI -> CAN/TWAI bridge.
