# Design Rules

- Keep the control architecture simple and explicit.
- Prefer Python for control math and trajectory logic.
- Prefer ODrive direct torque control for true CTC.
- Use TWAI/CAN only as transport/bridge unless a raw torque protocol is implemented.
- Avoid two active command sources at the same time.
- Stabilize single-axis control before adding dual-axis logic.
- Keep the GUI backend API consistent with the active controller.
- Prefer low-risk changes and clear separation of roles: UI, control math, transport, actuator.
