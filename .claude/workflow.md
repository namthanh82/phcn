# Workflow

1. Verify hardware and ODrive state.
2. Run Python backend.
3. Connect GUI.
4. Test offset and closed-loop enable.
5. Test desired motion with low torque limits first.
6. Confirm one axis works reliably before extending to the second axis.
7. Expand to dual-axis or CAN/TWAI bridge only after single-axis works.
8. Use the TWAI bridge only after the protocol is confirmed and the command path is unambiguous.
