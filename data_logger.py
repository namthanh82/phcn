"""Utilities to collect step-response logs from the GUI controller.

The first version is minimal and returns a snapshot of the current buffered
trajectory, which can then be fed into system identification or PSO tuning.
"""

from __future__ import annotations

from typing import Any


def snapshot_from_controller(ctrl: Any) -> dict[str, list[float]]:
    data = list(ctrl.get_data()) if ctrl and hasattr(ctrl, "get_data") else []
    return {
        "time": [row[0] for row in data],
        "pos0": [row[1] for row in data],
        "pos1": [row[2] for row in data],
        "set0": [row[3] for row in data],
        "set1": [row[4] for row in data],
    }
