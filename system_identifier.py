"""Very small system-identification helpers.

The first version estimates a first-order equivalent from logged step-response
data, enough to seed the PSO tuner.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass
class FirstOrderModel:
    gain: float
    tau: float


def estimate_first_order_model(time_s: Sequence[float], response: Sequence[float], setpoint: float) -> FirstOrderModel:
    if not time_s or not response:
        return FirstOrderModel(gain=1.0, tau=1.0)

    y0 = response[0]
    yss = response[-1]
    delta_u = setpoint if setpoint != 0 else 1.0
    gain = (yss - y0) / delta_u if delta_u else 1.0

    target = y0 + 0.632 * (yss - y0)
    tau = 1.0
    for t, y in zip(time_s, response):
        if (yss >= y0 and y >= target) or (yss < y0 and y <= target):
            tau = max(t - time_s[0], 1e-3)
            break

    return FirstOrderModel(gain=gain, tau=tau)
