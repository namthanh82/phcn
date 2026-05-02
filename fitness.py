"""Fitness utilities for CTC + PSO tuning.

These helpers are intentionally lightweight so they can be used with either
simulated data or logged experimental responses from the 2-motor system.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass
class FitnessWeights:
    itae: float = 0.45
    iae: float = 0.25
    overshoot: float = 0.20
    control_rms: float = 0.10
    saturation: float = 1.0


def _mean_square(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return sum(v * v for v in values) / len(values)


def compute_iae(error: Sequence[float], dt: float) -> float:
    return sum(abs(e) * dt for e in error)


def compute_itae(error: Sequence[float], dt: float) -> float:
    return sum(i * dt * abs(e) * dt for i, e in enumerate(error))


def compute_overshoot(response: Sequence[float], setpoint: float) -> float:
    if not response or setpoint == 0:
        return 0.0
    peak = max(response)
    return max(0.0, (peak - setpoint) / abs(setpoint))


def compute_control_rms(control_signal: Sequence[float]) -> float:
    return _mean_square(control_signal) ** 0.5


def compute_penalty_saturation(control_signal: Sequence[float], u_max: float) -> float:
    if not control_signal or u_max <= 0:
        return 0.0
    return sum(1.0 for u in control_signal if abs(u) > u_max) / len(control_signal)


def compute_fitness(
    response: Sequence[float],
    setpoint: float,
    control_signal: Sequence[float],
    dt: float,
    weights: FitnessWeights | None = None,
    u_max: float | None = None,
) -> float:
    """Compute a scalar objective J for PSO.

    Lower is better.
    """
    w = weights or FitnessWeights()
    error = [setpoint - y for y in response]
    iae = compute_iae(error, dt)
    itae = compute_itae(error, dt)
    overshoot = compute_overshoot(response, setpoint)
    urms = compute_control_rms(control_signal)
    sat = compute_penalty_saturation(control_signal, u_max or 0.0) if u_max else 0.0

    return (
        w.itae * itae
        + w.iae * iae
        + w.overshoot * overshoot
        + w.control_rms * urms
        + w.saturation * sat
    )
