"""Computed Torque Control helper.

This module provides a simple discrete-friendly CTC form for PSO tuning of
Kp/Kd before deploying to the real motor system.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class CTCGains:
    kp: float
    kd: float


@dataclass
class SimpleRigidBodyModel:
    mass_like: float = 1.0
    damping_like: float = 0.0
    gravity_like: float = 0.0


def ctc_control(
    qd: float,
    q: float,
    qd_dot: float,
    q_dot: float,
    qd_ddot: float,
    gains: CTCGains,
    model: SimpleRigidBodyModel | None = None,
) -> float:
    """Return a control action using a simplified computed torque form."""
    m = model or SimpleRigidBodyModel()
    e = qd - q
    de = qd_dot - q_dot
    v = qd_ddot + gains.kd * de + gains.kp * e
    return m.mass_like * v + m.damping_like * q_dot + m.gravity_like
