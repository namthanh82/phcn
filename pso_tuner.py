"""Offline PSO tuner for Kp/Kd.

This is a minimal implementation meant to be called from the GUI hook. It can
optimize against either a simulated model response or logged experimental data
that has already been reduced to a response trace.
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Sequence

from ctc_controller import CTCGains, SimpleRigidBodyModel, ctc_control
from fitness import compute_fitness, FitnessWeights
from system_identifier import FirstOrderModel


@dataclass
class PSOResult:
    kp: float
    kd: float
    fitness: float


@dataclass
class PSOConfig:
    particles: int = 20
    iterations: int = 30
    w: float = 0.72
    c1: float = 1.49
    c2: float = 1.49


def simulate_ctc_step(
    kp: float,
    kd: float,
    setpoint: float,
    duration_s: float = 2.0,
    dt: float = 0.01,
    model: SimpleRigidBodyModel | None = None,
) -> tuple[list[float], list[float]]:
    """Very small toy simulator for tuning.

    Replace later with a more accurate plant model or logged-data replay.
    """
    plant = model or SimpleRigidBodyModel(mass_like=1.0, damping_like=0.1, gravity_like=0.0)
    gains = CTCGains(kp=kp, kd=kd)
    q = 0.0
    q_dot = 0.0
    response: list[float] = []
    control: list[float] = []

    steps = max(1, int(duration_s / dt))
    for _ in range(steps):
        u = ctc_control(setpoint, q, 0.0, q_dot, 0.0, gains, plant)
        acc = (u - plant.damping_like * q_dot - plant.gravity_like) / max(plant.mass_like, 1e-6)
        q_dot += acc * dt
        q += q_dot * dt
        response.append(q)
        control.append(u)
    return response, control


def simulate_from_identified_model(
    kp: float,
    kd: float,
    setpoint: float,
    identified: FirstOrderModel,
    duration_s: float = 2.0,
    dt: float = 0.01,
) -> tuple[list[float], list[float]]:
    """Simulate a simple first-order plant identified from logged data."""
    gains = CTCGains(kp=kp, kd=kd)
    q = 0.0
    q_dot = 0.0
    response: list[float] = []
    control: list[float] = []

    steps = max(1, int(duration_s / dt))
    plant_gain = max(abs(identified.gain), 1e-6)
    tau = max(identified.tau, 1e-3)
    for _ in range(steps):
        u = ctc_control(setpoint, q, 0.0, q_dot, 0.0, gains, SimpleRigidBodyModel(mass_like=tau, damping_like=1.0 / plant_gain))
        dq = ((plant_gain * u) - q) / tau
        q += dq * dt
        q_dot = dq
        response.append(q)
        control.append(u)
    return response, control


def optimize_kp_kd(
    setpoint: float,
    model: SimpleRigidBodyModel | None = None,
    identified: FirstOrderModel | None = None,
    bounds: tuple[tuple[float, float], tuple[float, float]] = ((0.1, 500.0), (0.1, 100.0)),
    config: PSOConfig | None = None,
    weights: FitnessWeights | None = None,
) -> PSOResult:
    cfg = config or PSOConfig()
    w = weights or FitnessWeights()

    kp_min, kp_max = bounds[0]
    kd_min, kd_max = bounds[1]

    swarm = []
    for _ in range(cfg.particles):
        kp = random.uniform(kp_min, kp_max)
        kd = random.uniform(kd_min, kd_max)
        vk = [0.0, 0.0]
        swarm.append({"x": [kp, kd], "v": vk, "pbest": [kp, kd], "pbest_f": float("inf")})

    def evaluate(x: Sequence[float]) -> float:
        if identified is not None:
            response, control = simulate_from_identified_model(x[0], x[1], setpoint, identified)
        else:
            response, control = simulate_ctc_step(x[0], x[1], setpoint, model=model)
        return compute_fitness(response, setpoint, control, dt=0.01, weights=w)

    gbest = swarm[0]["x"][:]
    gbest_f = float("inf")

    for particle in swarm:
        f = evaluate(particle["x"])
        particle["pbest_f"] = f
        particle["pbest"] = particle["x"][:]
        if f < gbest_f:
            gbest_f = f
            gbest = particle["x"][:]

    for _ in range(cfg.iterations):
        for particle in swarm:
            for i in range(2):
                r1 = random.random()
                r2 = random.random()
                particle["v"][i] = (
                    cfg.w * particle["v"][i]
                    + cfg.c1 * r1 * (particle["pbest"][i] - particle["x"][i])
                    + cfg.c2 * r2 * (gbest[i] - particle["x"][i])
                )
                particle["x"][i] += particle["v"][i]

            particle["x"][0] = max(kp_min, min(kp_max, particle["x"][0]))
            particle["x"][1] = max(kd_min, min(kd_max, particle["x"][1]))

            f = evaluate(particle["x"])
            if f < particle["pbest_f"]:
                particle["pbest_f"] = f
                particle["pbest"] = particle["x"][:]
            if f < gbest_f:
                gbest_f = f
                gbest = particle["x"][:]

    return PSOResult(kp=gbest[0], kd=gbest[1], fitness=gbest_f)
