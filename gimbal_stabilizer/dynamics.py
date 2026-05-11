#!/usr/bin/env python3
"""SIYI A8 mini gimbal & zoom dynamics — single-env deployment port.

Pure-Python single-env port of the iris_ma6 training-time controllers:
  - GimbalRateLoop ← IsaacLab/.../iris_ma6/controller/gimbal_rate_loop.py
  - ZoomController(model=...) ← .../zoom_controller.py

Inputs are PHYSICAL units (rad/s for gimbal, levels/s for zoom). The
training-time Stage-1 action-denorm (cmd × max_rate) is NOT included —
any [-1, 1] normalization is the upstream caller's responsibility.

Architectural invariant (mas/035): the gimbal rate loop applies ONLY to
the user-LOS-rate command path. Body-motion compensation downstream
bypasses this loop and remains instantaneous.

Zoom model parity:
  first_order:  integrator + first-order lag                    (no quant)
  siyi_a8:      slew-clip + dead-time + integrator + lag + quant (mas/037)
Only siyi_a8 publishes a quantized output; the continuous internal state
is preserved in both modes so sub-quantum momentum survives across steps.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Tuple


# ───────────────────────── Gimbal rate loop ──────────────────────────

@dataclass
class GimbalRateLoopCfg:
    """Single-env gimbal rate-loop config.

    Defaults: rate_model.json fit (mas/035) and gimbal_dead_time_fit.json
    (mas/036). When ``enabled=False`` the loop is pass-through.
    """
    enabled: bool = True
    tau_yaw_s: float = 0.0995
    tau_pitch_s: float = 0.0954
    max_rate_per_axis: float = 1.28
    deadband_rad_s: float = 0.0
    dead_time_s: float = 0.0


class GimbalRateLoop:
    """Per-axis first-order rate loop with saturation/deadband/dead-time.

    Pipeline (when enabled), applied in order on each ``step`` call:
      1. Hard saturation at ±max_rate_per_axis
      2. Symmetric deadband (skipped if deadband_rad_s == 0)
      3. Input dead-time ring buffer (skipped if dead_time_s == 0)
      4. First-order lag toward (saturated, deadbanded, delayed) command
    """

    def __init__(self, cfg: GimbalRateLoopCfg):
        self.cfg = cfg
        self._omega_yaw = 0.0
        self._omega_pitch = 0.0
        # Lazy-allocated dead-time ring; depth = round(dead_time_s / dt) + 1
        # so that buf[0] after append == value pushed d steps ago.
        self._dt: float | None = None
        self._d: int = 0
        self._buf: Deque[Tuple[float, float]] | None = None

    @property
    def omega_yaw(self) -> float:
        return self._omega_yaw

    @property
    def omega_pitch(self) -> float:
        return self._omega_pitch

    def step(self, az_cmd: float, el_cmd: float, dt: float) -> Tuple[float, float]:
        """Advance the rate loop one step. Returns (yaw_rate, pitch_rate)."""
        if not self.cfg.enabled:
            self._omega_yaw = float(az_cmd)
            self._omega_pitch = float(el_cmd)
            return self._omega_yaw, self._omega_pitch

        # 1) Hard saturation per axis
        m = self.cfg.max_rate_per_axis
        az = max(-m, min(m, float(az_cmd)))
        el = max(-m, min(m, float(el_cmd)))

        # 2) Optional symmetric deadband
        if self.cfg.deadband_rad_s > 0.0:
            if abs(az) < self.cfg.deadband_rad_s:
                az = 0.0
            if abs(el) < self.cfg.deadband_rad_s:
                el = 0.0

        # 3) Input dead-time buffer
        if self.cfg.dead_time_s > 0.0:
            az, el = self._apply_dead_time(az, el, dt)

        # 4) Per-axis first-order lag toward the (delayed) command.
        # eps guard: τ ≈ 0 collapses to pass-through.
        eps = 1e-6
        alpha_y = (1.0 if self.cfg.tau_yaw_s < eps
                   else 1.0 - math.exp(-dt / max(self.cfg.tau_yaw_s, eps)))
        alpha_p = (1.0 if self.cfg.tau_pitch_s < eps
                   else 1.0 - math.exp(-dt / max(self.cfg.tau_pitch_s, eps)))
        self._omega_yaw += alpha_y * (az - self._omega_yaw)
        self._omega_pitch += alpha_p * (el - self._omega_pitch)
        return self._omega_yaw, self._omega_pitch

    def reset(self) -> None:
        self._omega_yaw = 0.0
        self._omega_pitch = 0.0
        if self._buf is not None:
            self._buf.clear()

    def _apply_dead_time(
        self, az: float, el: float, dt: float
    ) -> Tuple[float, float]:
        # (Re-)allocate ring on first call or if dt changed.
        if self._buf is None or self._dt != dt:
            self._dt = dt
            self._d = max(0, int(round(self.cfg.dead_time_s / max(dt, 1e-9))))
            if self._d == 0:
                self._buf = None
                return az, el
            self._buf = deque(maxlen=self._d + 1)
        if self._d == 0 or self._buf is None:
            return az, el
        self._buf.append((az, el))
        # Cold-start: while fewer than d+1 entries have been pushed, the
        # delayed slot has never seen a real command — output 0 (matches
        # training-time hardware-cold-start behavior).
        if len(self._buf) <= self._d:
            return 0.0, 0.0
        return self._buf[0]


# ──────────────────────────── Zoom dynamics ──────────────────────────

@dataclass
class ZoomDynamicsCfg:
    """Single-env zoom dynamics config.

    ``model`` selects between two training-parity variants:
      - "first_order": integrator + first-order lag only.
        Reads: tau_zoom_s, zoom_min, zoom_max.
      - "siyi_a8":     slew-clip + dead-time + integrator + lag + quantize.
        Reads: all fields.

    Input is physical levels/s (no action-denorm stage).
    """
    model: str = "siyi_a8"
    tau_zoom_s: float = 0.091
    zoom_min: float = 1.0
    zoom_max: float = 5.0
    v_max_levels_per_s: float = 3.16
    quantum_levels: float = 0.1
    dead_time_s: float = 0.0


class ZoomDynamics:
    """Selectable zoom dynamics with continuous internal state.

    Pipeline:
      [siyi_a8 only]  1. Lens slew clip (±v_max_levels_per_s)
      [siyi_a8 only]  2. Input dead-time buffer
                      3. Integrator: target ← clamp(target + rate·dt, [min, max])
                      4. First-order lag (τ_zoom)
                      5. Defensive post-lag clamp
      [siyi_a8 only]  6. Output quantization (on PUBLISHED path only —
                          internal stays continuous)
    """

    def __init__(self, cfg: ZoomDynamicsCfg):
        if cfg.model not in ("first_order", "siyi_a8"):
            raise ValueError(
                f"ZoomDynamicsCfg.model must be 'first_order' or 'siyi_a8', "
                f"got {cfg.model!r}")
        self.cfg = cfg
        # Match training: initialize at 1.0 (the canonical "no zoom" state).
        self._zoom_target: float = 1.0
        self._zoom: float = 1.0  # continuous post-lag state
        # Lazy-allocated dead-time ring (siyi_a8 only).
        self._dt: float | None = None
        self._d: int = 0
        self._buf: Deque[float] | None = None

    @property
    def zoom_internal(self) -> float:
        """Continuous post-lag zoom state (same in both modes)."""
        return self._zoom

    @property
    def zoom_published(self) -> float:
        """Env-facing zoom level. Quantized only in siyi_a8 mode."""
        if self.cfg.model == "siyi_a8":
            q = self.cfg.quantum_levels
            return round(self._zoom / q) * q
        return self._zoom

    def step(self, rate_levels_per_s: float, dt: float) -> float:
        """Advance one step; return the published zoom level."""
        rate = float(rate_levels_per_s)

        if self.cfg.model == "siyi_a8":
            # Stage 2: symmetric lens slew clip
            v = self.cfg.v_max_levels_per_s
            rate = max(-v, min(v, rate))
            # Stage 3: input dead-time buffer
            if self.cfg.dead_time_s > 0.0:
                rate = self._apply_dead_time(rate, dt)

        # Stage 4: integrator on the continuous target
        self._zoom_target += rate * dt
        self._zoom_target = max(
            self.cfg.zoom_min, min(self.cfg.zoom_max, self._zoom_target))

        # Stage 5: first-order lag toward the target
        eps = 1e-6
        alpha = (1.0 if self.cfg.tau_zoom_s < eps
                 else 1.0 - math.exp(-dt / max(self.cfg.tau_zoom_s, eps)))
        self._zoom += alpha * (self._zoom_target - self._zoom)

        # Stage 6: defensive post-lag clamp (matches training)
        self._zoom = max(self.cfg.zoom_min, min(self.cfg.zoom_max, self._zoom))

        return self.zoom_published

    def set_zoom(self, level: float) -> None:
        """Hard-set zoom (transient-free). Writes both target and post-lag state."""
        z = max(self.cfg.zoom_min, min(self.cfg.zoom_max, float(level)))
        self._zoom_target = z
        self._zoom = z

    def reset(self) -> None:
        self._zoom_target = 1.0
        self._zoom = 1.0
        if self._buf is not None:
            self._buf.clear()

    def _apply_dead_time(self, rate: float, dt: float) -> float:
        if self._buf is None or self._dt != dt:
            self._dt = dt
            self._d = max(0, int(round(self.cfg.dead_time_s / max(dt, 1e-9))))
            if self._d == 0:
                self._buf = None
                return rate
            self._buf = deque(maxlen=self._d + 1)
        if self._d == 0 or self._buf is None:
            return rate
        self._buf.append(rate)
        if len(self._buf) <= self._d:
            return 0.0
        return self._buf[0]
