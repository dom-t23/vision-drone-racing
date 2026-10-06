"""FPV camera renderer: draws what the onboard pinhole camera sees.

Produces a real image (sky, ground, orange square gate frames, pixel noise).
The autonomy only ever gets these pixels -- never the projected corners.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from vdr.geometry import camera_mount
from vdr.sim.config import SimConfig

SKY_BGR = (225, 196, 160)
GROUND_BGR = (78, 112, 92)
GATE_BGR = (24, 110, 248)

_SHIFT = 4                      # fixed-point bits for sub-pixel polygon filling
_SCALE = float(1 << _SHIFT)
_Z_NEAR = 0.05


def _clip_poly_near(P: np.ndarray, zn: float = _Z_NEAR) -> np.ndarray:
    """Sutherland-Hodgman clip of a 3D polygon (Nx3, camera frame) against z >= zn."""
    z = P[:, 2]
    if np.all(z >= zn):
        return P
    if np.all(z < zn):
        return P[:0]
    out = []
    n = len(P)
    for i in range(n):
        a, b = P[i], P[(i + 1) % n]
        ina, inb = a[2] >= zn, b[2] >= zn
        if ina:
            out.append(a)
        if ina != inb:
            t = (zn - a[2]) / (b[2] - a[2])
            out.append(a + t * (b - a))
    return np.array(out) if out else P[:0]


def _clip_poly_halfplane(poly, l) -> list:
    """Clip a 2D polygon to the half-plane l . [u, v, 1] <= 0."""
    out = []
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        fa = l[0] * a[0] + l[1] * a[1] + l[2]
        fb = l[0] * b[0] + l[1] * b[1] + l[2]
        if fa <= 0:
            out.append(a)
        if (fa <= 0) != (fb <= 0):
            t = fa / (fa - fb)
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    return out


class FpvCamera:
    def __init__(self, cfg: SimConfig, rng: np.random.Generator):
        self.cfg = cfg
        self.W, self.H = cfg.image_width, cfg.image_height
        f = (self.W / 2.0) / math.tan(math.radians(cfg.hfov_deg) / 2.0)
        self.K = np.array([[f, 0.0, (self.W - 1) / 2.0],
                           [0.0, f, (self.H - 1) / 2.0],
                           [0.0, 0.0, 1.0]])
        self.K_inv = np.linalg.inv(self.K)
        self.R_bc = camera_mount(math.radians(cfg.cam_uptilt_deg))
        self.t_bc = np.array([cfg.cam_offset_x, 0.0, cfg.cam_offset_z])
        self.rng = rng
        self._noise = self._make_noise_bank(8) if cfg.pixel_noise_sigma > 0 else None
        self._gate_polys = None

    # ------------------------------------------------------------------ setup
    def _make_noise_bank(self, n: int):
        bank = []
        for _ in range(n):
            nz = self.rng.normal(0.0, self.cfg.pixel_noise_sigma, (self.H, self.W, 3))
            pos = np.clip(nz, 0, 255).astype(np.uint8)
            neg = np.clip(-nz, 0, 255).astype(np.uint8)
            bank.append((pos, neg))
        return bank

    def set_gates(self, gates) -> None:
        """Precompute outer and inner square corners of every gate in world coordinates."""
        a = self.cfg.gate_outer / 2.0
        b = self.cfg.gate_inner / 2.0
        sq = np.array([[0.0, -1.0, 1.0], [0.0, 1.0, 1.0], [0.0, 1.0, -1.0], [0.0, -1.0, -1.0]])
        polys = []
        for g in gates:
            outer = (g.R @ (sq * [0.0, a, a]).T).T + g.center
            inner = (g.R @ (sq * [0.0, b, b]).T).T + g.center
            polys.append((g.center.copy(), outer, inner))
        self._gate_polys = polys

    # ---------------------------------------------------------------- render
    def camera_pose(self, p_wb: np.ndarray, R_wb: np.ndarray):
        R_wc = R_wb @ self.R_bc
        p_wc = p_wb + R_wb @ self.t_bc
        return R_wc, p_wc

    def _project(self, Pc: np.ndarray) -> np.ndarray:
        uv = (Pc[:, :2] / Pc[:, 2:3]) * [self.K[0, 0], self.K[1, 1]] + [self.K[0, 2], self.K[1, 2]]
        return np.round(uv * _SCALE).astype(np.int32)

    def render(self, p_wb: np.ndarray, R_wb: np.ndarray) -> np.ndarray:
        R_wc, p_wc = self.camera_pose(p_wb, R_wb)
        R_cw = R_wc.T
        img = np.empty((self.H, self.W, 3), np.uint8)
        img[:] = SKY_BGR

        # Ground: pixels whose viewing ray points below the horizon.
        n_c = R_cw @ np.array([0.0, 0.0, 1.0])          # world up, in camera frame
        l = self.K_inv.T @ n_c                            # horizon line coefficients
        rect = [(0.0, 0.0), (self.W, 0.0), (self.W, self.H), (0.0, self.H)]
        ground = _clip_poly_halfplane(rect, l)
        if len(ground) >= 3 and p_wc[2] > 0:
            pts = np.round(np.array(ground) * _SCALE).astype(np.int32)
            cv2.fillPoly(img, [pts], GROUND_BGR, lineType=cv2.LINE_8, shift=_SHIFT)

        # Gates, far to near (painter's algorithm).
        if self._gate_polys:
            depth = [float((R_cw @ (c - p_wc))[2]) for c, _, _ in self._gate_polys]
            for k in np.argsort(depth)[::-1]:
                c, outer, inner = self._gate_polys[k]
                if depth[k] < -2.0:
                    continue
                oc = _clip_poly_near((R_cw @ (outer - p_wc).T).T)
                if len(oc) < 3:
                    continue
                ic = _clip_poly_near((R_cw @ (inner - p_wc).T).T)
                contours = [self._project(oc)]
                if len(ic) >= 3:
                    contours.append(self._project(ic))
                # Outer + inner contour filled together -> frame with a hole, no seams.
                cv2.fillPoly(img, contours, GATE_BGR, lineType=cv2.LINE_AA, shift=_SHIFT)

        if self._noise is not None:
            pos, neg = self._noise[int(self.rng.integers(len(self._noise)))]
            cv2.add(img, pos, dst=img)
            cv2.subtract(img, neg, dst=img)
        return img

    def project_gate_corners(self, p_wb, R_wb, gate_index: int) -> np.ndarray:
        """Ground-truth projected outer corners (for TESTS and diagnostics only --
        never passed to the autonomy)."""
        R_wc, p_wc = self.camera_pose(p_wb, R_wb)
        _, outer, _ = self._gate_polys[gate_index]
        Pc = (R_wc.T @ (outer - p_wc).T).T
        return (Pc[:, :2] / Pc[:, 2:3]) * [self.K[0, 0], self.K[1, 1]] + [self.K[0, 2], self.K[1, 2]]
