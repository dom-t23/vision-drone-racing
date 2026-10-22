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


def _offset_convex(P: np.ndarray, delta: float) -> np.ndarray:
    """Move every edge of a convex polygon inward by `delta` (outward if negative)."""
    n = len(P)
    if n < 3:
        return P
    c = P.mean(axis=0)
    lines = []
    for i in range(n):
        a, b = P[i], P[(i + 1) % n]
        d = b - a
        L = math.hypot(d[0], d[1])
        if L < 1e-9:
            return P
        nrm = np.array([-d[1], d[0]]) / L
        if (c - a) @ nrm < 0:
            nrm = -nrm                     # inward normal
        lines.append((nrm, float(nrm @ a) + delta))
    out = []
    for i in range(n):
        (n1, c1), (n2, c2) = lines[i - 1], lines[i]
        det = n1[0] * n2[1] - n1[1] * n2[0]
        if abs(det) < 1e-9:
            return P
        out.append(np.linalg.solve(np.array([n1, n2]), [c1, c2]))
    out = np.array(out)
    if np.max(np.linalg.norm(out - P, axis=1)) > 4.0 * abs(delta) + 1e-6:
        return P                           # sliver / spike: leave it
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
        self._noise = (self._make_noise_bank(8) if cfg.pixel_noise_sigma > 0 and not cfg.pixel_noise_fresh
                       else None)
        self._gate_polys = None

    # ------------------------------------------------------------------ setup
    def _make_noise(self):
        """One frame of additive Gaussian noise, split into (positive, negative) uint8 parts."""
        nz = self.rng.normal(0.0, self.cfg.pixel_noise_sigma, (self.H, self.W, 3))
        return np.clip(nz, 0, 255).astype(np.uint8), np.clip(-nz, 0, 255).astype(np.uint8)

    def _make_noise_bank(self, n: int):
        return [self._make_noise() for _ in range(n)]

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

    def _project_uv(self, Pc: np.ndarray) -> np.ndarray:
        return (Pc[:, :2] / Pc[:, 2:3]) * [self.K[0, 0], self.K[1, 1]] + [self.K[0, 2], self.K[1, 2]]

    def _project(self, Pc: np.ndarray) -> np.ndarray:
        return np.round(self._project_uv(Pc) * _SCALE).astype(np.int32)

    def _fill_coverage(self, img: np.ndarray, polys_uv: list, colour) -> None:
        """Fill polygons (even-odd, so an inner contour is a hole) with area-coverage anti-aliasing.

        cv2's LINE_AA polygon fill paints roughly 0.7 px beyond the true edge on
        every side, so painted squares look too big and their openings too small
        -- a range bias of several percent at distance. Here the gate is
        rasterised into an N x N supersampled mask, box-filtered down to a per-pixel
        coverage fraction and alpha-blended over the image, like a camera
        pixel integrating light over its area. cv2's fill also paints sub-pixels
        within ~half a sub-pixel outside the polygon, so the outline is shrunk
        and the opening grown by that much first.
        """
        N = self.cfg.gate_supersample
        allp = np.vstack(polys_uv)
        u0 = int(max(0, math.floor(allp[:, 0].min()) - 1))
        v0 = int(max(0, math.floor(allp[:, 1].min()) - 1))
        u1 = int(min(self.W, math.ceil(allp[:, 0].max()) + 2))
        v1 = int(min(self.H, math.ceil(allp[:, 1].max()) + 2))
        if u1 <= u0 or v1 <= v0:
            return
        w, h = u1 - u0, v1 - v0
        mask = np.zeros((h * N, w * N), np.uint8)
        # native pixel centre u -> supersampled centre N * (u - u0 + 0.5) - 0.5
        pts = []
        for k, P in enumerate(polys_uv):
            Q = _offset_convex(N * (P - [u0, v0] + 0.5) - 0.5, 0.5 if k == 0 else -0.5)
            pts.append(np.round(Q * _SCALE).astype(np.int32))
        cv2.fillPoly(mask, pts, 255, lineType=cv2.LINE_8, shift=_SHIFT)
        a = cv2.resize(mask, (w, h), interpolation=cv2.INTER_AREA).astype(np.float32) * (1.0 / 255.0)
        roi = img[v0:v1, u0:u1].astype(np.float32)
        roi += a[:, :, None] * (np.array(colour, np.float32) - roi)
        img[v0:v1, u0:u1] = np.clip(np.round(roi), 0, 255).astype(np.uint8)

    def render(self, p_wb: np.ndarray, R_wb: np.ndarray) -> np.ndarray:
        img = self._render_scene(p_wb, R_wb)
        self._add_noise(img)
        return img

    def render_exposure(self, poses) -> np.ndarray:
        """Motion-blurred frame: the mean of noise-free renders at each (p_wb, R_wb) pose
        sampled over the shutter interval, then one frame of sensor noise."""
        acc = np.zeros((self.H, self.W, 3), np.float32)
        for p_wb, R_wb in poses:
            acc += self._render_scene(p_wb, R_wb)
        img = np.clip(np.round(acc * (1.0 / len(poses))), 0, 255).astype(np.uint8)
        self._add_noise(img)
        return img

    def _render_scene(self, p_wb: np.ndarray, R_wb: np.ndarray) -> np.ndarray:
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
                if self.cfg.gate_supersample >= 2:
                    polys = [self._project_uv(oc)]
                    if len(ic) >= 3:
                        polys.append(self._project_uv(ic))
                    self._fill_coverage(img, polys, GATE_BGR)
                    continue
                contours = [self._project(oc)]
                if len(ic) >= 3:
                    contours.append(self._project(ic))
                # Outer + inner contour filled together -> frame with a hole, no seams.
                cv2.fillPoly(img, contours, GATE_BGR, lineType=cv2.LINE_AA, shift=_SHIFT)
        return img

    def _add_noise(self, img: np.ndarray) -> None:
        if self._noise is not None:
            pos, neg = self._noise[int(self.rng.integers(len(self._noise)))]
        elif self.cfg.pixel_noise_sigma > 0:
            pos, neg = self._make_noise()
        else:
            pos = neg = None
        if pos is not None:
            cv2.add(img, pos, dst=img)
            cv2.subtract(img, neg, dst=img)

    def project_gate_corners(self, p_wb, R_wb, gate_index: int) -> np.ndarray:
        """Ground-truth projected outer corners (for TESTS and diagnostics only --
        never passed to the autonomy)."""
        R_wc, p_wc = self.camera_pose(p_wb, R_wb)
        _, outer, _ = self._gate_polys[gate_index]
        Pc = (R_wc.T @ (outer - p_wc).T).T
        return (Pc[:, :2] / Pc[:, 2:3]) * [self.K[0, 0], self.K[1, 1]] + [self.K[0, 2], self.K[1, 2]]
