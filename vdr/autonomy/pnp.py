"""Gate pose from four image corners (OpenCV solvePnP, IPPE_SQUARE).

Frames
------
o  : gate *object* frame used by IPPE_SQUARE. Origin at the gate centre,
     x right / y up as seen from the camera, z pointing towards the camera.
G  : gate *navigation* frame. x = pass direction (away from the camera,
     through the gate), y = left, z = up. Gates are upright, so G is gravity
     aligned and an inertial frame for the ESKF.

Ambiguities a square gate has, and how they're resolved
--------------------------------------------------------
* 4-fold rotational symmetry: any cyclic re-labelling of the corners fits
  equally well. Resolved with gravity -- the gate's "up" axis is chosen as the
  object axis closest to world-up (from the attitude estimate or accelerometer).
* Planar pose ambiguity (two IPPE solutions, the classic "flip" for small or
  distant squares). Both solutions are returned, best reprojection error first;
  the caller can use the filter prediction to pick between them.
* Front/back: corners are ordered clockwise on screen, which makes the object
  z axis face the camera, so the pass direction is always "away from the drone".
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

# IPPE_SQUARE's required object-point order: TL, TR, BR, BL (x right, y up, z towards viewer).
_UNIT_SQUARE = np.array([[-0.5, 0.5, 0.0], [0.5, 0.5, 0.0], [0.5, -0.5, 0.0], [-0.5, -0.5, 0.0]])

# Columns: the G axes expressed in the object frame o.
#   x_G = -z_o (through the gate, away from camera), y_G = -x_o, z_G = +y_o (up)
R_OG = np.array([[0.0, -1.0, 0.0],
                 [0.0, 0.0, 1.0],
                 [-1.0, 0.0, 0.0]])

# Rotations about the object z axis by k * 90 deg.
_RZ90 = [np.array([[math.cos(k * math.pi / 2), -math.sin(k * math.pi / 2), 0.0],
                   [math.sin(k * math.pi / 2), math.cos(k * math.pi / 2), 0.0],
                   [0.0, 0.0, 1.0]]).round(12) for k in range(4)]


@dataclass
class GatePose:
    """One PnP solution: the gate object frame o expressed in the camera frame."""
    R_co: np.ndarray
    t_co: np.ndarray            # gate centre in camera frame (m)
    reproj_err: float           # RMS reprojection error (px)
    size: float                 # physical square size used (m)
    ippe: Optional["GatePose"] = None   # upright solutions: the unconstrained IPPE seed they came from

    @property
    def distance(self) -> float:
        return float(np.linalg.norm(self.t_co))

    @property
    def R_cG(self) -> np.ndarray:
        return self.R_co @ R_OG


def _align_up(R_co: np.ndarray, up_cam: np.ndarray) -> np.ndarray:
    """Re-label corners (rotate about object z by k*90 deg) so object +y is closest to up."""
    best_k, best_dot = 0, -2.0
    for k in range(4):
        d = float((R_co @ _RZ90[k])[:, 1] @ up_cam)
        if d > best_dot:
            best_k, best_dot = k, d
    return R_co @ _RZ90[best_k]


def solve_gate_pose(corners: np.ndarray, K: np.ndarray, size: float, up_cam: np.ndarray) -> list:
    """All plausible IPPE_SQUARE solutions for one detected square, best first."""
    obj = _UNIT_SQUARE * size
    img = np.ascontiguousarray(corners, dtype=np.float64).reshape(4, 1, 2)
    try:
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, img, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error:
        return []
    sols = []
    for i in range(n):
        R_co, _ = cv2.Rodrigues(rvecs[i])
        t = tvecs[i].reshape(3)
        if t[2] <= 0.0:
            continue
        if R_co[:, 2] @ t > 0.0:      # object z must face the camera
            continue
        sols.append(GatePose(_align_up(R_co, up_cam), t, float(np.ravel(errs[i])[0]), size))
    sols.sort(key=lambda s: s.reproj_err)
    return sols


def gate_tilt_deg(pose: GatePose, up_cam: np.ndarray) -> float:
    """Angle between the gate's up axis and world-up (gates are upright -> should be ~0)."""
    d = float(np.clip(pose.R_co[:, 1] @ up_cam, -1.0, 1.0))
    return math.degrees(math.acos(d))


def body_pose_in_gate(pose: GatePose, R_bc: np.ndarray, t_bc: np.ndarray):
    """Drone body pose (R_Gb, p_b) in the gate navigation frame G from one PnP solution."""
    R_Gc = pose.R_cG.T
    p_cam = -R_Gc @ pose.t_co
    R_Gb = R_Gc @ R_bc.T
    p_b = p_cam - R_Gb @ t_bc
    return R_Gb, p_b


def gate_in_frame(pose: GatePose, R_Gb: np.ndarray, p_b: np.ndarray, R_bc: np.ndarray, t_bc: np.ndarray):
    """Pose (R, t) of the detected gate's navigation frame, expressed in the frame of (R_Gb, p_b)."""
    R_bG2 = R_bc @ pose.R_cG
    t_b = t_bc + R_bc @ pose.t_co
    return R_Gb @ R_bG2, p_b + R_Gb @ t_b


def gate_position_cov(pose: GatePose, K: np.ndarray, corner_sigma: float, depth_factor: float,
                      pos_floor: float) -> np.ndarray:
    """3x3 covariance (camera frame) of the measured gate-centre position t_co.

    First-order propagation of corner noise through the pinhole model:
      across the line of sight:  sigma_px * d / f
      along the line of sight:   depth_factor * sigma_px * d^2 / (f * L)
    """
    f = 0.5 * (K[0, 0] + K[1, 1])
    d = pose.distance
    s_lat = math.hypot(corner_sigma * d / f, pos_floor)
    s_dep = max(s_lat, math.hypot(depth_factor * corner_sigma * d * d / (f * pose.size), pos_floor))
    los = pose.t_co / d
    return (s_lat ** 2) * np.eye(3) + (s_dep ** 2 - s_lat ** 2) * np.outer(los, los)


def attitude_sigma(pose: GatePose, K: np.ndarray, corner_sigma: float, att_factor: float,
                   floor_rad: float) -> float:
    """1-sigma error (rad) of the relative attitude from a single square: ~ sigma_px * d / (f * L)."""
    f = 0.5 * (K[0, 0] + K[1, 1])
    return max(floor_rad, att_factor * corner_sigma * pose.distance / (f * pose.size))


# ------------------------------------------------------------------ upright (4-DoF) PnP
def _upright_basis(up_cam: np.ndarray):
    """Orthonormal basis (a, b) of the plane perpendicular to up (camera frame)."""
    up = up_cam / np.linalg.norm(up_cam)
    ref = np.array([1.0, 0.0, 0.0]) if abs(up[0]) < 0.9 else np.array([0.0, 0.0, 1.0])
    a = ref - (ref @ up) * up
    a /= np.linalg.norm(a)
    return up, a, np.cross(up, a)


def _upright_R(psi: float, up, a, b) -> np.ndarray:
    """Object frame with +y = up and +x horizontal at angle psi in the (a, b) plane."""
    x = math.cos(psi) * a + math.sin(psi) * b
    return np.column_stack([x, up, np.cross(x, up)])


def refine_upright(pose: GatePose, corners: np.ndarray, K: np.ndarray, up_cam: np.ndarray,
                   iters: int = 10, use: Optional[np.ndarray] = None) -> GatePose:
    """Re-solve a square's pose with the gate constrained upright (4 DoF: position + yaw).

    Gates are vertical, so with roll/pitch known (the filter's gravity direction, `up_cam`)
    only the gate's heading and position are unknown. Gauss-Newton on the 8 reprojection
    residuals, seeded from an (up-aligned) IPPE solution. The 2 rotational DoF a single small
    square constrains worst -- and the source of the IPPE "flip" -- are taken from gravity.

    `use` (4 bools, default all) fits only those corners (3 corners still fix the 4 DoF);
    the reported reprojection error is over the used corners only.
    """
    up, a, b = _upright_basis(up_cam)
    obj = _UNIT_SQUARE * pose.size
    uv = np.asarray(corners, float).reshape(4, 2)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    def project(R, t, P):
        X = (R @ P.T).T + t
        return np.column_stack([fx * X[:, 0] / X[:, 2] + cx, fy * X[:, 1] / X[:, 2] + cy]), X

    # Correspondence: the aligned pose re-labelled the corners by a multiple of 90 deg;
    # pick the cyclic labelling of the object square that matches the image best.
    best = None
    for k in range(4):
        P = np.roll(obj, k, axis=0)
        e = float(np.sum((project(pose.R_co, pose.t_co, P)[0] - uv) ** 2))
        if best is None or e < best[0]:
            best = (e, P)
    P = best[1]

    xo = pose.R_co[:, 0]
    psi = math.atan2(float(xo @ b), float(xo @ a))
    t = pose.t_co.astype(float).copy()
    for _ in range(iters):
        R = _upright_R(psi, up, a, b)
        pr, X = project(R, t, P)
        r = (pr - uv).reshape(-1)
        dx = -math.sin(psi) * a + math.cos(psi) * b
        dR = np.column_stack([dx, np.zeros(3), np.cross(dx, up)])
        dX_dpsi = (dR @ P.T).T
        iz = 1.0 / X[:, 2]
        J = np.zeros((4, 2, 4))
        J[:, 0, 1] = fx * iz
        J[:, 0, 3] = -fx * X[:, 0] * iz * iz
        J[:, 1, 2] = fy * iz
        J[:, 1, 3] = -fy * X[:, 1] * iz * iz
        J[:, :, 0] = np.einsum("nij,nj->ni", J[:, :, 1:4], dX_dpsi)
        J = J.reshape(8, 4)
        if use is not None:
            m = np.repeat(np.asarray(use, bool), 2)
            J, r = J[m], r[m]
        step = np.linalg.solve(J.T @ J + 1e-9 * np.eye(4), -J.T @ r)
        psi += float(step[0])
        t = t + step[1:]
        if t[2] <= 0.05:
            return pose                 # diverged behind the camera: keep the unconstrained fit
        if abs(step[0]) < 1e-6 and np.linalg.norm(step[1:]) < 1e-5:
            break
    R = _upright_R(psi, up, a, b)
    pr, _ = project(R, t, P)
    d2 = np.sum((pr - uv) ** 2, axis=1)
    if use is not None:
        d2 = d2[np.asarray(use, bool)]
    err = float(np.sqrt(np.mean(d2)))
    return GatePose(R, t, err, pose.size, ippe=pose)


def upright_solutions(sols: list, corners: np.ndarray, K: np.ndarray, up_cam: np.ndarray) -> list:
    """Upright-refine every IPPE solution; merge those that converge to the same pose."""
    out = []
    for s in sols:
        u = refine_upright(s, corners, K, up_cam)
        if u.R_co[:, 2] @ u.t_co > 0.0:
            continue                    # converged to a gate facing away from the camera
        dup = False
        for o in out:
            if (np.linalg.norm(o.t_co - u.t_co) < 1e-3 * max(o.distance, 1.0)
                    and float(o.R_co[:, 0] @ u.R_co[:, 0]) > math.cos(math.radians(0.5))):
                dup = True
                break
        if not dup:
            out.append(u)
    out.sort(key=lambda s: s.reproj_err)
    return out


def refine_upright_squares(pose: GatePose, squares: list, K: np.ndarray, up_cam: np.ndarray,
                           iters: int = 10) -> GatePose:
    """Upright (4-DoF) fit to several concentric, coplanar squares of one gate at once.

    `squares` is a list of (corners, size): typically the outer outline and the inner opening
    of the same gate (8 points instead of 4). Seeded from `pose` (an upright or IPPE solution
    of one of them); each square's corner labelling is the cyclic one that best matches the
    seed's projection. The returned pose keeps `pose.size` and `pose.ippe` so the measurement
    model and the tilt source are unchanged; only the fit itself uses more points.
    """
    up, a, b = _upright_basis(up_cam)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    def project(R, t, P):
        X = (R @ P.T).T + t
        return np.column_stack([fx * X[:, 0] / X[:, 2] + cx, fy * X[:, 1] / X[:, 2] + cy]), X

    Ps, uvs = [], []
    for corners, size in squares:
        uv = np.asarray(corners, float).reshape(4, 2)
        obj = _UNIT_SQUARE * size
        best = None
        for k in range(4):
            P = np.roll(obj, k, axis=0)
            e = float(np.sum((project(pose.R_co, pose.t_co, P)[0] - uv) ** 2))
            if best is None or e < best[0]:
                best = (e, P)
        Ps.append(best[1])
        uvs.append(uv)
    P = np.vstack(Ps)
    uv = np.vstack(uvs)
    n = P.shape[0]

    xo = pose.R_co[:, 0]
    psi = math.atan2(float(xo @ b), float(xo @ a))
    t = pose.t_co.astype(float).copy()
    for _ in range(iters):
        R = _upright_R(psi, up, a, b)
        pr, X = project(R, t, P)
        r = (pr - uv).reshape(-1)
        dx = -math.sin(psi) * a + math.cos(psi) * b
        dR = np.column_stack([dx, np.zeros(3), np.cross(dx, up)])
        dX_dpsi = (dR @ P.T).T
        iz = 1.0 / X[:, 2]
        J = np.zeros((n, 2, 4))
        J[:, 0, 1] = fx * iz
        J[:, 0, 3] = -fx * X[:, 0] * iz * iz
        J[:, 1, 2] = fy * iz
        J[:, 1, 3] = -fy * X[:, 1] * iz * iz
        J[:, :, 0] = np.einsum("nij,nj->ni", J[:, :, 1:4], dX_dpsi)
        J = J.reshape(2 * n, 4)
        step = np.linalg.solve(J.T @ J + 1e-9 * np.eye(4), -J.T @ r)
        psi += float(step[0])
        t = t + step[1:]
        if t[2] <= 0.05:
            return pose
        if abs(step[0]) < 1e-6 and np.linalg.norm(step[1:]) < 1e-5:
            break
    R = _upright_R(psi, up, a, b)
    pr, _ = project(R, t, P)
    err = float(np.sqrt(np.mean(np.sum((pr - uv) ** 2, axis=1))))
    return GatePose(R, t, err, pose.size, ippe=pose.ippe if pose.ippe is not None else pose)
