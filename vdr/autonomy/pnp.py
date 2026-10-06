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
