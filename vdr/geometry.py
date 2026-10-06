"""Small SO(3) helpers shared by the simulator and the autonomy stack.

Pure maths only -- no simulator state lives here, so sharing it does not
break the sim/autonomy separation.

Conventions used across the project:
    * World / gate navigation frames are gravity aligned, z up.
    * Body frame is FLU: x forward, y left, z up.
    * Camera frame is OpenCV: x right, y down, z forward (optical axis).
    * R_ab maps vectors expressed in frame b into frame a  (v_a = R_ab @ v_b).
"""

from __future__ import annotations

import math

import numpy as np

_I3 = np.eye(3)


def skew(v) -> np.ndarray:
    """Return the 3x3 cross-product matrix [v]x such that skew(a) @ b == cross(a, b)."""
    return np.array(
        [[0.0, -v[2], v[1]],
         [v[2], 0.0, -v[0]],
         [-v[1], v[0], 0.0]]
    )


def vee(M: np.ndarray) -> np.ndarray:
    """Inverse of skew for a skew-symmetric matrix."""
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


def exp_so3(phi) -> np.ndarray:
    """Rotation matrix for rotation vector phi (Rodrigues' formula)."""
    x, y, z = float(phi[0]), float(phi[1]), float(phi[2])
    theta2 = x * x + y * y + z * z
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    if theta2 < 1e-12:
        return _I3 + K + 0.5 * (K @ K)
    theta = math.sqrt(theta2)
    a = math.sin(theta) / theta
    b = (1.0 - math.cos(theta)) / theta2
    return _I3 + a * K + b * (K @ K)


def log_so3(R: np.ndarray) -> np.ndarray:
    """Rotation vector of rotation matrix R (inverse of exp_so3)."""
    cos_t = (R[0, 0] + R[1, 1] + R[2, 2] - 1.0) * 0.5
    cos_t = min(1.0, max(-1.0, cos_t))
    theta = math.acos(cos_t)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if theta < 1e-6:
        return 0.5 * w
    if math.pi - theta < 1e-4:
        # Near pi: recover the axis from the symmetric part.
        B = 0.5 * (R + _I3)
        i = int(np.argmax(np.diag(B)))
        axis = B[:, i] / math.sqrt(max(B[i, i], 1e-12))
        axis /= np.linalg.norm(axis)
        if axis @ w < 0:
            axis = -axis
        return theta * axis
    return (theta / (2.0 * math.sin(theta))) * w


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def yaw_of(R: np.ndarray) -> float:
    """Heading of the body x axis projected on the horizontal plane."""
    return math.atan2(R[1, 0], R[0, 0])


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def orthonormalize(R: np.ndarray) -> np.ndarray:
    """Project a nearly-orthonormal matrix back onto SO(3)."""
    U, _, Vt = np.linalg.svd(R)
    M = U @ Vt
    if np.linalg.det(M) < 0:
        U[:, -1] *= -1
        M = U @ Vt
    return M


def rotation_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Smallest rotation R with R @ a_hat == b_hat."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    axis = np.cross(a, b)
    s = np.linalg.norm(axis)
    c = float(a @ b)
    if s < 1e-12:
        if c > 0:
            return _I3.copy()
        # 180 degrees: any axis perpendicular to a
        perp = np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(a, [0.0, 1.0, 0.0])
        perp /= np.linalg.norm(perp)
        return exp_so3(math.pi * perp)
    return exp_so3(axis / s * math.atan2(s, c))


def camera_mount(uptilt_rad: float) -> np.ndarray:
    """R_bc for a forward-looking camera pitched UP by `uptilt_rad`.

    Columns are the camera axes (x right, y down, z forward) expressed in
    the FLU body frame.
    """
    c, s = math.cos(uptilt_rad), math.sin(uptilt_rad)
    z_c = np.array([c, 0.0, s])          # optical axis, tilted up
    x_c = np.array([0.0, -1.0, 0.0])     # image right = body right (-y)
    y_c = np.cross(z_c, x_c)             # image down
    return np.column_stack([x_c, y_c, z_c])
