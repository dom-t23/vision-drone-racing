import math

import numpy as np

from vdr.geometry import (camera_mount, exp_so3, log_so3, orthonormalize, rot_x, rot_y, rot_z,
                          rotation_between, skew, vee, wrap_angle, yaw_of)


def test_exp_log_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(200):
        phi = rng.normal(size=3) * rng.uniform(0, 3.0)
        R = exp_so3(phi)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-10)
        assert np.isclose(np.linalg.det(R), 1.0)
        assert np.allclose(exp_so3(log_so3(R)), R, atol=1e-8)


def test_log_near_pi():
    axis = np.array([1.0, 2.0, -0.5])
    axis /= np.linalg.norm(axis)
    R = exp_so3(axis * (math.pi - 1e-6))
    assert np.allclose(exp_so3(log_so3(R)), R, atol=1e-6)


def test_skew_vee():
    v = np.array([0.3, -1.2, 2.0])
    assert np.allclose(vee(skew(v)), v)
    assert np.allclose(skew(v) @ np.array([1.0, 2.0, 3.0]), np.cross(v, [1.0, 2.0, 3.0]))


def test_elementary_rotations_and_yaw():
    assert np.allclose(rot_z(math.pi / 2) @ [1, 0, 0], [0, 1, 0])
    assert np.allclose(rot_y(math.pi / 2) @ [0, 0, 1], [1, 0, 0])
    assert np.allclose(rot_x(math.pi / 2) @ [0, 1, 0], [0, 0, 1])
    assert math.isclose(yaw_of(rot_z(0.7) @ rot_y(0.2)), 0.7, abs_tol=1e-9)
    assert math.isclose(wrap_angle(3 * math.pi), math.pi, abs_tol=1e-9) or math.isclose(
        wrap_angle(3 * math.pi), -math.pi, abs_tol=1e-9)


def test_rotation_between():
    rng = np.random.default_rng(1)
    for _ in range(50):
        a, b = rng.normal(size=3), rng.normal(size=3)
        R = rotation_between(a, b)
        assert np.allclose(R @ (a / np.linalg.norm(a)), b / np.linalg.norm(b), atol=1e-9)
    a = np.array([0.0, 0.0, 1.0])
    assert np.allclose(rotation_between(a, -a) @ a, -a, atol=1e-9)


def test_orthonormalize():
    R = exp_so3([0.1, 0.2, 0.3]) + 1e-3
    M = orthonormalize(R)
    assert np.allclose(M @ M.T, np.eye(3), atol=1e-12)


def test_camera_mount_points_forward_and_up():
    tilt = math.radians(20)
    R_bc = camera_mount(tilt)
    assert np.allclose(R_bc @ R_bc.T, np.eye(3))
    assert np.isclose(np.linalg.det(R_bc), 1.0)
    optical = R_bc @ [0, 0, 1]                      # camera z in body frame
    assert np.allclose(optical, [math.cos(tilt), 0, math.sin(tilt)])
    assert np.allclose(R_bc @ [1, 0, 0], [0, -1, 0])   # image right = body right
