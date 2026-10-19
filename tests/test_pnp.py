import math

import cv2
import numpy as np

from vdr.autonomy import pnp
from vdr.autonomy.detector import order_clockwise
from vdr.geometry import camera_mount, exp_so3, log_so3, rot_x, rot_y, rot_z

K = np.array([[134.3, 0.0, 159.5], [0.0, 134.3, 119.5], [0.0, 0.0, 1.0]])
R_BC = camera_mount(math.radians(20))
T_BC = np.array([0.08, 0.0, 0.02])
L = 1.7


def _scene(rng):
    """Random upright gate in front of a randomly attitude'd drone; returns truth + image corners."""
    d = rng.uniform(2.0, 9.0)
    p_G = np.array([-d, rng.uniform(-1.5, 1.5), rng.uniform(-0.8, 0.8)])     # drone in gate frame
    yaw = math.atan2(-p_G[1], -p_G[0]) + rng.uniform(-0.25, 0.25)
    R_Gb = rot_z(yaw) @ rot_y(rng.uniform(-0.1, 0.45)) @ rot_x(rng.uniform(-0.35, 0.35))
    # gate corners in G (x = 0 plane), project through the camera
    sq = np.array([[0, -1, 1], [0, 1, 1], [0, 1, -1], [0, -1, -1]], float) * [0, L / 2, L / 2]
    R_Gc = R_Gb @ R_BC
    p_c = p_G + R_Gb @ T_BC
    Pc = (R_Gc.T @ (sq - p_c).T).T
    uv = (Pc[:, :2] / Pc[:, 2:3]) * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
    t_true = R_Gc.T @ (np.zeros(3) - p_c)
    up_cam = R_BC.T @ R_Gb.T @ np.array([0.0, 0.0, 1.0])
    return p_G, R_Gb, uv, t_true, up_cam


def test_exact_corners_recover_drone_pose():
    rng = np.random.default_rng(0)
    for _ in range(100):
        p_G, R_Gb, uv, t_true, up_cam = _scene(rng)
        sols = pnp.solve_gate_pose(order_clockwise(uv), K, L, up_cam)
        assert sols, "no solution"
        s = sols[0]
        assert np.allclose(s.t_co, t_true, atol=1e-6)
        R_est, p_est = pnp.body_pose_in_gate(s, R_BC, T_BC)
        assert np.linalg.norm(p_est - p_G) < 1e-5
        assert np.linalg.norm(log_so3(R_est.T @ R_Gb)) < 1e-6


def test_corner_labelling_is_resolved_by_gravity():
    """All four cyclic labellings of the square give the same pose once 'up' is known."""
    rng = np.random.default_rng(1)
    p_G, R_Gb, uv, _, up_cam = _scene(rng)
    q = order_clockwise(uv)
    ref = None
    for k in range(4):
        s = pnp.solve_gate_pose(np.roll(q, k, axis=0), K, L, up_cam)[0]
        R, p = pnp.body_pose_in_gate(s, R_BC, T_BC)
        if ref is None:
            ref = (R, p)
        assert np.allclose(R, ref[0], atol=1e-6) and np.allclose(p, ref[1], atol=1e-6)


def test_gate_faces_the_camera_and_pass_direction_points_away():
    rng = np.random.default_rng(2)
    p_G, R_Gb, uv, t_true, up_cam = _scene(rng)
    s = pnp.solve_gate_pose(order_clockwise(uv), K, L, up_cam)[0]
    # object z (towards viewer) points back at the camera
    assert s.R_co[:, 2] @ s.t_co < 0
    # G's x axis (pass direction) points away from the camera
    assert s.R_cG[:, 0] @ s.t_co > 0


def test_gate_in_frame_is_consistent():
    """A gate seen from a pose estimated relative to the same gate lands at the origin."""
    rng = np.random.default_rng(3)
    p_G, R_Gb, uv, _, up_cam = _scene(rng)
    s = pnp.solve_gate_pose(order_clockwise(uv), K, L, up_cam)[0]
    R_Gb_est, p_est = pnp.body_pose_in_gate(s, R_BC, T_BC)
    R, t = pnp.gate_in_frame(s, R_Gb_est, p_est, R_BC, T_BC)
    assert np.allclose(t, 0, atol=1e-6) and np.allclose(R, np.eye(3), atol=1e-6)


def test_noisy_corners_position_error_matches_model():
    """With ~1 px corner noise the gate-centre error stays inside the model's 3-sigma."""
    rng = np.random.default_rng(4)
    worst = 0.0
    for _ in range(200):
        p_G, R_Gb, uv, t_true, up_cam = _scene(rng)
        noisy = uv + rng.normal(0, 1.0, uv.shape)
        sols = pnp.solve_gate_pose(order_clockwise(noisy), K, L, up_cam)
        if not sols:
            continue
        s = sols[0]
        C = pnp.gate_position_cov(s, K, corner_sigma=1.5, depth_factor=0.7, pos_floor=0.03)
        r = s.t_co - t_true
        worst = max(worst, float(r @ np.linalg.solve(C, r)))
    # chi-squared(3) 99.9% quantile is 16.3; allow some slack for model simplifications
    assert worst < 30.0, worst


def test_cv2_ippe_square_is_available():
    assert hasattr(cv2, "SOLVEPNP_IPPE_SQUARE")


def test_upright_refinement_is_exact_on_clean_corners():
    rng = np.random.default_rng(3)
    for _ in range(50):
        p_G, R_Gb, uv, t_true, up_cam = _scene(rng)
        q = order_clockwise(uv)
        us = pnp.upright_solutions(pnp.solve_gate_pose(q, K, L, up_cam), q, K, up_cam)
        assert len(us) == 1                      # both IPPE solutions converge to one upright pose
        assert np.allclose(us[0].t_co, t_true, atol=1e-5)
        assert abs(float(us[0].R_co[:, 1] @ up_cam) - 1.0) < 1e-9
        R_est, p_est = pnp.body_pose_in_gate(us[0], R_BC, T_BC)
        assert np.linalg.norm(log_so3(R_est.T @ R_Gb)) < 1e-5


def test_upright_refinement_reduces_yaw_error_with_noisy_corners():
    """Fixing roll/pitch from gravity takes the badly constrained rotation DoF out of PnP."""
    from vdr.geometry import yaw_of, wrap_angle
    rng = np.random.default_rng(4)
    e_ippe, e_up = [], []
    for _ in range(200):
        p_G, R_Gb, uv, _, up_cam = _scene(rng)
        q = order_clockwise(uv + rng.normal(0.0, 0.5, uv.shape))
        sols = pnp.solve_gate_pose(q, K, L, up_cam)
        if not sols:
            continue
        us = pnp.upright_solutions(sols, q, K, up_cam)
        for s, E in ((sols[0], e_ippe), (us[0], e_up)):
            R, _ = pnp.body_pose_in_gate(s, R_BC, T_BC)
            E.append(wrap_angle(yaw_of(R) - yaw_of(R_Gb)) ** 2)
    assert math.sqrt(np.mean(e_up)) < 0.8 * math.sqrt(np.mean(e_ippe))


def test_upright_solution_keeps_its_ippe_seed():
    """The stack takes roll/pitch from the unconstrained seed, so it must travel with the fit."""
    rng = np.random.default_rng(6)
    _, _, uv, _, up_cam = _scene(rng)
    q = order_clockwise(uv)
    sols = pnp.solve_gate_pose(q, K, L, up_cam)
    us = pnp.upright_solutions(sols, q, K, up_cam)
    assert us[0].ippe is not None and any(us[0].ippe is s for s in sols)
