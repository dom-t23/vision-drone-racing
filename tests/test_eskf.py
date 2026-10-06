import math

import numpy as np

from vdr.autonomy.eskf import CHI2_3DOF_99, ESKF
from vdr.geometry import camera_mount, exp_so3, rot_z

R_BC = camera_mount(math.radians(20))
T_BC = np.array([0.08, 0.0, 0.02])


def _filter(p=(-5.0, 0.3, 0.1)):
    f = ESKF(accel_noise=0.08, gyro_noise=0.003, accel_bias_walk=0.005, gyro_bias_walk=0.0005)
    P0 = np.diag([0.3] * 3 + [0.1] * 3 + [0.03] * 3 + [0.1] * 3 + [0.01] * 3) ** 2
    f.initialize(p, np.zeros(3), np.eye(3), P0)
    return f


def _t_co(p, R):
    return R_BC.T @ (R.T @ (-np.asarray(p)) - T_BC)


def test_hover_imu_keeps_state_still():
    f = _filter()
    p0 = f.p.copy()
    for _ in range(200):                                   # 1 s of perfect hover IMU at 200 Hz
        f.predict(np.array([0.0, 0.0, 9.81]), np.zeros(3), 0.005)
    assert np.linalg.norm(f.p - p0) < 1e-9 and np.linalg.norm(f.v) < 1e-9


def test_constant_acceleration_integrates():
    f = _filter(p=(0.0, 0.0, 0.0))
    for _ in range(200):
        f.predict(np.array([2.0, 0.0, 9.81]), np.zeros(3), 0.005)
    assert np.isclose(f.v[0], 2.0, atol=1e-9)
    assert np.isclose(f.p[0], 1.0, atol=1e-2)


def test_gate_measurement_pulls_state_and_shrinks_covariance():
    f = _filter(p=(-5.0, 0.3, 0.1))
    true_p = np.array([-5.2, 0.0, 0.0])
    Rn = np.eye(3) * 0.05 ** 2
    before = np.trace(f.P[:3, :3])
    for _ in range(20):
        ok, _ = f.update_gate_position(_t_co(true_p, np.eye(3)), Rn, R_BC, T_BC)
        assert ok
    assert np.linalg.norm(f.p - true_p) < 0.1
    assert np.trace(f.P[:3, :3]) < before


def test_chi_squared_gate_rejects_outliers():
    f = _filter(p=(-5.0, 0.0, 0.0))
    Rn = np.eye(3) * 0.05 ** 2
    wild = _t_co(np.array([-5.0, 4.0, 2.0]), np.eye(3))         # 4 m sideways: impossible
    p_before = f.p.copy()
    ok, chi2 = f.update_gate_position(wild, Rn, R_BC, T_BC, gate=True, thresh=CHI2_3DOF_99)
    assert not ok and chi2 > CHI2_3DOF_99
    assert np.allclose(f.p, p_before)
    ok, _ = f.update_gate_position(wild, Rn, R_BC, T_BC, gate=False)
    assert ok                                                   # without gating it is swallowed


def test_attitude_update_corrects_yaw():
    f = _filter()
    R_true = rot_z(0.05)
    for _ in range(10):
        f.update_attitude(R_true, np.eye(3) * math.radians(1.0) ** 2)
    assert abs(math.atan2(f.R[1, 0], f.R[0, 0]) - 0.05) < 0.005


def test_position_jacobian_matches_finite_differences():
    f = _filter(p=(-4.0, 0.5, -0.3))
    f.R = exp_so3([0.05, -0.1, 0.2])
    r0, H, _ = f.gate_position_innovation(np.zeros(3), np.eye(3), R_BC, T_BC)
    z0 = -r0
    eps = 1e-6
    for i in range(3):
        dp = np.zeros(3); dp[i] = eps
        z = R_BC.T @ (f.R.T @ (-(f.p + dp)) - T_BC)
        assert np.allclose((z - z0) / eps, H[:, i], atol=1e-5)
        dth = np.zeros(3); dth[i] = eps
        R = f.R @ exp_so3(dth)
        z = R_BC.T @ (R.T @ (-f.p) - T_BC)
        assert np.allclose((z - z0) / eps, H[:, 6 + i], atol=1e-5)


def test_change_frame_moves_origin_to_new_gate():
    f = _filter(p=(0.4, 0.0, 0.0))                  # just through gate G
    f.v = np.array([8.0, 0.0, 0.0])
    yaw, t = math.radians(30), np.array([8.0, 4.0, 0.5])
    f.change_frame(yaw, t, np.eye(3) * 0.2 ** 2, math.radians(5))
    expected = rot_z(yaw).T @ (np.array([0.4, 0.0, 0.0]) - t)
    assert np.allclose(f.p, expected)
    assert np.allclose(f.v, rot_z(yaw).T @ [8.0, 0.0, 0.0])
    assert np.all(np.linalg.eigvalsh(f.P) > 0)
