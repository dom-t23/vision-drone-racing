import cv2
import math

import numpy as np

from vdr.interface import Command
from vdr.sim import SimConfig, Simulator
from vdr.sim.course import generate_course
from vdr.sim.dynamics import QuadDynamics, QuadState
from vdr.geometry import yaw_of


def _state():
    return QuadState(p=np.array([0.0, 0.0, 2.0]), v=np.zeros(3), R=np.eye(3), omega=np.zeros(3), thrust=9.81)


def test_hover_is_an_equilibrium():
    dyn = QuadDynamics(SimConfig())
    s = _state()
    for _ in range(400):
        dyn.step(s, 9.81, np.zeros(3))
    assert np.linalg.norm(s.p - [0, 0, 2]) < 1e-9


def test_zero_thrust_falls_at_g():
    cfg = SimConfig(tau_thrust=1e-4)
    dyn = QuadDynamics(cfg)
    s = _state()
    for _ in range(40):                                   # 0.1 s
        dyn.step(s, 0.0, np.zeros(3))
    assert math.isclose(s.v[2], -9.81 * 0.1, rel_tol=0.05)


def test_yaw_rate_command_turns_the_drone():
    dyn = QuadDynamics(SimConfig())
    s = _state()
    for _ in range(400):                                  # 1 s at 1 rad/s (minus the rate lag)
        dyn.step(s, 9.81, np.array([0.0, 0.0, 1.0]))
    assert 0.9 < yaw_of(s.R) < 1.0


def test_imu_reads_gravity_at_hover():
    sim = Simulator(SimConfig(gyro_noise=0.0, accel_noise=0.0, gyro_bias_sigma=0.0,
                              accel_bias_sigma=0.0, gyro_bias_walk=0.0, accel_bias_walk=0.0), seed=0)
    obs = sim.reset()
    for _ in range(5):
        obs = sim.step(Command.hover())
    assert np.allclose(obs.accel, [0, 0, 9.81], atol=1e-6)
    assert np.allclose(obs.gyro, 0, atol=1e-9)


def test_courses_are_deterministic_and_valid():
    cfg = SimConfig()
    for seed in range(30):
        a = generate_course(cfg, np.random.default_rng(seed))
        b = generate_course(cfg, np.random.default_rng(seed))
        assert np.allclose(a.centers(), b.centers())
        C = a.centers()
        # spacing and heights
        legs = np.linalg.norm(np.diff(C, axis=0), axis=1)
        assert np.all(legs > cfg.spacing_min - 1.0)       # (height change adds a little)
        assert np.all((C[:, 2] >= cfg.z_min) & (C[:, 2] <= cfg.z_max))
        # next gate is the nearest gate in the forward cone of each gate
        for i in range(len(C) - 1):
            n = a.gates[i].normal
            ahead = []
            for j in range(len(C)):
                d = C[j] - C[i]
                if j != i and d @ n / np.linalg.norm(d) > math.cos(math.radians(cfg.next_gate_cone_deg)):
                    ahead.append((np.linalg.norm(d), j))
            assert min(ahead)[1] == i + 1


def test_scoring_pass_and_crash():
    cfg = SimConfig()
    sim = Simulator(cfg, seed=0)
    g = sim.course.gates[0]
    # straight through the centre -> pass
    sim._check_events(g.center - 0.1 * g.normal, g.center + 0.1 * g.normal)
    assert sim.next_gate == 1 and not sim.done
    # through the frame of gate 1 -> crash
    g1 = sim.course.gates[1]
    off = g1.R @ np.array([0.0, (cfg.gate_inner + cfg.gate_outer) / 4.0, 0.0])
    sim._check_events(g1.center + off - 0.1 * g1.normal, g1.center + off + 0.1 * g1.normal)
    assert sim.outcome == "crash_gate"


def test_race_clock_scales_with_course_length():
    sim = Simulator(SimConfig(), seed=3)
    assert sim.time_limit < SimConfig().time_limit
    assert math.isclose(sim.time_limit, 2.0 + sim.path_length / 5.0)



def test_supersampled_gate_fill_has_area_coverage():
    """gate_supersample: an edge pixel half covered by the polygon gets ~half the colour,
    and the painted area matches the polygon's (cv2 LINE_AA paints ~0.7 px too wide)."""
    import numpy as np
    from vdr.sim.camera import FpvCamera
    cfg = SimConfig(pixel_noise_sigma=0.0, gate_supersample=8)
    cam = FpvCamera(cfg, np.random.default_rng(0))
    cam.W, cam.H = 40, 40
    img = np.zeros((40, 40, 3), np.uint8)
    sq = np.array([[10.03, 10.03], [30.03, 10.03], [30.03, 30.03], [10.03, 30.03]])
    hole = np.array([[15.03, 15.03], [25.03, 15.03], [25.03, 25.03], [15.03, 25.03]])
    cam._fill_coverage(img, [sq, hole], (255, 255, 255))
    a = img[..., 0].astype(float) / 255.0
    for v, u in ((20, 10), (20, 30), (10, 20), (30, 20), (20, 15), (20, 25)):
        assert abs(a[v, u] - 0.5) < 0.1, (v, u, a[v, u])
    assert a[20, 9] == 0.0 and a[20, 11] == 1.0
    assert abs(a.sum() - (400 - 100)) < 1.0

    legacy = np.zeros((40, 40, 3), np.uint8)
    cam.cfg = SimConfig(pixel_noise_sigma=0.0)
    pts = [np.round(P * 16).astype(np.int32) for P in (sq, hole)]
    cv2.fillPoly(legacy, pts, (255, 255, 255), lineType=cv2.LINE_AA, shift=4)
    assert legacy[..., 0].sum() / 255.0 - 300 > 20   # the old fill is visibly too fat


def test_camera_latency_delivers_the_frame_captured_earlier():
    """With latency, each frame arrives N ticks late and shows the world at capture time."""
    base = SimConfig(pixel_noise_sigma=0.0)
    lag = base.override(camera_latency=0.03)              # 0.03 s / 5 ms ticks = 6 ticks
    ticks = int(round(lag.camera_latency / lag.control_dt))
    cmd = Command(thrust=12.0, rates=np.array([0.0, 0.3, 0.0]))   # climb and pitch: frames change
    a, b = Simulator(base, seed=3), Simulator(lag, seed=3)
    fa = {0: a.reset().image}
    ob = b.reset()
    assert ob.image is None
    fb = {}
    for k in range(1, 30):
        oa, ob = a.step(cmd), b.step(cmd)
        if oa.image is not None:
            fa[k] = oa.image
        if ob.image is not None:
            fb[k] = ob.image
    assert sorted(fb) == [k + ticks for k in sorted(fa) if k + ticks < 30]
    for k, img in fb.items():
        assert np.array_equal(img, fa[k - ticks])
    # zero latency (the default) leaves the frame schedule unchanged
    assert SimConfig().camera_latency == 0.0


def test_motion_blur_averages_the_shutter_interval():
    """exposure_time > 0: a still drone renders exactly as before; a spinning one smears edges
    over the angle turned during the exposure, and the sampled poses span [t - exposure, t]."""
    sharp_cfg = SimConfig(pixel_noise_sigma=0.0)
    blur_cfg = sharp_cfg.override(exposure_time=0.02)
    a, b = Simulator(sharp_cfg, seed=3), Simulator(blur_cfg, seed=3)
    assert np.array_equal(a.reset().image, b.reset().image)      # no motion yet: identical

    cmd = Command(thrust=9.81, rates=np.array([0.0, 0.0, 4.0]))   # fast yaw
    for _ in range(3 * sharp_cfg.camera_every):
        oa, ob = a.step(cmd), b.step(cmd)
    assert oa.image is not None and ob.image is not None
    assert np.allclose(a.state.p, b.state.p) and np.allclose(a.state.R, b.state.R)

    def mixed(img):   # pixels that are none of sky, ground or gate: a blend across an edge
        from vdr.sim.camera import GATE_BGR, GROUND_BGR, SKY_BGR
        x = img.astype(int)
        far = [np.abs(x - np.array(c)).sum(-1) > 30 for c in (SKY_BGR, GROUND_BGR, GATE_BGR)]
        return int(np.sum(far[0] & far[1] & far[2]))

    assert not np.array_equal(oa.image, ob.image)
    assert mixed(ob.image) > 2 * mixed(oa.image)

    assert len(b._exposure_poses()) > 8                 # ~11 px of motion -> many sub-frames
    still = Simulator(blur_cfg, seed=3)
    still.reset()
    assert len(still._exposure_poses()) == 1            # no motion -> a single render
    poses = b._exposure_poses(n=8)
    yaw_now = yaw_of(b.state.R)
    first, last = yaw_of(poses[0][1]), yaw_of(poses[-1][1])
    w = b.state.omega[2]
    n = 8
    assert abs((yaw_now - first) - w * blur_cfg.exposure_time * (1 - 0.5 / n)) < 0.01
    assert abs((yaw_now - last) - w * blur_cfg.exposure_time * 0.5 / n) < 0.01
    assert SimConfig().exposure_time == 0.0       # default: instantaneous shutter


def test_distractors_are_clutter_off_the_racing_line():
    """n_distractors > 0 adds orange clutter beside the course without changing the course,
    IMU or camera-noise streams; every object keeps its clearance from the legs and gates,
    it is painted in gate-like colours, and n_distractors = 0 renders exactly as before."""
    from vdr.sim.course import _segment_point_dist
    base_cfg = SimConfig(pixel_noise_sigma=0.0)
    clut_cfg = base_cfg.override(n_distractors=25)
    a, b = Simulator(base_cfg, seed=5), Simulator(clut_cfg, seed=5)
    assert a.course.distractors == [] and len(b.course.distractors) == 25
    assert np.allclose(a.course.centers(), b.course.centers())
    assert np.allclose(a.course.start_position, b.course.start_position)
    again = Simulator(clut_cfg, seed=5)
    assert all(np.allclose(d.vertices, e.vertices) for d, e in zip(b.course.distractors, again.course.distractors))

    C = b.course.centers()
    legs = [(b.course.start_position, C[0])] + [(C[i], C[i + 1]) for i in range(len(C) - 1)]
    for d in b.course.distractors:
        assert d.kind in ("panel", "cone")
        assert np.all(d.vertices[:, 2] >= -1e-9)
        for p, q in legs:
            for v in d.vertices:
                assert _segment_point_dist(p, q, v) >= clut_cfg.distractor_clearance - 1e-9
        hsv = cv2.cvtColor(np.uint8([[d.colour]]), cv2.COLOR_BGR2HSV)[0, 0]
        assert hsv[0] <= 25 and hsv[1] >= 120 and hsv[2] >= 100    # passes a gate-paint threshold

    ia, ib = a.reset().image, b.reset().image
    hsv_a = cv2.inRange(cv2.cvtColor(ia, cv2.COLOR_BGR2HSV), (0, 120, 100), (25, 255, 255))
    hsv_b = cv2.inRange(cv2.cvtColor(ib, cv2.COLOR_BGR2HSV), (0, 120, 100), (25, 255, 255))
    assert np.count_nonzero(hsv_b) > np.count_nonzero(hsv_a)       # clutter is in view at the start
    assert np.array_equal(ia, Simulator(base_cfg, seed=5).reset().image)
    assert SimConfig().n_distractors == 0         # default: no clutter (old behaviour)
