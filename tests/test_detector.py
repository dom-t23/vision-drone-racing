import math

import numpy as np

from vdr.autonomy import AutonomyConfig
from vdr.autonomy.detector import GateDetector, OUTER, fit_quad_ransac, order_clockwise
from vdr.geometry import rot_y, rot_z
from vdr.sim import SimConfig, Simulator


def _match_error(det_corners, truth):
    """Max corner error (px) over the 4 cyclic alignments."""
    t = order_clockwise(np.asarray(truth, float))
    return min(np.abs(np.roll(det_corners, k, axis=0) - t).max() for k in range(4))


def _view(sim, gate_index, p_G, yaw=0.0, pitch=0.0):
    g = sim.course.gates[gate_index]
    p_w = g.center + g.R @ np.asarray(p_G, float)
    R_wb = g.R @ rot_z(yaw) @ rot_y(pitch)
    return p_w, R_wb


def test_single_gate_corners_are_accurate():
    sim = Simulator(SimConfig(pixel_noise_sigma=3.0), seed=11)
    det = GateDetector(AutonomyConfig(), 320, 240)
    errs = []
    for d in (3.0, 4.5, 6.0):
        p_w, R_wb = _view(sim, 0, [-d, 0.3, 0.2], yaw=-0.05, pitch=0.25)
        img = sim.camera.render(p_w, R_wb)
        truth = sim.camera.project_gate_corners(p_w, R_wb, 0)
        dets = [x for x in det.detect(img) if x.kind == OUTER]
        best = min(dets, key=lambda x: np.linalg.norm(x.corners.mean(0) - truth.mean(0)))
        errs.append(_match_error(best.corners, truth))
    assert max(errs) < 2.0, errs


def test_overlapping_gates_still_give_the_near_gate():
    """At the start the next gates are seen through/next to gate 0; the outline merges."""
    det = GateDetector(AutonomyConfig(), 320, 240)
    ok = 0
    for seed in range(8):
        sim = Simulator(SimConfig(), seed)
        obs = sim.reset()
        truth = sim.camera.project_gate_corners(sim.state.p, sim.state.R, 0)
        dets = det.detect(obs.image)
        if dets and _match_error(dets[0].corners, truth) < 4.5 and dets[0].kind == OUTER:
            ok += 1
        elif dets:
            # an inner-opening detection of gate 0 is also acceptable
            ok += int(np.linalg.norm(dets[0].corners.mean(0) - truth.mean(0)) < 4.0)
    assert ok >= 7, ok


def test_ransac_quad_ignores_a_protrusion():
    # a 40x40 square outline with a bump on its right edge
    pts = []
    for x in range(10, 50):
        pts.append((x, 10))
    for y in range(10, 25):
        pts.append((50, y))
    for x in range(50, 56):
        pts.append((x, 25))
    for y in range(25, 31):
        pts.append((56, y))
    for x in range(56, 49, -1):
        pts.append((x, 31))
    for y in range(31, 50):
        pts.append((50, y))
    for x in range(50, 9, -1):
        pts.append((x, 50))
    for y in range(50, 9, -1):
        pts.append((10, y))
    q = fit_quad_ransac(np.array(pts, float))
    assert q is not None
    assert _match_error(q, [[10, 10], [50, 10], [50, 50], [10, 50]]) < 1.0


def test_empty_image_has_no_detections():
    det = GateDetector(AutonomyConfig(), 320, 240)
    img = np.zeros((240, 320, 3), np.uint8)
    img[:] = (225, 196, 160)
    assert det.detect(img) == []


def test_subpixel_corners_reduce_corner_error():
    """With area-coverage rendering, edge refinement brings corners to a fraction of a pixel."""
    from vdr.autonomy.detector import INNER
    sim = Simulator(SimConfig(pixel_noise_sigma=3.0, gate_supersample=8), seed=11)
    base = GateDetector(AutonomyConfig(subpixel_corners=False), 320, 240)
    sub = GateDetector(AutonomyConfig(subpixel_corners=True), 320, 240)
    e_base, e_sub = [], []
    for d in (3.0, 6.0, 10.0):
        p_w, R_wb = _view(sim, 0, [-d, 0.3, 0.2], yaw=-0.05, pitch=0.25)
        img = sim.camera.render(p_w, R_wb)
        truth = sim.camera.project_gate_corners(p_w, R_wb, 0)
        for det, errs in ((base, e_base), (sub, e_sub)):
            dets = [x for x in det.detect(img) if x.kind == OUTER]
            best = min(dets, key=lambda x: np.linalg.norm(x.corners.mean(0) - truth.mean(0)))
            errs.append(_match_error(best.corners, truth))
    assert max(e_sub) < 0.6, e_sub
    assert np.mean(e_sub) < 0.5 * np.mean(e_base), (e_sub, e_base)
