"""Gate detector: finds square gate frames in the raw camera image.

Pipeline: HSV colour threshold -> contours with a two-level hierarchy
(outer boundaries + holes) -> quadrilateral fit per contour.

Two things make this harder than "find orange squares":

* Every gate looks the same, and the next gate is usually seen THROUGH or
  right next to the one being approached. Touching gates merge into one blob,
  so the outline is no longer a 4-gon. The quad fit therefore has a robust
  fallback: sequential RANSAC finds the four dominant straight edges of the
  contour and intersects them, treating the other gate's pixels as outliers.
* Each frame has two usable squares -- its OUTER boundary and its INNER
  opening (the hole). Holes often survive overlaps that ruin the outline, so
  unused holes become detections in their own right. Each detection records
  which square it is so PnP uses the right physical size.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from vdr.autonomy.config import AutonomyConfig

OUTER = "outer"
INNER = "inner"


@dataclass
class GateDetection:
    corners: np.ndarray            # (4, 2) float64, visually clockwise in the image
    kind: str                      # OUTER or INNER
    area: float                    # px^2 of the fitted quad
    inner: Optional[np.ndarray] = None   # matching inner-opening corners for an OUTER detection
    robust: bool = False           # True if the RANSAC edge fit was needed


def order_clockwise(pts: np.ndarray) -> np.ndarray:
    """Sort 4 image points visually clockwise (image v axis points down)."""
    c = pts.mean(axis=0)
    ang = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    return pts[np.argsort(ang)]


def _quad_area(q: np.ndarray) -> float:
    x, y = q[:, 0], q[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def edge_support(q: np.ndarray, pts: np.ndarray, tol: float = 1.5) -> float:
    """Fraction of contour points lying within `tol` px of the quad's edges."""
    d = np.full(len(pts), np.inf)
    for i in range(4):
        a, b = q[i], q[(i + 1) % 4]
        ab = b - a
        t = np.clip(((pts - a) @ ab) / max(float(ab @ ab), 1e-9), 0.0, 1.0)
        proj = a + t[:, None] * ab
        d = np.minimum(d, np.hypot(*(pts - proj).T))
    return float(np.mean(d < tol))


def _intersect(l1, l2) -> Optional[np.ndarray]:
    """Intersection of lines given as (n, c) with n . x + c = 0."""
    A = np.array([l1[0], l2[0]])
    det = A[0, 0] * A[1, 1] - A[0, 1] * A[1, 0]
    if abs(det) < 1e-9:
        return None
    return np.linalg.solve(A, -np.array([l1[1], l2[1]]))


def fit_quad_ransac(pts: np.ndarray, tol: float = 1.2, min_frac: float = 0.07) -> Optional[np.ndarray]:
    """Fit a quadrilateral to an ordered contour by finding its 4 dominant straight edges.

    Line hypotheses come from pairs of points a short way apart along the
    contour (both usually on the same edge); the hypothesis with most inliers
    wins, is refined by least squares, its inliers are removed, repeat x4.
    """
    N = len(pts)
    if N < 16:
        return None
    w = max(3, N // 16)
    remaining = np.ones(N, bool)
    lines = []
    for _ in range(4):
        idx = np.nonzero(remaining)[0]
        if len(idx) < 2 * w:
            return None
        starts = idx[:: max(1, len(idx) // 48)]
        a = pts[starts]
        b = pts[(starts + w) % N]
        d = b - a
        nrm = np.hypot(d[:, 0], d[:, 1])
        ok = nrm > 1e-6
        if not np.any(ok):
            return None
        a, d, nrm = a[ok], d[ok], nrm[ok]
        n = np.stack([-d[:, 1], d[:, 0]], axis=1) / nrm[:, None]
        c = -np.einsum("ij,ij->i", n, a)
        P = pts[idx]
        dist = np.abs(n @ P.T + c[:, None])                 # hypotheses x remaining points
        counts = (dist < tol).sum(axis=1)
        best = int(np.argmax(counts))
        if counts[best] < max(5, min_frac * N):
            return None
        inl = P[dist[best] < tol]
        vx, vy, x0, y0 = cv2.fitLine(inl.astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01).ravel()
        n_ref = np.array([-vy, vx], float)
        c_ref = -float(n_ref @ [x0, y0])
        lines.append((n_ref, c_ref, inl.mean(axis=0)))
        # remove points near the refined line (slightly wider band)
        remaining[idx[np.abs(P @ n_ref + c_ref) < 1.5 * tol]] = False

    centre = pts.mean(axis=0)
    ang = [np.arctan2(m[1] - centre[1], m[0] - centre[0]) for _, _, m in lines]
    lines = [lines[i] for i in np.argsort(ang)]
    corners = []
    for i in range(4):
        l1, l2 = lines[i], lines[(i + 1) % 4]
        cos = abs(float(l1[0] @ l2[0]))
        if cos > 0.94:                                     # adjacent edges nearly parallel
            return None
        x = _intersect(l1, l2)
        if x is None:
            return None
        corners.append(x)
    q = np.array(corners)
    if not cv2.isContourConvex(q.astype(np.float32).reshape(-1, 1, 2)):
        return None
    return order_clockwise(q)


class GateDetector:
    def __init__(self, cfg: AutonomyConfig, width: int, height: int):
        self.cfg = cfg
        self.W, self.H = width, height
        self.lo = np.array([cfg.hue_lo, cfg.sat_min, cfg.val_min], np.uint8)
        self.hi = np.array([cfg.hue_hi, 255, 255], np.uint8)
        self.last_mask: Optional[np.ndarray] = None

    def _in_image(self, q: np.ndarray) -> bool:
        m = self.cfg.border_margin
        return not (q[:, 0].min() < m or q[:, 1].min() < m or
                    q[:, 0].max() > self.W - 1 - m or q[:, 1].max() > self.H - 1 - m)

    def _quad(self, contour, contour_area: float):
        """Returns (quad, robust_flag) or (None, False)."""
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, self.cfg.approx_eps * peri, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            q = approx.reshape(4, 2).astype(np.float64)
            return (order_clockwise(q), False) if self._in_image(q) else (None, False)
        if not self.cfg.robust_quad_fit:
            return None, False
        pts = contour.reshape(-1, 2).astype(np.float64)
        q = fit_quad_ransac(pts)
        if q is None or not self._in_image(q):
            return None, False
        ratio = _quad_area(q) / max(contour_area, 1.0)
        if not 0.5 <= ratio <= 1.8:
            return None, False
        # Most of the outline must lie on the fitted edges; two merged gates don't.
        if edge_support(q, pts) < self.cfg.min_edge_support:
            return None, False
        return q, True

    def detect(self, image_bgr: np.ndarray) -> list:
        cfg = self.cfg
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lo, self.hi)
        self.last_mask = mask
        contours, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        if hier is None:
            return []
        hier = hier[0]
        areas = [cv2.contourArea(c) for c in contours]
        dets: list = []
        used_holes = set()
        # Pass 1: outer boundaries, each paired with its largest hole (its own opening).
        for i, cnt in enumerate(contours):
            if hier[i][3] != -1 or areas[i] < cfg.min_area_outer:
                continue
            best, best_area = None, 0.0
            child = hier[i][2]
            while child != -1:
                if areas[child] > best_area:
                    best, best_area = child, areas[child]
                child = hier[child][0]
            if best is None:
                continue                      # a solid blob, not a frame
            q, robust = self._quad(cnt, areas[i])
            if q is None:
                continue
            qa = _quad_area(q)
            if not 0.25 <= best_area / qa <= 0.95:
                continue                      # frame proportions are wrong (or merged gates)
            inner_q, _ = self._quad(contours[best], best_area)
            if inner_q is not None:
                used_holes.add(best)
            dets.append(GateDetection(q, OUTER, qa, inner_q, robust))
        # Pass 2: holes not already used -> inner-opening detections (merged / overlapping gates).
        for i, cnt in enumerate(contours):
            if hier[i][3] == -1 or i in used_holes or areas[i] < cfg.min_area_inner:
                continue
            q, robust = self._quad(cnt, areas[i])
            if q is None:
                continue
            dets.append(GateDetection(q, INNER, _quad_area(q), None, robust))
        dets.sort(key=lambda d: -d.area)
        return dets
