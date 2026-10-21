"""Single-person identity lock based on colour appearance and motion continuity.

Pure Python/OpenCV (no ROS) so the logic can be tested without a camera.

The lock only decides *which detection box* belongs to the person the user
locked onto. Position filtering stays in ``person_3d_node``.
"""

from dataclasses import dataclass
import time
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np


OFF = "OFF"
PENDING = "PENDING"
LOCKED = "LOCKED"
LOST = "LOST"

BBox = Tuple[float, float, float, float]


@dataclass(frozen=True)
class LockConfig:
    # Torso HSV histogram.
    hist_bins_h: int = 8
    hist_bins_s: int = 4
    hist_bins_v: int = 4
    hist_min_value: int = 20
    torso_x0: float = 0.20
    torso_y0: float = 0.15
    torso_x1: float = 0.80
    torso_y1: float = 0.55
    min_torso_px: int = 64
    min_box_height_frac: float = 0.06

    # Primary selection when the lock request arrives.
    center_weight: float = 1.0
    depth_weight: float = 0.35
    depth_none_term: float = 0.5
    depth_none_penalty: float = 0.05
    depth_max_m: float = 6.0

    # Frame-to-frame association while locked.
    track_color_max_dist: float = 0.55
    track_relaxed_color_max_dist: float = 0.70
    track_strong_iou: float = 0.45
    track_min_iou: float = 0.10
    track_max_center_norm: float = 2.0
    track_max_scale_ratio: float = 2.5
    ambiguity_score_margin: float = 0.08

    # Re-acquisition after the target was lost.
    reacquire_color_max_dist: float = 0.40
    reacquire_confirm_frames: int = 2
    reacquire_match_margin: float = 0.06

    # Appearance adaptation while matched.
    reference_update_alpha: float = 0.10
    reference_update_max_dist: float = 0.35

    # Camera gap after which spatial continuity is no longer trusted.
    gap_reset_sec: float = 0.5


@dataclass
class Detection:
    bbox: BBox
    confidence: float = 1.0
    depth_m: Optional[float] = None
    hist: Optional[np.ndarray] = None


@dataclass
class LockResult:
    state: str
    selected_index: Optional[int]
    reason: str
    color_distance: Optional[float] = None
    num_detections: int = 0
    generation: int = 0
    reacquire_hits: int = 0


def torso_histogram(
    frame_bgr: np.ndarray,
    bbox: BBox,
    cfg: LockConfig,
) -> Optional[np.ndarray]:
    """Return an L1-normalised HSV histogram of the torso region."""

    if (
        frame_bgr is None
        or getattr(frame_bgr, "ndim", 0) != 3
        or frame_bgr.shape[2] != 3
    ):
        return None

    frame_h, frame_w = frame_bgr.shape[:2]
    x1, y1, x2, y2 = bbox
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1

    box_w = x2 - x1
    box_h = y2 - y1
    if box_w < 2.0 or box_h < 2.0:
        return None

    rx1 = int(round(x1 + cfg.torso_x0 * box_w))
    rx2 = int(round(x1 + cfg.torso_x1 * box_w))
    ry1 = int(round(y1 + cfg.torso_y0 * box_h))
    ry2 = int(round(y1 + cfg.torso_y1 * box_h))

    rx1 = max(0, min(frame_w, rx1))
    rx2 = max(0, min(frame_w, rx2))
    ry1 = max(0, min(frame_h, ry1))
    ry2 = max(0, min(frame_h, ry2))
    if rx2 <= rx1 or ry2 <= ry1:
        return None
    if (rx2 - rx1) * (ry2 - ry1) < cfg.min_torso_px:
        return None

    roi = frame_bgr[ry1:ry2, rx1:rx2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    mask = (hsv[:, :, 2] >= cfg.hist_min_value).astype(np.uint8) * 255
    if cv2.countNonZero(mask) < int(0.1 * mask.size):
        # Near-black clothing: keep the pixels instead of failing.
        mask = None

    hist = cv2.calcHist(
        [hsv],
        [0, 1, 2],
        mask,
        [cfg.hist_bins_h, cfg.hist_bins_s, cfg.hist_bins_v],
        [0, 180, 0, 256, 0, 256],
    )
    hist = hist.flatten().astype(np.float32)
    total = float(hist.sum())
    if total <= 0.0 or not np.isfinite(total):
        return None
    return hist / total


def histogram_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Bhattacharyya distance: 0 = identical, 1 = disjoint."""

    if a is None or b is None:
        return 1.0
    a = np.ascontiguousarray(a, dtype=np.float32).reshape(-1, 1)
    b = np.ascontiguousarray(b, dtype=np.float32).reshape(-1, 1)
    if a.shape != b.shape:
        return 1.0
    value = float(cv2.compareHist(a, b, cv2.HISTCMP_BHATTACHARYYA))
    return max(0.0, min(1.0, value))


def bbox_center(bbox: BBox) -> Tuple[float, float]:
    x1, y1, x2, y2 = bbox
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def bbox_area(bbox: BBox) -> float:
    x1, y1, x2, y2 = bbox
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def bbox_diag(bbox: BBox) -> float:
    x1, y1, x2, y2 = bbox
    return float(np.hypot(x2 - x1, y2 - y1))


def bbox_iou(a: BBox, b: BBox) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if inter <= 0.0:
        return 0.0
    union = bbox_area(a) + bbox_area(b) - inter
    return inter / union if union > 0.0 else 0.0


def _normalize_hist(hist: np.ndarray) -> Optional[np.ndarray]:
    total = float(hist.sum())
    if total <= 0.0 or not np.isfinite(total):
        return None
    return (hist / total).astype(np.float32)


class PersonLock:
    """State machine that pins a single person across frames."""

    def __init__(self, config: Optional[LockConfig] = None) -> None:
        self.cfg = config or LockConfig()
        self._state = OFF
        self._generation = 0
        self._ref_hist: Optional[np.ndarray] = None
        self._last_bbox: Optional[BBox] = None
        self._last_seen: Optional[float] = None
        self._pending_bbox: Optional[BBox] = None
        self._pending_hits = 0

    @property
    def state(self) -> str:
        return self._state

    @property
    def generation(self) -> int:
        return self._generation

    @property
    def reference_histogram(self) -> Optional[np.ndarray]:
        return self._ref_hist

    # ------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------

    def request_lock(self) -> None:
        """Lock the next primary person. Idempotent while already locked."""

        if self._state != OFF:
            return
        self._clear_identity()
        self._state = PENDING

    def request_relock(self) -> bool:
        """Drop the current identity and select a new primary person."""

        if self._state == OFF:
            return False
        self._clear_identity()
        self._state = PENDING
        return True

    def request_unlock(self) -> None:
        self._clear_identity()
        self._state = OFF

    # ------------------------------------------------------------
    # Per-frame update
    # ------------------------------------------------------------

    def update(
        self,
        detections: Sequence[Detection],
        frame_width: int,
        frame_height: int,
        now: Optional[float] = None,
    ) -> LockResult:
        now = time.monotonic() if now is None else float(now)
        num = len(detections)

        if self._state == OFF:
            return LockResult(OFF, None, "UNLOCKED", num_detections=num)

        if self._state == PENDING:
            return self._acquire(
                detections, frame_width, frame_height, now, num)

        if self._state == LOCKED:
            if (
                self._last_seen is not None
                and now - self._last_seen > self.cfg.gap_reset_sec
            ):
                self._state = LOST
                self._pending_bbox = None
                self._pending_hits = 0
                return LockResult(
                    LOST, None, "GAP_RESET",
                    num_detections=num, generation=self._generation)

            return self._match(detections, now, num)

        # LOST
        return self._reacquire(detections, now, num)

    # ------------------------------------------------------------
    # Lock-time primary selection
    # ------------------------------------------------------------

    def _acquire(
        self,
        detections: Sequence[Detection],
        frame_width: int,
        frame_height: int,
        now: float,
        num: int,
    ) -> LockResult:
        index = self._select_primary(
            detections, frame_width, frame_height)
        if index is None:
            reason = "NO_DETECTION" if num == 0 else "NO_APPEARANCE"
            return LockResult(
                PENDING, None, reason,
                num_detections=num, generation=self._generation)

        detection = detections[index]
        self._ref_hist = _normalize_hist(detection.hist)
        if self._ref_hist is None:
            return LockResult(
                PENDING, None, "NO_APPEARANCE",
                num_detections=num, generation=self._generation)

        self._last_bbox = detection.bbox
        self._last_seen = now
        self._state = LOCKED
        self._generation += 1
        return LockResult(
            LOCKED, index, "ACQUIRED",
            color_distance=0.0,
            num_detections=num,
            generation=self._generation,
        )

    def _select_primary(
        self,
        detections: Sequence[Detection],
        frame_width: int,
        frame_height: int,
    ) -> Optional[int]:
        best_index = None
        best_key = None
        half_width = max(frame_width / 2.0, 1.0)

        for index, detection in enumerate(detections):
            if detection.hist is None:
                continue

            x1, y1, x2, y2 = detection.bbox
            if (y2 - y1) < self.cfg.min_box_height_frac * frame_height:
                continue

            cx, _ = bbox_center(detection.bbox)
            center_offset = min(
                abs(cx - frame_width / 2.0) / half_width, 1.0)

            depth = detection.depth_m
            if depth is not None and 0.1 < depth < 10.0:
                depth_term = min(depth, self.cfg.depth_max_m) / self.cfg.depth_max_m
                penalty = 0.0
            else:
                depth_term = self.cfg.depth_none_term
                penalty = self.cfg.depth_none_penalty

            score = (
                self.cfg.center_weight * center_offset
                + self.cfg.depth_weight * depth_term
                + penalty
            )

            # Smaller key wins; ties prefer a larger box, then confidence.
            key = (score, -bbox_area(detection.bbox), -detection.confidence)
            if best_key is None or key < best_key:
                best_key = key
                best_index = index

        return best_index

    # ------------------------------------------------------------
    # Association while LOCKED
    # ------------------------------------------------------------

    def _match(
        self,
        detections: Sequence[Detection],
        now: float,
        num: int,
    ) -> LockResult:
        if self._ref_hist is None or self._last_bbox is None:
            self._state = LOST
            return LockResult(
                LOST, None, "NO_REFERENCE",
                num_detections=num, generation=self._generation)

        last_diag = max(bbox_diag(self._last_bbox), 1.0)
        last_area = max(bbox_area(self._last_bbox), 1.0)
        candidates = []

        for index, detection in enumerate(detections):
            if detection.hist is None:
                continue

            dist = histogram_distance(self._ref_hist, detection.hist)
            iou = bbox_iou(self._last_bbox, detection.bbox)

            cx, cy = bbox_center(detection.bbox)
            lx, ly = bbox_center(self._last_bbox)
            center_norm = float(np.hypot(cx - lx, cy - ly)) / last_diag

            area_ratio = max(
                bbox_area(detection.bbox) / last_area,
                last_area / max(bbox_area(detection.bbox), 1.0),
            )
            if area_ratio > self.cfg.track_max_scale_ratio:
                continue

            spatial_ok = (
                iou >= self.cfg.track_min_iou
                or center_norm <= self.cfg.track_max_center_norm
            )
            color_limit = (
                self.cfg.track_relaxed_color_max_dist
                if iou >= self.cfg.track_strong_iou
                else self.cfg.track_color_max_dist
            )
            if not spatial_ok or dist > color_limit:
                continue

            score = (
                0.60 * (1.0 - dist)
                + 0.30 * iou
                + 0.10 * (1.0 - min(center_norm, 1.0))
            )
            candidates.append((score, dist, index, detection))

        if not candidates:
            self._state = LOST
            self._pending_bbox = None
            self._pending_hits = 0
            return LockResult(
                LOST, None, "NO_MATCH",
                num_detections=num, generation=self._generation)

        candidates.sort(key=lambda item: item[0], reverse=True)
        best_score, best_dist, best_index, best_detection = candidates[0]

        if (
            len(candidates) > 1
            and best_score - candidates[1][0] < self.cfg.ambiguity_score_margin
        ):
            self._state = LOST
            self._pending_bbox = None
            self._pending_hits = 0
            return LockResult(
                LOST, None, "AMBIGUOUS",
                color_distance=best_dist,
                num_detections=num,
                generation=self._generation,
            )

        if best_dist <= self.cfg.reference_update_max_dist:
            blended = (
                (1.0 - self.cfg.reference_update_alpha) * self._ref_hist
                + self.cfg.reference_update_alpha * best_detection.hist
            )
            updated = _normalize_hist(blended)
            if updated is not None:
                self._ref_hist = updated

        self._last_bbox = best_detection.bbox
        self._last_seen = now
        self._pending_bbox = None
        self._pending_hits = 0
        return LockResult(
            LOCKED, best_index, "MATCHED",
            color_distance=best_dist,
            num_detections=num,
            generation=self._generation,
        )

    # ------------------------------------------------------------
    # Re-acquisition while LOST
    # ------------------------------------------------------------

    def _reacquire(
        self,
        detections: Sequence[Detection],
        now: float,
        num: int,
    ) -> LockResult:
        if self._ref_hist is None:
            self._state = PENDING
            return LockResult(
                PENDING, None, "NO_REFERENCE",
                num_detections=num, generation=self._generation)

        candidates = []
        for index, detection in enumerate(detections):
            if detection.hist is None:
                continue
            dist = histogram_distance(self._ref_hist, detection.hist)
            if dist > self.cfg.reacquire_color_max_dist:
                continue
            candidates.append((dist, index, detection))

        if not candidates:
            self._pending_bbox = None
            self._pending_hits = 0
            return LockResult(
                LOST, None, "NO_MATCH",
                num_detections=num, generation=self._generation)

        candidates.sort(key=lambda item: item[0])
        best_dist, best_index, best_detection = candidates[0]

        if (
            len(candidates) > 1
            and candidates[1][0] - best_dist < self.cfg.reacquire_match_margin
        ):
            self._pending_bbox = None
            self._pending_hits = 0
            return LockResult(
                LOST, None, "AMBIGUOUS",
                color_distance=best_dist,
                num_detections=num,
                generation=self._generation,
            )

        if (
            self._pending_bbox is not None
            and bbox_iou(self._pending_bbox, best_detection.bbox) >= 0.3
        ):
            self._pending_hits += 1
        else:
            self._pending_hits = 1
        self._pending_bbox = best_detection.bbox

        if self._pending_hits < self.cfg.reacquire_confirm_frames:
            return LockResult(
                LOST, None, "REACQUIRING",
                color_distance=best_dist,
                num_detections=num,
                generation=self._generation,
                reacquire_hits=self._pending_hits,
            )

        # Confirmed. Use the fresh appearance so slow lighting changes are
        # absorbed, then resume normal tracking.
        refreshed = _normalize_hist(best_detection.hist)
        if refreshed is not None:
            self._ref_hist = refreshed
        self._last_bbox = best_detection.bbox
        self._last_seen = now
        self._pending_bbox = None
        self._pending_hits = 0
        self._state = LOCKED
        self._generation += 1
        return LockResult(
            LOCKED, best_index, "REACQUIRED",
            color_distance=best_dist,
            num_detections=num,
            generation=self._generation,
        )

    # ------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------

    def _clear_identity(self) -> None:
        self._ref_hist = None
        self._last_bbox = None
        self._last_seen = None
        self._pending_bbox = None
        self._pending_hits = 0
