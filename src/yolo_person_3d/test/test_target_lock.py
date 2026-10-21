#!/usr/bin/env python3
"""Self-tests for the pure target-lock logic. Run directly with python3."""

import os
import sys

sys.path.insert(
    0, os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)

import numpy as np

from yolo_person_3d.target_lock import (
    Detection,
    LOCKED,
    LOST,
    OFF,
    PENDING,
    PersonLock,
    histogram_distance,
    torso_histogram,
)

FRAME_W = 640
FRAME_H = 480
BOX_A = (270.0, 100.0, 370.0, 300.0)
BOX_A_MOVED = (290.0, 100.0, 390.0, 300.0)
BOX_B = (20.0, 100.0, 120.0, 300.0)


def one_hot(index):
    hist = np.zeros(8 * 4 * 4, dtype=np.float32)
    hist[index] = 1.0
    return hist


def det(bbox, hist, confidence=0.9, depth=None):
    return Detection(bbox, confidence, depth, hist)


def assert_eq(actual, expected, message):
    assert actual == expected, f"{message}: {actual!r} != {expected!r}"


def test_off_publishes_nothing():
    lock = PersonLock()
    result = lock.update([det(BOX_A, one_hot(1))], FRAME_W, FRAME_H)
    assert_eq(result.state, OFF, "state before lock")
    assert result.selected_index is None, "nothing selected before lock"


def test_lock_prefers_centered_person_over_nearer_edge_person():
    lock = PersonLock()
    lock.request_lock()
    result = lock.update(
        [
            det(BOX_A, one_hot(1), depth=3.0),
            det(BOX_B, one_hot(50), depth=1.5),
        ],
        FRAME_W,
        FRAME_H,
    )
    assert_eq(result.state, LOCKED, "lock state")
    assert_eq(result.selected_index, 0, "centered person wins")
    assert_eq(result.generation, 1, "first generation")


def test_lock_prefers_nearer_person_within_center_region():
    lock = PersonLock()
    lock.request_lock()
    near_center = (250.0, 100.0, 350.0, 300.0)
    result = lock.update(
        [
            det(BOX_A, one_hot(1), depth=3.0),
            det(near_center, one_hot(50), depth=1.5),
        ],
        FRAME_W,
        FRAME_H,
    )
    assert_eq(result.selected_index, 1, "nearer centered person wins")


def test_locked_person_stays_selected_when_others_appear():
    lock = PersonLock()
    lock.request_lock()
    lock.update([det(BOX_A, one_hot(1), depth=2.0)], FRAME_W, FRAME_H, now=0.0)
    result = lock.update(
        [
            det(BOX_A_MOVED, one_hot(1), depth=2.1),
            det(BOX_B, one_hot(50), depth=1.0),
        ],
        FRAME_W,
        FRAME_H,
        now=0.1,
    )
    assert_eq(result.state, LOCKED, "still locked")
    assert_eq(result.selected_index, 0, "same person selected")


def test_locked_person_absent_never_switches():
    lock = PersonLock()
    lock.request_lock()
    lock.update([det(BOX_A, one_hot(1))], FRAME_W, FRAME_H, now=0.0)
    result = lock.update(
        [det(BOX_B, one_hot(50))], FRAME_W, FRAME_H, now=0.1)
    assert_eq(result.state, LOST, "lost when target absent")
    assert result.selected_index is None, "no other person selected"

    result = lock.update([], FRAME_W, FRAME_H, now=0.2)
    assert_eq(result.state, LOST, "stays lost with no detections")
    assert result.selected_index is None, "still no selection"


def test_reacquire_same_person_after_occlusion():
    lock = PersonLock()
    lock.request_lock()
    lock.update([det(BOX_A, one_hot(1))], FRAME_W, FRAME_H, now=0.0)
    lock.update([det(BOX_B, one_hot(50))], FRAME_W, FRAME_H, now=0.1)

    # Wrong-colour person never re-acquires.
    result = lock.update(
        [det(BOX_B, one_hot(50))], FRAME_W, FRAME_H, now=0.2)
    assert_eq(result.state, LOST, "wrong colour stays lost")

    moved = (40.0, 120.0, 140.0, 320.0)
    result = lock.update(
        [det(moved, one_hot(1))], FRAME_W, FRAME_H, now=0.3)
    assert_eq(result.state, LOST, "one confirmation frame is not enough")
    assert_eq(result.reacquire_hits, 1, "first reacquire hit")

    result = lock.update(
        [det(moved, one_hot(1))], FRAME_W, FRAME_H, now=0.4)
    assert_eq(result.state, LOCKED, "reacquired after two frames")
    assert_eq(result.selected_index, 0, "reacquired person selected")
    assert_eq(result.generation, 2, "generation incremented on reacquire")


def test_crossing_ambiguity_stops_instead_of_guessing():
    lock = PersonLock()
    lock.request_lock()
    lock.update([det(BOX_A, one_hot(1))], FRAME_W, FRAME_H, now=0.0)
    overlapping = (280.0, 100.0, 380.0, 300.0)
    result = lock.update(
        [
            det(BOX_A, one_hot(1)),
            det(overlapping, one_hot(1)),
        ],
        FRAME_W,
        FRAME_H,
        now=0.1,
    )
    assert_eq(result.state, LOST, "ambiguous candidates stop tracking")
    assert_eq(result.reason, "AMBIGUOUS", "ambiguity reason")
    assert result.selected_index is None, "no guess"


def test_gap_reset_after_camera_stall():
    lock = PersonLock()
    lock.request_lock()
    lock.update([det(BOX_A, one_hot(1))], FRAME_W, FRAME_H, now=0.0)
    result = lock.update(
        [det(BOX_A, one_hot(1))], FRAME_W, FRAME_H, now=1.0)
    assert_eq(result.state, LOST, "gap forces lost")
    assert_eq(result.reason, "GAP_RESET", "gap reason")


def test_relock_and_unlock():
    lock = PersonLock()
    assert lock.request_relock() is False, "relock without lock is rejected"

    lock.request_lock()
    lock.update([det(BOX_A, one_hot(1))], FRAME_W, FRAME_H, now=0.0)
    assert_eq(lock.state, LOCKED, "locked")
    assert lock.request_relock() is True, "relock while locked"
    assert_eq(lock.state, PENDING, "relock repends selection")
    result = lock.update(
        [det(BOX_B, one_hot(50))], FRAME_W, FRAME_H, now=0.1)
    assert_eq(result.state, LOCKED, "relocked onto new person")
    assert_eq(result.selected_index, 0, "new primary selected")
    assert_eq(result.generation, 2, "relock increments generation")

    lock.request_unlock()
    assert_eq(lock.state, OFF, "unlock state")
    result = lock.update(
        [det(BOX_B, one_hot(50))], FRAME_W, FRAME_H, now=0.2)
    assert result.selected_index is None, "no target after unlock"


def test_repeated_lock_does_not_switch_person():
    lock = PersonLock()
    lock.request_lock()
    lock.update([det(BOX_A, one_hot(1))], FRAME_W, FRAME_H, now=0.0)
    reference = lock.reference_histogram.copy()

    lock.request_lock()  # repeated "跟着我"
    result = lock.update(
        [det(BOX_B, one_hot(50))], FRAME_W, FRAME_H, now=0.1)
    assert_eq(result.state, LOST, "still waiting for the same identity")
    assert_eq(result.generation, 1, "generation unchanged")
    assert np.allclose(reference, lock.reference_histogram), "descriptor kept"


def test_histogram_normalisation_and_colour_distance():
    frame = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    frame[:, :FRAME_W // 2] = (0, 0, 255)       # red (BGR)
    frame[:, FRAME_W // 2:] = (255, 0, 0)       # blue

    red_box = (50.0, 100.0, 300.0, 400.0)
    blue_box = (350.0, 100.0, 600.0, 400.0)
    red_hist = torso_histogram(frame, red_box, PersonLock().cfg)
    blue_hist = torso_histogram(frame, blue_box, PersonLock().cfg)

    assert red_hist is not None and blue_hist is not None, "histograms exist"
    assert abs(float(red_hist.sum()) - 1.0) < 1e-5, "L1 normalised"
    same = histogram_distance(red_hist, red_hist)
    different = histogram_distance(red_hist, blue_hist)
    assert same < 0.01, f"same colour distance {same}"
    assert different > 0.5, f"different colour distance {different}"


def test_tiny_roi_has_no_histogram():
    frame = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    assert torso_histogram(frame, (0.0, 0.0, 10.0, 10.0), PersonLock().cfg) is None


def main():
    tests = [
        test_off_publishes_nothing,
        test_lock_prefers_centered_person_over_nearer_edge_person,
        test_lock_prefers_nearer_person_within_center_region,
        test_locked_person_stays_selected_when_others_appear,
        test_locked_person_absent_never_switches,
        test_reacquire_same_person_after_occlusion,
        test_crossing_ambiguity_stops_instead_of_guessing,
        test_gap_reset_after_camera_stall,
        test_relock_and_unlock,
        test_repeated_lock_does_not_switch_person,
        test_histogram_normalisation_and_colour_distance,
        test_tiny_roi_has_no_histogram,
    ]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print(f"ALL {len(tests)} TESTS PASSED")


if __name__ == "__main__":
    main()
