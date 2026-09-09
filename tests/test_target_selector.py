"""Unit tests for the perception logic that runs without a drone or a model.
感知邏輯的單元測試，不需要無人機或模型。

Uses stdlib unittest so no extra dependency is needed:
使用標準庫 unittest，不必再裝額外套件：

    python -m unittest discover -s tests -v
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vision.colour_filter import colour_ratio, is_target_colour, measure_colour  # noqa: E402
from vision.detection import Detection  # noqa: E402
from vision.target_selector import TargetSelector  # noqa: E402

FRAME_W, FRAME_H = 960, 720


def box(class_name="cup", confidence=0.9, x1=0.0, y1=0.0, x2=100.0, y2=100.0) -> Detection:
    return Detection(class_name, confidence, x1, y1, x2, y2)


def solid_frame(bgr: tuple[int, int, int]) -> np.ndarray:
    frame = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    frame[:, :] = bgr
    return frame


class DetectionGeometryTests(unittest.TestCase):
    def test_derived_geometry(self):
        detection = box(x1=100, y1=200, x2=300, y2=500)
        self.assertEqual(detection.width, 200)
        self.assertEqual(detection.height, 300)
        self.assertEqual(detection.center_x, 200)
        self.assertEqual(detection.center_y, 350)
        self.assertEqual(detection.area, 60000)

    def test_area_ratio_is_fraction_of_frame(self):
        # Half the width and half the height is a quarter of the area.
        # 寬高各一半，面積就是四分之一。
        detection = box(x1=0, y1=0, x2=FRAME_W / 2, y2=FRAME_H / 2)
        self.assertAlmostEqual(detection.area_ratio(FRAME_W, FRAME_H), 0.25)

    def test_error_sign_convention(self):
        centred = box(x1=FRAME_W / 2 - 10, y1=FRAME_H / 2 - 10, x2=FRAME_W / 2 + 10, y2=FRAME_H / 2 + 10)
        self.assertEqual(centred.error_from_centre(FRAME_W, FRAME_H), (0.0, 0.0))

        # A target on the right must produce a positive error so the yaw
        # controller turns clockwise.
        # 目標在右邊必須產生正誤差，偏航控制器才會往右轉。
        right = box(x1=FRAME_W - 100, y1=0, x2=FRAME_W, y2=100)
        error_x, _ = right.error_from_centre(FRAME_W, FRAME_H)
        self.assertGreater(error_x, 0)


class TargetSelectorTests(unittest.TestCase):
    def setUp(self):
        self.frame = solid_frame((0, 0, 0))
        self.selector = TargetSelector(
            target_class="cup",
            min_confidence=0.5,
            colour_filter_enabled=False,
        )

    def test_no_detections_means_no_target(self):
        result = self.selector.select([], self.frame)
        self.assertFalse(result.found)
        self.assertIsNone(result.target)

    def test_ignores_other_classes(self):
        result = self.selector.select([box(class_name="person")], self.frame)
        self.assertFalse(result.found)
        self.assertEqual(result.candidates, [])

    def test_rejects_low_confidence_and_records_it(self):
        result = self.selector.select([box(confidence=0.2)], self.frame)
        self.assertFalse(result.found)
        self.assertEqual(len(result.rejected_by_confidence), 1)

    def test_prefers_largest_box_over_highest_confidence(self):
        # The nearest instance matters more than the most certain one: a small,
        # very confident cup further away must not win.
        # 最近的實例比最有把握的更重要：遠處那只很小、但信心很高的杯子不該贏。
        small_confident = box(confidence=0.99, x1=0, y1=0, x2=50, y2=50)
        large = box(confidence=0.60, x1=100, y1=100, x2=400, y2=400)

        result = self.selector.select([small_confident, large], self.frame)
        self.assertIs(result.target, large)

    def test_selection_is_order_independent(self):
        a = box(confidence=0.7, x1=0, y1=0, x2=200, y2=200)
        b = box(confidence=0.8, x1=300, y1=300, x2=400, y2=400)

        forward = self.selector.select([a, b], self.frame).target
        backward = self.selector.select([b, a], self.frame).target
        self.assertIs(forward, a)
        self.assertIs(backward, a)

    def test_equal_boxes_break_tie_deterministically(self):
        left = box(confidence=0.8, x1=0, y1=0, x2=100, y2=100)
        right = box(confidence=0.8, x1=500, y1=0, x2=600, y2=100)

        first = self.selector.select([left, right], self.frame).target
        second = self.selector.select([right, left], self.frame).target
        self.assertIs(first, left)
        self.assertIs(second, left)


class ColourFilterTests(unittest.TestCase):
    def setUp(self):
        self.detection = box(x1=100, y1=100, x2=300, y2=300)

    def test_pure_red_scores_high(self):
        red = solid_frame((0, 0, 255))  # BGR  # OpenCV 的 BGR 順序
        self.assertAlmostEqual(measure_colour(red, self.detection, "red"), 1.0, places=3)
        self.assertTrue(is_target_colour(red, self.detection, "red"))

    def test_blue_scores_zero(self):
        blue = solid_frame((255, 0, 0))
        self.assertAlmostEqual(measure_colour(blue, self.detection, "red"), 0.0, places=3)
        self.assertFalse(is_target_colour(blue, self.detection, "red"))

    def test_both_red_hue_intervals_match(self):
        # Red wraps around hue 0, so it needs two HSV intervals. Check that a
        # sample from each side is counted.
        # 紅色繞過色相 0，所以需要兩段 HSV 區間。確認兩側各取一個樣本都會被算進去。
        import cv2

        for hue in (2, 176):
            patch = np.full((10, 10, 3), (hue, 200, 200), dtype=np.uint8)
            bgr = cv2.cvtColor(patch, cv2.COLOR_HSV2BGR)
            from config import RED_HSV_RANGES

            self.assertAlmostEqual(colour_ratio(bgr, RED_HSV_RANGES), 1.0, places=3)

    def test_crop_is_clamped_to_frame(self):
        frame = solid_frame((0, 0, 255))
        outside = box(x1=900, y1=700, x2=2000, y2=2000)
        # Must not raise even though the box runs past the frame edge.
        # 即使框超出畫面邊緣，也不可拋例外。
        self.assertGreater(measure_colour(frame, outside, "red"), 0.0)

    def test_unknown_colour_never_matches(self):
        frame = solid_frame((0, 0, 255))
        self.assertEqual(measure_colour(frame, self.detection, "chartreuse"), 0.0)


class SelectorWithColourTests(unittest.TestCase):
    def setUp(self):
        self.selector = TargetSelector(
            target_class="cup",
            min_confidence=0.5,
            colour_filter_enabled=True,
            target_colour="red",
            min_colour_ratio=0.25,
        )

    def test_red_cup_accepted(self):
        frame = solid_frame((0, 0, 255))
        result = self.selector.select([box()], frame)
        self.assertTrue(result.found)
        self.assertAlmostEqual(result.colour_ratio, 1.0, places=3)

    def test_white_cup_rejected_but_reported(self):
        frame = solid_frame((255, 255, 255))
        result = self.selector.select([box()], frame)
        self.assertFalse(result.found)
        self.assertEqual(len(result.rejected_by_colour), 1)
        # Still a candidate: it was the right class with enough confidence,
        # which is what distinguishes "wrong colour" from "not detected".
        # 它仍是候選：類別對、信心也夠。這正是「顏色不對」與「沒偵測到」的差別。
        self.assertEqual(len(result.candidates), 1)

    def test_picks_the_red_one_when_a_bigger_white_one_exists(self):
        frame = solid_frame((255, 255, 255))
        frame[100:300, 100:300] = (0, 0, 255)

        red_small = box(x1=100, y1=100, x2=300, y2=300)
        white_large = box(x1=400, y1=100, x2=900, y2=600)

        result = self.selector.select([white_large, red_small], frame)
        self.assertIs(result.target, red_small)


if __name__ == "__main__":
    unittest.main(verbosity=2)
