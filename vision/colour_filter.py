"""Optional colour validation for a detected object.
對已偵測物件做可選的顏色驗證。

YOLO's COCO classes have no notion of colour: a red cup and a white cup are
both "cup". When the experiment needs a specific one, this module checks what
fraction of the bounding box matches the requested hue.
YOLO 的 COCO 類別沒有顏色概念：紅杯與白杯都是 "cup"。實驗若需要指定顏色，
這個模組會檢查框內有多少比例符合指定色相。

Kept separate from detection so it can be switched off entirely
(``config.COLOUR_FILTER_ENABLED``) without touching the detector, and so it can
be tested on static crops.
與偵測分開，才能完全關閉（``config.COLOUR_FILTER_ENABLED``）而不動到偵測器，也能用靜態裁切圖測試。

Limitation: HSV thresholding is sensitive to lighting. Under the warm indoor
lamp in this room, a red object drifts towards orange and may fall below the
ratio threshold. Tune ``COLOUR_MIN_RATIO`` on real frames rather than trusting
the default.
限制：HSV 門檻對光線很敏感。室內暖燈下，紅色會偏向橘色，可能低於比例門檻。
請用真實影格調 ``COLOUR_MIN_RATIO``，不要迷信預設值。
"""

from __future__ import annotations

import logging
from typing import Optional, Sequence

import cv2
import numpy as np

import config
from vision.detection import Detection

LOGGER = logging.getLogger(__name__)

HsvRange = tuple[tuple[int, int, int], tuple[int, int, int]]

# Red straddles the wrap-around point of OpenCV's 0-179 hue scale, so it needs
# two intervals. Other colours can be added here as single-interval entries.
# 紅色跨過 OpenCV 0-179 色相的繞回點，所以需要兩段區間。其他顏色可以在這裡用單一段加入。
COLOUR_RANGES: dict[str, Sequence[HsvRange]] = {
    "red": config.RED_HSV_RANGES,
}


def crop(frame_bgr: np.ndarray, detection: Detection) -> Optional[np.ndarray]:
    """Cut the bounding box out of the frame, clamped to the frame bounds.
    從畫面裁出邊界框，並夾在畫面範圍內。
    """
    height, width = frame_bgr.shape[:2]
    x1, y1, x2, y2 = detection.as_int_box()

    x1 = max(0, min(x1, width - 1))
    y1 = max(0, min(y1, height - 1))
    x2 = max(x1 + 1, min(x2, width))
    y2 = max(y1 + 1, min(y2, height))

    region = frame_bgr[y1:y2, x1:x2]
    return region if region.size else None


def colour_ratio(crop_bgr: np.ndarray, ranges: Sequence[HsvRange]) -> float:
    """Fraction of pixels in the crop that fall inside any of the HSV ranges.
    裁切區塊中，落在任一 HSV 區間內的像素比例。
    """
    if crop_bgr is None or crop_bgr.size == 0:
        return 0.0

    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        mask |= cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))

    return float(np.count_nonzero(mask)) / mask.size


def measure_colour(frame_bgr: np.ndarray, detection: Detection, colour: str) -> float:
    """Colour match ratio for one detection, in the range 0..1.
    單一偵測的顏色吻合比例，範圍 0..1。
    """
    ranges = COLOUR_RANGES.get(colour)
    if ranges is None:
        LOGGER.warning("No HSV ranges defined for colour %r; treating as no match", colour)
        return 0.0

    region = crop(frame_bgr, detection)
    if region is None:
        return 0.0

    return colour_ratio(region, ranges)


def is_target_colour(
    frame_bgr: np.ndarray,
    detection: Detection,
    colour: str = config.TARGET_COLOUR,
    min_ratio: float = config.COLOUR_MIN_RATIO,
) -> bool:
    return measure_colour(frame_bgr, detection, colour) >= min_ratio
