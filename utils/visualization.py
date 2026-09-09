"""On-screen debug overlay.
螢幕上的除錯疊圖。

Drawing only. This module never decides anything - it renders whatever the main
loop hands it, so that what you see on screen is exactly the state the
controller acted on.
只負責畫。這個模組不做任何決策——主迴圈給什麼就畫什麼，因此畫面上看到的，就是控制器當下依據的狀態。
"""

from __future__ import annotations

from typing import Optional, Sequence

import cv2
import numpy as np

from vision.detection import Detection

_FONT = cv2.FONT_HERSHEY_SIMPLEX
_SCALE = 0.5
_THICKNESS = 1
_LINE_HEIGHT = 20
_MARGIN = 10

WHITE = (255, 255, 255)
GREEN = (0, 220, 0)
AMBER = (0, 190, 255)
RED = (0, 0, 255)


def _panel(frame: np.ndarray, height: int, width: int) -> None:
    """Darken the area behind the text so it stays readable on any scene.
    把文字後方區域變暗，讓任何場景下都仍可讀。
    """
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (width, height), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, dst=frame)


def draw_hud(frame: np.ndarray, lines: Sequence[tuple[str, tuple[int, int, int]]]) -> np.ndarray:
    """Draw ``(text, colour)`` rows in the top-left corner. Modifies in place.
    在左上角畫出 ``(文字, 顏色)`` 列。就地修改影格。
    """
    if not lines:
        return frame

    # Size the backing panel to the longest row so nothing spills onto the
    # image, which matters once target details make the rows variable length.
    # 底板寬度依最長那一列決定，避免文字溢到畫面。目標細節讓列長變不固定後，這點更重要。
    text_width = max(cv2.getTextSize(text, _FONT, _SCALE, _THICKNESS)[0][0] for text, _ in lines)
    panel_width = min(frame.shape[1], text_width + 2 * _MARGIN)
    _panel(frame, _MARGIN + _LINE_HEIGHT * len(lines), panel_width)

    y = _MARGIN + _LINE_HEIGHT - 6
    for text, colour in lines:
        cv2.putText(frame, text, (_MARGIN, y), _FONT, _SCALE, colour, _THICKNESS, cv2.LINE_AA)
        y += _LINE_HEIGHT

    return frame


def draw_crosshair(frame: np.ndarray, colour: tuple[int, int, int] = WHITE) -> np.ndarray:
    """Mark the frame centre - the reference point for yaw alignment later.
    標出畫面中心——之後偏航對齊的參考點。
    """
    height, width = frame.shape[:2]
    cx, cy = width // 2, height // 2
    cv2.line(frame, (cx - 12, cy), (cx + 12, cy), colour, 1, cv2.LINE_AA)
    cv2.line(frame, (cx, cy - 12), (cx, cy + 12), colour, 1, cv2.LINE_AA)
    return frame


def draw_detections(
    frame: np.ndarray,
    detections: Sequence[Detection],
    target: Optional[Detection] = None,
) -> np.ndarray:
    """Outline every detection, then highlight the selected target.
    先框出所有偵測，再突顯被選中的目標。

    Non-targets are drawn too, deliberately: when the drone ignores an object
    you need to see whether it was never detected or detected and rejected.
    非目標也刻意畫出來：無人機忽略某物件時，你必須看得出是「從沒偵測到」還是「偵測到但被拒絕」。
    """
    for detection in detections:
        if target is not None and detection is target:
            continue
        _draw_box(frame, detection, AMBER, 1)

    if target is not None:
        _draw_box(frame, target, GREEN, 2)
        cx, cy = int(target.center_x), int(target.center_y)
        cv2.circle(frame, (cx, cy), 5, GREEN, -1, cv2.LINE_AA)
        # Line from frame centre to target centre: this is the alignment error
        # the yaw controller will act on in Milestone 4.
        # 從畫面中心連到目標中心：這就是里程碑 4 偏航控制器會處理的對齊誤差。
        height, width = frame.shape[:2]
        cv2.line(frame, (width // 2, height // 2), (cx, cy), GREEN, 1, cv2.LINE_AA)

    return frame


def _draw_box(
    frame: np.ndarray,
    detection: Detection,
    colour: tuple[int, int, int],
    thickness: int,
) -> None:
    x1, y1, x2, y2 = detection.as_int_box()
    cv2.rectangle(frame, (x1, y1), (x2, y2), colour, thickness)

    label = f"{detection.class_name} {detection.confidence:.2f}"
    size, _ = cv2.getTextSize(label, _FONT, _SCALE, _THICKNESS)
    # Keep the label inside the frame when the box touches the top edge.
    # 框貼到畫面頂端時，仍把標籤留在畫面內。
    label_y = max(y1 - 4, size[1] + 4)
    cv2.rectangle(frame, (x1, label_y - size[1] - 4), (x1 + size[0] + 4, label_y), colour, -1)
    cv2.putText(frame, label, (x1 + 2, label_y - 3), _FONT, _SCALE, (0, 0, 0), _THICKNESS, cv2.LINE_AA)


def battery_colour(battery: Optional[int], abort_level: int, takeoff_level: int) -> tuple[int, int, int]:
    if battery is None:
        return AMBER
    if battery <= abort_level:
        return RED
    if battery < takeoff_level:
        return AMBER
    return GREEN


def placeholder_frame(width: int = 960, height: int = 720, message: str = "NO VIDEO") -> np.ndarray:
    """A black frame to show while waiting for the first decoded image.
    等待第一張解碼影像時顯示的黑畫面。
    """
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    size, _ = cv2.getTextSize(message, _FONT, 1.0, 2)
    origin = ((width - size[0]) // 2, (height + size[1]) // 2)
    cv2.putText(frame, message, origin, _FONT, 1.0, RED, 2, cv2.LINE_AA)
    return frame
