"""Pick exactly one detection to fly towards.
從多個偵測中選出「正好一個」要飛過去的目標。

The navigation loop must never see ambiguity: given the same frame it has to
choose the same target every time, otherwise the yaw controller oscillates
between two cups. The ordering below is therefore total - no ties are left to
the order YOLO happened to return boxes in.
導航迴圈絕不能看到歧義：同一張影格每次都必須選同一個目標，否則偏航控制器會在兩只杯子間搖擺。
因此下面的排序是全序——不會把平手留給 YOLO 剛好回傳框的順序。

Selection order:  # 選擇順序
    1. class name must match the target class  # 類別名稱必須符合目標類別
    2. confidence must clear the threshold  # 信心必須超過門檻
    3. optional colour validation  # 可選的顏色驗證
    4. largest bounding-box area  # 最大框面積
    5. tie-break on confidence, then on position  # 平手時比信心，再比位置

Largest area is the primary key rather than highest confidence because area is
what the approach controller consumes, and the biggest instance is the nearest
one. Choosing a smaller, more confident box further away would make the drone
fly past a closer object.
主鍵是最大面積而不是最高信心，因為接近控制器吃的就是面積，而最大的實例通常就是最近的。
若選遠處那只較小、但信心更高的杯子，無人機會飛過近處的目標。
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

import config
from vision.colour_filter import measure_colour
from vision.detection import Detection

LOGGER = logging.getLogger(__name__)


@dataclass
class SelectionResult:
    """What the selector chose, plus why the others were dropped.
    選擇器選了誰，以及其他物件被丟掉的原因。

    The rejected lists exist for debugging: on screen it is otherwise
    impossible to tell "YOLO saw nothing" from "the colour filter rejected
    everything", and those need very different fixes.
    被拒清單是給除錯用的：畫面上否則分不出「YOLO 什麼都沒看到」與「顏色濾鏡全拒」，而這兩種修法完全不同。
    """

    target: Optional[Detection] = None
    candidates: list[Detection] = field(default_factory=list)
    rejected_by_confidence: list[Detection] = field(default_factory=list)
    rejected_by_colour: list[Detection] = field(default_factory=list)
    colour_ratio: Optional[float] = None

    @property
    def found(self) -> bool:
        return self.target is not None


class TargetSelector:
    def __init__(
        self,
        target_class: str = config.TARGET_OBJECT,
        min_confidence: float = config.TARGET_CONFIDENCE,
        colour_filter_enabled: bool = config.COLOUR_FILTER_ENABLED,
        target_colour: str = config.TARGET_COLOUR,
        min_colour_ratio: float = config.COLOUR_MIN_RATIO,
    ) -> None:
        self.target_class = target_class
        self.min_confidence = min_confidence
        self.colour_filter_enabled = colour_filter_enabled
        self.target_colour = target_colour
        self.min_colour_ratio = min_colour_ratio

    def select(self, detections: Sequence[Detection], frame_bgr: np.ndarray) -> SelectionResult:
        result = SelectionResult()

        for detection in detections:
            if detection.class_name != self.target_class:
                continue
            if detection.confidence < self.min_confidence:
                result.rejected_by_confidence.append(detection)
                continue
            result.candidates.append(detection)

        if not result.candidates:
            return result

        accepted: list[tuple[Detection, Optional[float]]] = []
        for detection in result.candidates:
            if not self.colour_filter_enabled:
                accepted.append((detection, None))
                continue

            ratio = measure_colour(frame_bgr, detection, self.target_colour)
            if ratio >= self.min_colour_ratio:
                accepted.append((detection, ratio))
            else:
                result.rejected_by_colour.append(detection)

        if not accepted:
            return result

        # Fully deterministic ordering; the position term only ever matters for
        # two identically sized, identically confident boxes.
        # 完全確定性的排序；位置項只有在兩框面積相同、信心也相同時才會生效。
        best, ratio = max(
            accepted,
            key=lambda item: (
                item[0].area,
                item[0].confidence,
                -item[0].x1,
                -item[0].y1,
            ),
        )
        result.target = best
        result.colour_ratio = ratio
        return result
