"""The single detection record passed between perception and navigation.
感知層與導航層之間傳遞的單一偵測紀錄。

Geometry is exposed as properties derived from the corner coordinates rather
than stored as separate fields. Storing ``width`` next to ``x1``/``x2`` invites
the two going out of sync after any edit; deriving them makes that impossible.
The dataclass is frozen for the same reason - a detection is a measurement, and
nothing downstream should be rewriting it.
幾何量以「由角落座標推導」的屬性公開，而不是另外存一份欄位。若同時存 ``width`` 與 ``x1``/``x2``，
之後一改就容易對不上；推導則不可能不同步。dataclass 設成 frozen 也是同一理由——偵測是一次量測，下游不該改寫它。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Detection:
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def center_x(self) -> float:
        return (self.x1 + self.x2) / 2.0

    @property
    def center_y(self) -> float:
        return (self.y1 + self.y2) / 2.0

    @property
    def area(self) -> float:
        return self.width * self.height

    def area_ratio(self, frame_width: int, frame_height: int) -> float:
        """Bounding-box area as a fraction of the frame.
        框面積佔整張畫面的比例。

        This is the proximity proxy used by the approach controller. It is NOT
        a distance: the same ratio means very different real distances for
        objects of different physical size.
        這是接近控制器用的「遠近代理指標」。它不是距離：同樣的比例，對不同實體大小的物件，代表完全不同的真實距離。
        """
        frame_area = float(frame_width * frame_height)
        return self.area / frame_area if frame_area > 0 else 0.0

    def error_from_centre(self, frame_width: int, frame_height: int) -> tuple[float, float]:
        """Pixel offset of the target from the frame centre.
        目標相對畫面中心的像素偏移。

        Positive ``error_x`` means the target sits to the right, which the yaw
        controller answers with a clockwise rotation. Positive ``error_y`` means
        the target sits below centre (image y grows downward); after yaw lock
        the height controller descends.
        ``error_x`` 為正代表目標在右邊，偏航控制器會以順時針旋轉回應。
        ``error_y`` 為正代表目標在畫面中心下方（影像 y 向下增加）；偏航鎖定後高度控制器會下降。
        """
        return (
            self.center_x - frame_width / 2.0,
            self.center_y - frame_height / 2.0,
        )

    def as_int_box(self) -> tuple[int, int, int, int]:
        return int(self.x1), int(self.y1), int(self.x2), int(self.y2)
