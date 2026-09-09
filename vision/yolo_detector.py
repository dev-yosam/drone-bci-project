"""Ultralytics YOLO wrapper.
Ultralytics YOLO 的包裝層。

Deliberately knows nothing about the Tello. It takes a BGR frame and returns
``Detection`` objects, which means the whole perception path can be exercised
on a webcam, a saved video or a still image with no drone present.
刻意完全不知道 Tello。它吃一張 BGR 影格、回傳 ``Detection`` 清單，因此整條感知路徑
可以在網路攝影機、存檔影片或靜態圖片上驗證，不必有無人機。

Switching models is a config change only:
換模型只改設定：
    MODEL_PATH = "yolo11n.pt"   # pretrained COCO  # 預訓練 COCO
    MODEL_PATH = "best.pt"      # your own fine-tuned weights  # 你自己微調的權重
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

import numpy as np

import config
from vision.detection import Detection

LOGGER = logging.getLogger(__name__)


def resolve_model_path(model_path: str) -> str:
    """Turn a bare weight filename into a path inside the project.
    把單純的權重檔名轉成專案內路徑。

    Ultralytics downloads unknown weights on first use, which fails on the
    Tello router because it has no internet. Pinning the location to
    ``models/`` means a weight file that was fetched once keeps working
    offline, and a missing one fails immediately with a clear path.
    Ultralytics 第一次遇到未知權重會下載，但 Tello 路由器沒有對外網路，下載會失敗。
    把位置釘在 ``models/``：抓過一次就能離線用；缺檔則立刻失敗，並帶出清楚路徑。
    """
    candidate = Path(model_path)
    if candidate.is_absolute() or candidate.parent != Path("."):
        return str(candidate)

    local = config.MODEL_DIR / candidate.name
    return str(local) if local.exists() else model_path


class YoloDetector:
    def __init__(
        self,
        model_path: str = config.MODEL_PATH,
        device: str = config.YOLO_DEVICE,
        img_size: int = config.YOLO_IMG_SIZE,
        min_confidence: float = config.TARGET_CONFIDENCE,
    ) -> None:
        # Imported here rather than at module scope so that the rest of the
        # project (and the unit tests) can be imported without ultralytics and
        # its multi-gigabyte torch dependency installed.
        # 放在這裡 import，而不是模組頂層，這樣專案其餘部分（以及單元測試）
        # 即使沒裝 ultralytics 與數 GB 的 torch，也仍能被匯入。
        try:
            from ultralytics import YOLO
        except ImportError as error:  # pragma: no cover - environment issue  # 環境問題，不納入覆蓋率
            raise ImportError(
                "ultralytics is not installed. Run:\n"
                "    pip install ultralytics"
            ) from error

        self.model_path = resolve_model_path(model_path)
        self.device = device
        self.img_size = img_size
        # Filtering at the model keeps obviously weak boxes out of the pipeline.
        # The selector applies the same threshold again, which is harmless and
        # keeps the selector independently testable.
        # 在模型端先過濾，明顯太弱的框就不會進管線。
        # 選擇器會再套一次同樣門檻，無害，而且讓選擇器能獨立測試。
        self.min_confidence = min_confidence

        LOGGER.info("Loading YOLO model %s on %s", self.model_path, device)
        self.model = YOLO(self.model_path)
        self.class_names: dict[int, str] = dict(self.model.names)
        LOGGER.info("Model loaded with %d classes", len(self.class_names))

    def supports_class(self, class_name: str) -> bool:
        return class_name in self.class_names.values()

    def detect(self, frame_bgr: np.ndarray) -> list[Detection]:
        """Run inference on one BGR frame.
        對一張 BGR 影格做推論。

        Ultralytics treats a raw numpy array as BGR, which matches what
        TelloController hands out, so no conversion is needed here.
        Ultralytics 把原始 numpy 陣列當成 BGR，這正好是 TelloController 交出的格式，這裡不必再轉。
        """
        if frame_bgr is None or frame_bgr.size == 0:
            return []

        results = self.model.predict(
            source=frame_bgr,
            imgsz=self.img_size,
            conf=self.min_confidence,
            device=self.device,
            verbose=False,
        )
        if not results:
            return []

        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []

        xyxy = boxes.xyxy.cpu().numpy()
        confidences = boxes.conf.cpu().numpy()
        class_ids = boxes.cls.cpu().numpy().astype(int)
        names = getattr(result, "names", self.class_names)

        detections: list[Detection] = []
        for (x1, y1, x2, y2), confidence, class_id in zip(xyxy, confidences, class_ids):
            detections.append(
                Detection(
                    class_name=names.get(int(class_id), str(class_id)),
                    confidence=float(confidence),
                    x1=float(x1),
                    y1=float(y1),
                    x2=float(x2),
                    y2=float(y2),
                )
            )
        return detections


def try_create(**kwargs) -> Optional[YoloDetector]:
    """Build a detector, or return None with a logged reason.
    建立偵測器；失敗則回傳 None 並寫下原因。

    Milestone 1 behaviour must keep working when YOLO is unavailable: a missing
    package or a failed weight download should degrade to a plain video feed,
    not crash a session with a drone in the air.
    YOLO 不可用時，里程碑 1 的行為仍須能跑：套件缺失或權重下載失敗，應降級成純影像，
    而不是在無人機還在空中時把整次執行當掉。
    """
    try:
        return YoloDetector(**kwargs)
    except Exception as error:  # noqa: BLE001 - any failure means "no detector"  # 任何失敗都當成「沒有偵測器」
        LOGGER.error("YOLO detector unavailable: %s", error)
        return None
