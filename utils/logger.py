"""Logging setup and per-frame CSV recording.
紀錄設定，以及逐幀寫入 CSV。

Two separate concerns on purpose:
刻意拆成兩件事：

* ``setup_logging`` handles human-readable diagnostics (console + file).
  ``setup_logging`` 負責給人看的診斷（終端 + 檔案）。
* ``FlightCsvLogger`` writes one row per frame for later research analysis.
  ``FlightCsvLogger`` 每幀寫一列，供之後研究分析。

The CSV schema is fixed now and covers every field the later milestones will
produce. Columns that are not yet filled stay empty rather than appearing
halfway through the project, so recordings from different milestones can be
concatenated without reconciling headers.
CSV 欄位現在就定死，涵蓋後續里程碑會產出的所有欄位。還沒填的欄位維持空白，
而不是做到一半才突然出現新欄，這樣不同里程碑的紀錄才能直接接在一起、不必對表頭。
"""

from __future__ import annotations

import csv
import logging
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

CSV_FIELDS: tuple[str, ...] = (
    "timestamp",
    "elapsed_s",
    "state",
    "battery",
    "fps",
    "video_ok",
    "target_class",
    "confidence",
    "x1",
    "y1",
    "x2",
    "y2",
    "bbox_ratio",
    "error_x",
    "error_y",
    "cmd_lr",
    "cmd_fb",
    "cmd_ud",
    "cmd_yaw",
    "bci_state",
    "errp_probability",
    "safety_trigger",
    "notes",
)


def _run_stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def setup_logging(level: str, log_dir: Path, tello_level: str = "WARNING") -> Path:
    """Configure root logging to console and a timestamped file.
    把根 logger 設成同時輸出到終端與帶時間戳的檔案。

    Returns the log file path. djitellopy is quieted separately because it logs
    every UDP command at INFO, which makes our own messages unreadable.
    回傳日誌檔路徑。djitellopy 另外靜音，因為它會把每條 UDP 指令都打成 INFO，我們自己的訊息會看不見。
    """
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"run_{_run_stamp()}.log"

    root = logging.getLogger()
    root.setLevel(getattr(logging, level.upper(), logging.INFO))
    # Re-running in the same interpreter (e.g. a notebook) would otherwise
    # stack duplicate handlers.
    # 同一個直譯器裡重跑（例如 notebook）否則會疊上重複的 handler。
    for handler in list(root.handlers):
        root.removeHandler(handler)

    fmt = logging.Formatter("%(asctime)s %(levelname)-7s %(name)s | %(message)s", "%H:%M:%S")

    console = logging.StreamHandler(sys.stdout)
    console.setFormatter(fmt)
    root.addHandler(console)

    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(fmt)
    root.addHandler(file_handler)

    # djitellopy attaches its own StreamHandler at import time, which would
    # print every record a second time in its own format. Strip it and let the
    # records propagate to our handlers instead.
    # djitellopy 在 import 時就會掛上自己的 StreamHandler，每筆紀錄會再用它的格式印第二次。
    # 先拆掉，讓紀錄只走我們的 handler。
    tello_logger = logging.getLogger("djitellopy")
    for handler in list(tello_logger.handlers):
        tello_logger.removeHandler(handler)
    tello_logger.setLevel(getattr(logging, tello_level.upper(), logging.WARNING))

    return log_path


class FlightCsvLogger:
    """Append one row per frame. Use as a context manager.
    每幀附加一列。用 context manager 使用。
    """

    def __init__(self, log_dir: Path, enabled: bool = True) -> None:
        self.enabled = enabled
        self.path: Optional[Path] = None
        self._file = None
        self._writer: Optional[csv.DictWriter] = None
        self._start = time.time()

        if not self.enabled:
            return

        log_dir.mkdir(parents=True, exist_ok=True)
        self.path = log_dir / f"flight_{_run_stamp()}.csv"

    def __enter__(self) -> "FlightCsvLogger":
        if self.enabled and self.path is not None:
            self._file = self.path.open("w", newline="", encoding="utf-8")
            self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
            self._writer.writeheader()
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    def log(self, **fields: Any) -> None:
        """Write a row. Unknown keys are rejected loudly to catch typos.
        寫入一列。未知欄位名稱會大聲拒絕，以便抓出拼字錯誤。
        """
        if self._writer is None:
            return

        unknown = set(fields) - set(CSV_FIELDS)
        if unknown:
            raise KeyError(f"Unknown CSV fields: {sorted(unknown)}")

        row: dict[str, Any] = {name: "" for name in CSV_FIELDS}
        row.update(fields)
        row["timestamp"] = datetime.now().isoformat(timespec="milliseconds")
        row["elapsed_s"] = round(time.time() - self._start, 3)

        self._writer.writerow(row)

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None
