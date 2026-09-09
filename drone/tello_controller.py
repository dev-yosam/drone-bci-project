"""Thin, defensive wrapper around djitellopy's Tello.
包在 djitellopy 的 Tello 外面的薄層、防禦性包裝。

This is the ONLY module allowed to talk to the drone. Every other module
produces data or desired actions; the controller decides whether they actually
reach the hardware. Centralising it here gives us three things:
這是唯一允許跟無人機通訊的模組。其他模組只產生資料或「想做的動作」；
要不要真的送到硬體，由這個控制器決定。集中在這裡有三個好處：

1. A single place where ``config.DRY_RUN`` is enforced, so no other module can
   accidentally move the drone.
   只有這裡會執行 ``config.DRY_RUN``，其他模組不可能不小心讓無人機移動。
2. A single UDP client, avoiding the "port 8889 already in use" failures that
   happen when two Tello objects exist at once.
   全專案只有一個 UDP 用戶端，避免同時存在兩個 Tello 物件時出現「8889 已被佔用」。
3. Idempotent shutdown, so an exception anywhere in the main loop still leaves
   the drone in a safe state.
   關閉可以重複呼叫；主迴圈任何地方出錯，無人機仍會回到安全狀態。

Milestone 1 scope: connect, stream video, read telemetry, shut down safely.
No takeoff and no movement commands are implemented yet - ``land`` and
``emergency`` exist only because a safe shutdown path needs them.
里程碑 1 範圍：連線、串流影像、讀遙測、安全關閉。
尚未實作起飛與移動——``land`` 與 ``emergency`` 存在，只因為安全關閉路徑需要它們。
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from djitellopy import Tello, TelloException

import config

LOGGER = logging.getLogger(__name__)

# djitellopy takes these as opaque strings; map from the readable config values
# so config.py does not have to import djitellopy.
# djitellopy 把這些當成不透明字串；在這裡把好讀的設定值對應過去，config.py 就不必匯入 djitellopy。
_RESOLUTIONS = {"480p": Tello.RESOLUTION_480P, "720p": Tello.RESOLUTION_720P}
_FPS = {"5": Tello.FPS_5, "15": Tello.FPS_15, "30": Tello.FPS_30}

# BackgroundFrameRead pre-fills this placeholder before the first real frame
# arrives, so a frame of exactly this shape means "decoder not producing yet".
# BackgroundFrameRead 在第一張真影格到來前會先填這個佔位圖；
# 尺寸剛好是這個，就代表解碼器還沒產出畫面。
_PLACEHOLDER_SHAPE = (300, 400, 3)


@dataclass
class FrameResult:
    """One read from the video pipeline.
    影像管線的一次讀取結果。

    ``is_new`` distinguishes "the decoder handed us a fresh frame" from "we are
    looking at the previous frame again", which is what video-loss detection
    needs. ``frame`` is BGR and may be None before the first frame arrives.
    ``is_new`` 用來分辨「解碼器給了新影格」還是「又看到上一張」，這正是判斷影像中斷所需。
    ``frame`` 是 BGR，第一張影格到達前可能是 None。
    """

    frame: Optional[np.ndarray]
    is_new: bool


class TelloController:
    def __init__(self, host: str = config.TELLO_HOST, dry_run: bool = config.DRY_RUN) -> None:
        self.host = host
        self.dry_run = dry_run

        self.tello: Optional[Tello] = None
        self._frame_read = None
        self._last_raw_frame: Optional[np.ndarray] = None
        self._last_frame_time: float = 0.0
        self._first_frame_seen: bool = False

        self._telemetry_cache: dict[str, tuple[float, object]] = {}
        self._is_shutdown: bool = False

    # -- connection ---------------------------------------------------------
    # -- 連線 ---------------------------------------------------------------

    def connect(self) -> None:
        """Open the control link and verify the drone answers.
        打開控制連線，並確認無人機有回應。

        Raises OSError if UDP 8889 is already taken (a leftover Python process
        is the usual cause) and TelloException if the drone does not respond.
        若 UDP 8889 已被佔用（通常是上一支 Python 還沒結束）會拋 OSError；
        無人機沒回應則拋 TelloException。
        """
        LOGGER.info("Connecting to Tello at %s (dry_run=%s)", self.host, self.dry_run)

        # Constructing Tello binds UDP 8889 and starts the receiver threads.
        # 建立 Tello 物件時就會 bind UDP 8889，並啟動接收執行緒。
        self.tello = Tello(host=self.host, retry_count=config.CONNECT_RETRY_COUNT)
        self.tello.connect()

        LOGGER.info("Connected. Battery %s%%, SDK %s", self.battery, self.sdk_version)

    # -- video --------------------------------------------------------------
    # -- 影像 ---------------------------------------------------------------

    def start_video(self) -> None:
        """Turn the camera on and wait for the decoder to produce a frame.
        開啟相機，並等到解碼器產出第一張影格。

        Note: djitellopy's own docstring warns that Tello EDU may not support
        streaming while joined to a WiFi network (station mode). If that is the
        case on this firmware, this call raises and the fix is to switch to
        direct AP mode - see the README.
        注意：djitellopy 自己的文件警告，Tello EDU 在加入 Wi-Fi（Station 模式）時
        可能不支援串流。若這版韌體就是如此，這個呼叫會失敗，解法是改直連 AP 模式——見 README。
        """
        if self.tello is None:
            raise RuntimeError("connect() must be called before start_video()")

        if config.APPLY_VIDEO_SETTINGS:
            self._apply_video_settings()

        self.tello.streamon()
        # get_frame_read() opens the UDP H.264 stream with PyAV and starts the
        # decode thread. It raises TelloException if no frames arrive.
        # get_frame_read() 用 PyAV 打開 UDP H.264 串流並啟動解碼執行緒。
        # 若沒有影格到來會拋 TelloException。
        self._frame_read = self.tello.get_frame_read()

        self._wait_for_first_frame()

    def _apply_video_settings(self) -> None:
        """Best-effort resolution/fps/bitrate setup.
        盡力設定解析度 / 幀率 / 位元率。

        These are SDK 3.0 commands. Older firmware rejects them, and that is
        not a reason to abort the run, so failures are logged and ignored.
        這些是 SDK 3.0 指令。較舊韌體會拒絕，這不該讓整次執行中止，所以失敗只記錄、不中斷。
        """
        assert self.tello is not None
        settings = (
            ("resolution", lambda: self.tello.set_video_resolution(_RESOLUTIONS[config.VIDEO_RESOLUTION])),
            ("fps", lambda: self.tello.set_video_fps(_FPS[config.VIDEO_FPS])),
            ("bitrate", lambda: self.tello.set_video_bitrate(config.VIDEO_BITRATE_MBPS)),
        )
        for name, apply in settings:
            try:
                apply()
            except (TelloException, KeyError) as error:
                LOGGER.warning("Could not set video %s: %s", name, error)

    def _wait_for_first_frame(self) -> None:
        deadline = time.time() + config.FIRST_FRAME_TIMEOUT_S
        while time.time() < deadline:
            if self.read_frame().is_new:
                LOGGER.info("First video frame received")
                return
            time.sleep(0.1)

        raise TelloException(
            f"No video frame within {config.FIRST_FRAME_TIMEOUT_S:.0f}s. "
            "In station mode this often means the firmware does not stream over "
            "a router; try direct AP mode. Also check that Windows Firewall "
            "allows inbound UDP 11111."
        )

    def read_frame(self) -> FrameResult:
        """Return the newest frame as BGR.
        回傳最新影格，格式為 BGR。

        djitellopy hands out RGB arrays (they come from PIL), so they must be
        converted before anything OpenCV-based touches them. Getting this wrong
        silently breaks the red-cup colour filter later on.
        djitellopy 交出的是 RGB 陣列（來自 PIL），任何 OpenCV 程式碰到之前都必須轉換。
        這裡弄錯，後面的紅杯顏色濾鏡會默默失效。
        """
        if self._frame_read is None:
            return FrameResult(None, False)

        raw = self._frame_read.frame
        if raw is None:
            return FrameResult(None, False)

        if not self._first_frame_seen and raw.shape == _PLACEHOLDER_SHAPE:
            # Placeholder that BackgroundFrameRead allocates at construction.
            # BackgroundFrameRead 建立時預先配置的佔位圖。
            return FrameResult(None, False)

        # The decode thread rebinds .frame to a new array per frame, so object
        # identity is a cheap and reliable "did anything arrive" test.
        # 解碼執行緒每張影格都會把 .frame 指到新陣列，所以用物件識別碼就能便宜、可靠地判斷「有沒有新東西來」。
        is_new = raw is not self._last_raw_frame
        if is_new:
            self._last_raw_frame = raw
            self._last_frame_time = time.time()
            self._first_frame_seen = True

        return FrameResult(cv2.cvtColor(raw, cv2.COLOR_RGB2BGR), is_new)

    @property
    def seconds_since_frame(self) -> float:
        if self._last_frame_time == 0.0:
            return float("inf")
        return time.time() - self._last_frame_time

    @property
    def video_ok(self) -> bool:
        return self.seconds_since_frame < config.VIDEO_LOSS_TIMEOUT_S

    # -- telemetry ----------------------------------------------------------
    # -- 遙測 ---------------------------------------------------------------

    def _cached(self, key: str, read, ttl: float = 1.0, default=None):
        """Read a telemetry field at most once per ttl seconds.
        同一個遙測欄位，最多每 ttl 秒讀一次。

        The state fields come from a background UDP thread, so reads are cheap,
        but they raise while the first state packet is still in flight. Caching
        plus a default keeps the HUD from flickering or crashing on startup.
        狀態欄位來自背景 UDP 執行緒，讀取本身很便宜，但第一包狀態還沒到時會拋錯。
        快取加上預設值，可避免啟動時 HUD 閃爍或當掉。
        """
        now = time.time()
        cached = self._telemetry_cache.get(key)
        if cached is not None and now - cached[0] < ttl:
            return cached[1]

        try:
            value = read()
        except (TelloException, KeyError, ValueError) as error:
            LOGGER.debug("Telemetry %s unavailable: %s", key, error)
            return cached[1] if cached is not None else default

        self._telemetry_cache[key] = (now, value)
        return value

    @property
    def battery(self) -> Optional[int]:
        if self.tello is None:
            return None
        return self._cached("battery", self.tello.get_battery)

    @property
    def height_cm(self) -> Optional[int]:
        if self.tello is None:
            return None
        return self._cached("height", self.tello.get_height)

    @property
    def temperature_c(self) -> Optional[float]:
        if self.tello is None:
            return None
        return self._cached("temperature", self.tello.get_temperature)

    @property
    def sdk_version(self) -> Optional[str]:
        if self.tello is None:
            return None
        # Queried over the command channel rather than the state stream, so use
        # a long TTL: it never changes during a run.
        # 這項是走指令通道查詢，不是狀態串流，所以 TTL 設很長：一次飛行中它不會變。
        return self._cached("sdk", self.tello.query_sdk_version, ttl=3600.0)

    # -- movement (only what a safe shutdown needs in Milestone 1) ----------
    # -- 移動（里程碑 1 只實作安全關閉所需）--------------------------------

    def land(self) -> None:
        if self.tello is None:
            return
        if self.dry_run:
            LOGGER.info("[DRY_RUN] land() suppressed")
            return
        self.tello.land()

    def emergency(self) -> None:
        """Cut the motors immediately.
        立刻切斷馬達。

        Sent without waiting for a reply so it still goes out while another
        command is blocked on a timeout. The drone will drop, so this is a last
        resort, not a normal stop.
        不等回覆就送出，因此另一條指令卡在逾時時，這條仍能出去。無人機會自由落體，
        所以這是最後手段，不是一般停止。
        """
        if self.tello is None:
            return
        if self.dry_run:
            LOGGER.warning("[DRY_RUN] emergency() suppressed")
            return
        self.tello.emergency()

    # -- shutdown -----------------------------------------------------------
    # -- 關閉 ---------------------------------------------------------------

    def shutdown(self) -> None:
        """Release the drone and the UDP ports. Safe to call more than once.
        釋放無人機與 UDP 埠。可以安全地重複呼叫。
        """
        if self._is_shutdown:
            return
        self._is_shutdown = True

        if self.tello is None:
            return

        LOGGER.info("Shutting down Tello link")
        try:
            # Tello.end() lands if flying, stops the stream and closes sockets.
            # In dry run nothing ever took off, so landing is a no-op anyway.
            # Tello.end() 若在飛會先降落，然後停串流並關閉 socket。
            # dry run 從沒起飛，降落本身也是空操作。
            self.tello.end()
        except Exception as error:  # noqa: BLE001 - shutdown must never raise  # 關閉流程絕不可再拋例外
            LOGGER.warning("Error during shutdown (ignored): %s", error)
        finally:
            self.tello = None
            self._frame_read = None
