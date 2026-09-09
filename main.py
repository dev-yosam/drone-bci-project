"""Milestones 1-2: connect, stream video, detect the target object, display.
里程碑 1-2：連線、串流影像、偵測目標物件、顯示畫面。

Still no autonomy. The drone never takes off and no movement command is ever
sent - the point of Milestone 2 is to tune detection while standing on the
ground with the object in your hand.
目前仍無自主飛行。無人機不會起飛，也不會送出任何移動指令——里程碑 2 的目的是
人站在地上、手上拿著目標物，先把偵測調好。

Run:  # 執行方式
    python main.py                       # station mode host from config.py  # Station 模式，主機 IP 來自 config.py
    python main.py --host 192.168.10.1   # direct AP mode  # 直連 AP 模式
    python main.py --no-video            # telemetry only  # 只看遙測、不要影像
    python main.py --no-detect           # video without YOLO  # 有影像但不跑 YOLO
    python main.py --colour              # enable the red-cup filter  # 開啟紅杯顏色濾鏡

Keys (the OpenCV window must have focus):  # 按鍵（OpenCV 視窗必須在前景）
    Q / Esc  quit and shut down  # 結束並安全關閉
    L        land (no-op here, verifies the command path)  # 降落（此階段無作用，只驗證指令路徑）
    B        simulate an ErrP trigger (HUD only for now)  # 模擬 ErrP 觸發（目前只改 HUD）
    R        clear the simulated BCI stop  # 清除模擬的 BCI 停止
    C        toggle the colour filter at runtime  # 執行中切換顏色濾鏡
"""

from __future__ import annotations

import argparse
import logging
import socket
import sys
import time
from collections import deque
from typing import Optional

import cv2
from djitellopy import Tello, TelloException

import config
from drone.tello_controller import FrameResult, TelloController
from utils import visualization as viz
from utils.logger import FlightCsvLogger, setup_logging
from vision.detection import Detection
from vision.target_selector import SelectionResult, TargetSelector
from vision.yolo_detector import try_create as try_create_detector

LOGGER = logging.getLogger("main")

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_PORT_BUSY = 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tello BCI project - Milestones 1-2")
    parser.add_argument("--host", default=config.TELLO_HOST, help="Tello IP address / Tello IP 位址")
    parser.add_argument(
        "--live",
        action="store_true",
        help="Disable DRY_RUN. No movement is implemented yet regardless. / 關閉 DRY_RUN。此階段仍沒有移動指令。",
    )
    parser.add_argument("--no-video", action="store_true", help="Skip the video stream / 略過影像串流")
    parser.add_argument("--no-detect", action="store_true", help="Skip YOLO detection / 略過 YOLO 偵測")
    parser.add_argument(
        "--colour",
        action="store_true",
        help="Enable the colour filter regardless of config / 不論設定檔為何都開啟顏色濾鏡",
    )
    parser.add_argument(
        "--target",
        default=config.TARGET_OBJECT,
        help="COCO class to look for (default from config) / 要尋找的 COCO 類別（預設來自設定檔）",
    )
    return parser.parse_args()


def control_port_available(port: int = Tello.CONTROL_UDP_PORT) -> bool:
    """Check UDP 8889 before djitellopy grabs it.
    在 djitellopy 佔用 UDP 8889 之前先檢查埠是否空閒。

    A leftover Python process holding this port is by far the most common
    startup failure here, and djitellopy's own error (WinError 10048) does not
    say how to fix it. Checking first lets us print something actionable.
    最常見的啟動失敗，就是上一支 Python 行程還沒結束、仍佔著這個埠；
    而 djitellopy 自己的錯誤（WinError 10048）不會告訴你怎麼修。先檢查才能印出可執行的排除步驟。
    """
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(("", port))
        return True
    except OSError:
        return False
    finally:
        probe.close()


def port_busy_message(port: int = Tello.CONTROL_UDP_PORT) -> str:
    return (
        f"UDP port {port} is already in use, so the Tello link cannot be opened.\n"
        "Usually a previous run is still alive, or Packet Sender is open.\n"
        "Fix it with:\n"
        f"    netstat -ano | findstr :{port}\n"
        "    taskkill /PID <the PID from the last column> /F"
    )


class BciSimulator:
    """Stand-in for the Stage 2 ErrP detector.
    第二階段 ErrP 偵測器的替身。

    Exposes the same single value the real interface will
    (``errp_probability``) so the navigation code never has to care whether the
    signal came from EEG or from a key press.
    對外只暴露真實介面也會有的那一個值（``errp_probability``），
    讓導航程式不必在意訊號來自 EEG 還是按鍵。
    """

    def __init__(self) -> None:
        self.stopped = False

    def trigger(self) -> None:
        self.stopped = True
        LOGGER.warning("Simulated ErrP trigger -> BCI_STOP")

    def reset(self) -> None:
        if self.stopped:
            LOGGER.info("BCI stop cleared")
        self.stopped = False

    @property
    def errp_probability(self) -> float:
        return 0.95 if self.stopped else 0.0

    @property
    def state(self) -> str:
        return "ERRP_STOP" if self.stopped else "NORMAL"


class FpsMeter:
    """Rolling rate over the last ``window`` ticks.
    用最近 ``window`` 次計時計算滑動幀率。
    """

    def __init__(self, window: int = 30) -> None:
        self._times: deque[float] = deque(maxlen=window)

    def tick(self) -> None:
        self._times.append(time.time())

    @property
    def rate(self) -> float:
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        return (len(self._times) - 1) / span if span > 0 else 0.0


def build_hud_lines(
    state: str,
    controller: TelloController,
    loop_fps: float,
    video_fps: float,
    bci: BciSimulator,
    video_on: bool,
    selector: Optional[TargetSelector],
    selection: SelectionResult,
    frame_shape: tuple[int, int],
) -> list[tuple[str, tuple[int, int, int]]]:
    battery = controller.battery
    lines = [
        (f"STATE: {state}", viz.GREEN if state == "READY" else viz.AMBER),
        (
            f"Battery: {battery if battery is not None else '--'}%",
            viz.battery_colour(battery, config.MIN_BATTERY_ABORT, config.MIN_BATTERY_TAKEOFF),
        ),
        # Two separate rates: the loop runs faster than the 30 fps the Tello
        # sends, so a single number would overstate how fresh the image is.
        # 兩個獨立幀率：主迴圈通常比 Tello 送來的 30 fps 還快，
        # 只顯示一個數字會高估影像有多新鮮。
        (f"Loop: {loop_fps:5.1f} fps   Video: {video_fps:5.1f} fps", viz.WHITE),
    ]

    if video_on:
        lines.append(
            ("Link: OK", viz.GREEN) if controller.video_ok
            else (f"Link: VIDEO LOST {controller.seconds_since_frame:.1f}s", viz.RED)
        )
    else:
        lines.append(("Link: VIDEO DISABLED", viz.AMBER))

    if selector is None:
        lines.append(("Detect: OFF", viz.AMBER))
    else:
        height, width = frame_shape
        target = selection.target
        if target is not None:
            error_x, error_y = target.error_from_centre(width, height)
            lines.append((f"Target: {target.class_name} {target.confidence:.2f}", viz.GREEN))
            lines.append(
                (
                    f"BBox ratio: {target.area_ratio(width, height):.4f}  "
                    f"err x={error_x:+.0f} y={error_y:+.0f}",
                    viz.GREEN,
                )
            )
            if selection.colour_ratio is not None:
                lines.append((f"Colour match: {selection.colour_ratio:.2f}", viz.GREEN))
        else:
            reason = f"Searching for '{selector.target_class}'"
            if selection.rejected_by_colour:
                reason += f"  ({len(selection.rejected_by_colour)} rejected by colour)"
            elif selection.rejected_by_confidence:
                reason += f"  ({len(selection.rejected_by_confidence)} below confidence)"
            lines.append((reason, viz.AMBER))

        lines.append(
            (
                f"Colour filter: {'ON ' + selector.target_colour if selector.colour_filter_enabled else 'OFF'}",
                viz.WHITE,
            )
        )

    lines.append(
        (f"BCI: {bci.state} p={bci.errp_probability:.2f}",
         viz.RED if bci.stopped else viz.GREEN)
    )
    lines.append(
        ("MODE: DRY_RUN (no movement)", viz.AMBER) if controller.dry_run
        else ("MODE: LIVE", viz.RED)
    )
    lines.append(("Q quit  L land  B ErrP  R reset  C colour", viz.WHITE))
    return lines


def csv_row_for(selection: SelectionResult, frame_shape: tuple[int, int]) -> dict[str, object]:
    """Detection columns for one frame, blank when there is no target.
    單幀的偵測欄位；沒有目標時維持空白。
    """
    target: Optional[Detection] = selection.target
    if target is None:
        return {}

    height, width = frame_shape
    error_x, error_y = target.error_from_centre(width, height)
    return {
        "target_class": target.class_name,
        "confidence": round(target.confidence, 4),
        "x1": round(target.x1, 1),
        "y1": round(target.y1, 1),
        "x2": round(target.x2, 1),
        "y2": round(target.y2, 1),
        "bbox_ratio": round(target.area_ratio(width, height), 5),
        "error_x": round(error_x, 1),
        "error_y": round(error_y, 1),
    }


def run(args: argparse.Namespace) -> int:
    dry_run = config.DRY_RUN and not args.live
    video_on = config.VIDEO_ENABLED and not args.no_video
    detect_on = video_on and not args.no_detect

    controller = TelloController(host=args.host, dry_run=dry_run)
    bci = BciSimulator()
    loop_fps = FpsMeter()
    video_fps = FpsMeter()
    video_loss_logged = False

    selector: Optional[TargetSelector] = None
    detector = None
    detections: list[Detection] = []
    selection = SelectionResult()
    frame_count = 0

    if detect_on:
        # Loading the model can take a few seconds; do it before connecting so
        # a missing package fails fast instead of leaving a live link open.
        # 載入模型可能要幾秒；先做這步再連線，套件缺失會立刻失敗，不會留下一條已開啟的無人機連線。
        detector = try_create_detector(min_confidence=config.TARGET_CONFIDENCE)
        if detector is None:
            LOGGER.warning("Continuing without detection")
            detect_on = False
        else:
            if not detector.supports_class(args.target):
                LOGGER.error(
                    "Class %r is not in this model. Pick one of: %s",
                    args.target,
                    ", ".join(sorted(detector.class_names.values())),
                )
                return EXIT_ERROR
            selector = TargetSelector(
                target_class=args.target,
                colour_filter_enabled=config.COLOUR_FILTER_ENABLED or args.colour,
            )

    try:
        controller.connect()
    except OSError as error:
        # WinError 10048 surfaces here if the port was taken between the probe
        # and the bind.
        # 若探測與真正 bind 之間埠被佔走，WinError 10048 會在這裡出現。
        LOGGER.error("Could not open the control socket: %s", error)
        LOGGER.error(port_busy_message())
        return EXIT_PORT_BUSY
    except TelloException as error:
        LOGGER.error("Tello did not respond: %s", error)
        LOGGER.error(
            "Check that the drone is powered on, that this laptop is on the same "
            "network as %s, and that the battery is not too low (flashing red).",
            args.host,
        )
        return EXIT_ERROR

    if video_on:
        try:
            controller.start_video()
        except TelloException as error:
            LOGGER.error("Video stream failed: %s", error)
            LOGGER.error("Continuing without video. See the README for the AP-mode fallback.")
            video_on = False
            detect_on = False

    cv2.namedWindow(config.WINDOW_NAME, cv2.WINDOW_NORMAL)

    with FlightCsvLogger(config.LOG_DIR, config.CSV_LOG_ENABLED) as csv_log:
        try:
            while True:
                loop_fps.tick()

                result = controller.read_frame() if video_on else FrameResult(None, False)
                frame = result.frame

                if result.is_new:
                    video_fps.tick()
                    frame_count += 1

                    # Inference only on genuinely new frames, and only every
                    # Nth of those. Re-running it on a repeated frame would
                    # burn CPU for an identical answer.
                    # 只對「真的新」的影格推論，而且只跑其中每第 N 張。
                    # 對重複影格再跑一次，只是浪費 CPU、答案完全一樣。
                    if detector is not None and frame_count % config.DETECT_EVERY_N_FRAMES == 0:
                        detections = detector.detect(frame)
                        selection = selector.select(detections, frame)

                if video_on and not controller.video_ok and not video_loss_logged:
                    LOGGER.warning("No new frame for %.1fs", controller.seconds_since_frame)
                    video_loss_logged = True
                elif video_on and controller.video_ok:
                    video_loss_logged = False

                if frame is None:
                    frame = viz.placeholder_frame(
                        message="NO VIDEO" if video_on else "VIDEO DISABLED"
                    )
                    detections, selection = [], SelectionResult()

                frame_shape = (frame.shape[0], frame.shape[1])
                state = "READY" if (not video_on or controller.video_ok) else "VIDEO_LOST"

                viz.draw_crosshair(frame)
                if detect_on:
                    viz.draw_detections(frame, detections, selection.target)
                viz.draw_hud(
                    frame,
                    build_hud_lines(
                        state, controller, loop_fps.rate, video_fps.rate, bci,
                        video_on, selector, selection, frame_shape,
                    ),
                )

                if config.DISPLAY_SCALE != 1.0:
                    frame = cv2.resize(frame, None, fx=config.DISPLAY_SCALE, fy=config.DISPLAY_SCALE)

                cv2.imshow(config.WINDOW_NAME, frame)

                csv_log.log(
                    state=state,
                    battery=controller.battery if controller.battery is not None else "",
                    fps=round(video_fps.rate, 2),
                    video_ok=int(controller.video_ok) if video_on else "",
                    bci_state=bci.state,
                    errp_probability=bci.errp_probability,
                    **csv_row_for(selection, frame_shape),
                )

                # Reading keys from the OpenCV window rather than a global hook
                # means keystrokes only reach the drone when this window has
                # focus. A global listener would fire while typing elsewhere.
                # 從 OpenCV 視窗讀鍵，而不是用全域鍵盤鉤子：只有這個視窗在前景時，
                # 按鍵才會送到無人機。全域監聽會在你打字到別的程式時也觸發。
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    LOGGER.info("Quit requested")
                    break
                if key == ord("l"):
                    LOGGER.info("Land requested")
                    controller.land()
                if key == ord("b"):
                    bci.trigger()
                if key == ord("r"):
                    bci.reset()
                if key == ord("c") and selector is not None:
                    selector.colour_filter_enabled = not selector.colour_filter_enabled
                    LOGGER.info("Colour filter %s", "on" if selector.colour_filter_enabled else "off")

                # The window manager reports this once the user clicks the X.
                # 使用者點了視窗的 X 之後，視窗管理員會回報這個狀態。
                if cv2.getWindowProperty(config.WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    LOGGER.info("Window closed")
                    break

        except KeyboardInterrupt:
            LOGGER.info("Interrupted from the keyboard")
        except Exception:  # noqa: BLE001 - log then fall through to shutdown  # 先記錄，再落到 shutdown
            LOGGER.exception("Unhandled error in the main loop")
            return EXIT_ERROR
        finally:
            # Runs on every exit path, including exceptions, so the drone link
            # and the UDP ports are always released.
            # 任何結束路徑都會跑到這裡（包含例外），確保無人機連線與 UDP 埠一定會釋放。
            controller.shutdown()
            cv2.destroyAllWindows()

    return EXIT_OK


def main() -> int:
    args = parse_args()

    log_path = setup_logging(config.LOG_LEVEL, config.LOG_DIR, config.TELLO_LOG_LEVEL)
    LOGGER.info("Starting. Log file: %s", log_path)

    if not control_port_available():
        LOGGER.error(port_busy_message())
        return EXIT_PORT_BUSY

    return run(args)


if __name__ == "__main__":
    sys.exit(main())
