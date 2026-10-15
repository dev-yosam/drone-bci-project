"""Milestones 1-4: connect, detect, take off, yaw-scan, yaw-align, then height-align.
里程碑 1-4：連線、偵測、起飛、偏航掃描、偏航對齊，再高度對齊。

Default is still DRY_RUN: the state machine runs and the HUD shows the RC
that *would* be sent, but the motors never spin. Pass ``--live`` only when
you are ready to fly, with the OpenCV window focused and a clear floor.
預設仍是 DRY_RUN：狀態機會跑、HUD 會顯示「本來會送」的 RC，但馬達不會轉。
只有準備好要飛、OpenCV 視窗在前景、地面淨空時，才加 ``--live``。

Run:  # 執行方式
    python main.py                       # station mode, DRY_RUN  # Station 模式，模擬飛行
    python main.py --live                # real takeoff / scan / align  # 真正起飛／掃描／對齊
    python main.py --host 192.168.10.1   # direct AP mode  # 直連 AP 模式
    python main.py --no-video            # telemetry only  # 只看遙測
    python main.py --no-detect           # video without YOLO  # 有影像但不跑 YOLO
    python main.py --colour              # enable the red-cup filter  # 開啟紅杯顏色濾鏡

Keys (the OpenCV window must have focus):  # 按鍵（OpenCV 視窗必須在前景）
    T        takeoff, hover, yaw scan, then align  # 起飛 → 懸停 → 偏航掃描 → 對齊
    L        land  # 降落
    E        emergency (motors off, the drone drops)  # 緊急停止（切馬達，無人機會掉下來）
    Q / Esc  quit and shut down  # 結束並安全關閉
    B        simulate an ErrP trigger (hover)  # 模擬 ErrP（改為懸停）
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
from navigation.flight_supervisor import FlightState, FlightSupervisor, SupervisorInputs
from utils import visualization as viz
from utils.logger import FlightCsvLogger, setup_logging
from vision.detection import Detection
from vision.target_selector import SelectionResult, TargetSelector
from vision.yolo_detector import try_create as try_create_detector

LOGGER = logging.getLogger("main")

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_PORT_BUSY = 2

_STATE_COLOUR = {
    FlightState.GROUND: viz.GREEN,
    FlightState.TAKEOFF: viz.AMBER,
    FlightState.HOVER_SETTLE: viz.AMBER,
    FlightState.SCANNING: viz.AMBER,
    FlightState.ALIGNING: viz.AMBER,
    FlightState.TARGET_LOCKED: viz.GREEN,
    FlightState.SCAN_TIMEOUT: viz.AMBER,
    FlightState.BCI_STOP: viz.RED,
    FlightState.VIDEO_LOST: viz.RED,
    FlightState.LANDING: viz.AMBER,
    FlightState.ABORT: viz.RED,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tello BCI project - Milestones 1-4")
    parser.add_argument("--host", default=config.TELLO_HOST, help="Tello IP address / Tello IP 位址")
    parser.add_argument(
        "--live",
        action="store_true",
        help="Disable DRY_RUN and allow takeoff / scan / align. / 關閉 DRY_RUN，允許起飛、掃描與對齊。",
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
    supervisor: FlightSupervisor,
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
    height = controller.height_cm
    lr, fb, ud, yaw = controller.last_rc
    state = supervisor.state
    lines = [
        (f"STATE: {state.value}", _STATE_COLOUR.get(state, viz.WHITE)),
        (
            f"Battery: {battery if battery is not None else '--'}%   "
            f"H: {height if height is not None else '--'} cm",
            viz.battery_colour(battery, config.MIN_BATTERY_ABORT, config.MIN_BATTERY_TAKEOFF),
        ),
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
        height_px, width = frame_shape
        target = selection.target
        if target is not None:
            error_x, error_y = target.error_from_centre(width, height_px)
            lines.append((f"Target: {target.class_name} {target.confidence:.2f}", viz.GREEN))
            lines.append(
                (
                    f"BBox ratio: {target.area_ratio(width, height_px):.4f}  "
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

    lines.append((f"RC yaw={yaw:+d}  lr={lr:+d} fb={fb:+d} ud={ud:+d}", viz.WHITE))
    if supervisor.safety_trigger:
        lines.append((f"Safety: {supervisor.safety_trigger}", viz.RED))

    lines.append(
        (f"BCI: {bci.state} p={bci.errp_probability:.2f}",
         viz.RED if bci.stopped else viz.GREEN)
    )
    lines.append(
        ("MODE: DRY_RUN (no movement)", viz.AMBER) if controller.dry_run
        else ("MODE: LIVE", viz.RED)
    )
    lines.append(("T takeoff  L land  E emergency  Q quit  B ErrP  R  C", viz.WHITE))
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

    if args.live and not video_on:
        LOGGER.error("LIVE mode requires video. Do not pass --no-video.")
        return EXIT_ERROR

    if args.live and config.DRY_RUN:
        LOGGER.warning("LIVE MODE: motors can spin. Keep a clear floor and the window focused.")
    if dry_run:
        LOGGER.info("DRY_RUN is on. Press T to walk the flight states without taking off.")

    controller = TelloController(host=args.host, dry_run=dry_run)
    supervisor = FlightSupervisor(controller)
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
            if args.live:
                LOGGER.error(
                    "LIVE mode will not fly without a camera. Power-cycle the Tello, "
                    "confirm Wi-Fi, then retry. Do not press T."
                )
                LOGGER.error(
                    "LIVE 沒有影像就不能飛。請重開 Tello、確認 Wi-Fi 後再試。不要按 T。"
                )
                controller.shutdown()
                return EXIT_ERROR
            LOGGER.error("Continuing without video. See the README for the AP-mode fallback.")
            video_on = False
            detect_on = False

    cv2.namedWindow(config.WINDOW_NAME, cv2.WINDOW_NORMAL)

    with FlightCsvLogger(config.LOG_DIR, config.CSV_LOG_ENABLED) as csv_log:
        try:
            while True:
                loop_fps.tick()
                detection_tick = False

                result = controller.read_frame() if video_on else FrameResult(None, False)
                frame = result.frame

                if result.is_new:
                    video_fps.tick()
                    frame_count += 1

                    if (
                        detector is not None
                        and selector is not None
                        and frame_count % config.DETECT_EVERY_N_FRAMES == 0
                    ):
                        detections = detector.detect(frame)
                        selection = selector.select(detections, frame)
                        detection_tick = True

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

                # Pixel offset from frame centre: +x is right, +y is below.
                # 相對畫面中心的像素偏移：+x 在右，+y 在下。
                error_x = None
                error_y = None
                if selection.target is not None:
                    error_x, error_y = selection.target.error_from_centre(
                        frame.shape[1], frame.shape[0]
                    )

                supervisor.step(
                    SupervisorInputs(
                        now=time.time(),
                        target_found=selection.target is not None,
                        detection_tick=detection_tick,
                        bci_stopped=bci.stopped,
                        video_ok=controller.video_ok if video_on else True,
                        video_on=video_on,
                        battery=controller.battery,
                        error_x=error_x,
                        error_y=error_y,
                    )
                )

                frame_shape = (frame.shape[0], frame.shape[1])

                viz.draw_crosshair(frame)
                if detect_on:
                    viz.draw_detections(frame, detections, selection.target)
                viz.draw_hud(
                    frame,
                    build_hud_lines(
                        supervisor, controller, loop_fps.rate, video_fps.rate, bci,
                        video_on, selector, selection, frame_shape,
                    ),
                )

                if config.DISPLAY_SCALE != 1.0:
                    frame = cv2.resize(frame, None, fx=config.DISPLAY_SCALE, fy=config.DISPLAY_SCALE)

                cv2.imshow(config.WINDOW_NAME, frame)

                lr, fb, ud, yaw = controller.last_rc
                csv_log.log(
                    state=supervisor.state.value,
                    battery=controller.battery if controller.battery is not None else "",
                    fps=round(video_fps.rate, 2),
                    video_ok=int(controller.video_ok) if video_on else "",
                    bci_state=bci.state,
                    errp_probability=bci.errp_probability,
                    cmd_lr=lr,
                    cmd_fb=fb,
                    cmd_ud=ud,
                    cmd_yaw=yaw,
                    safety_trigger=supervisor.safety_trigger,
                    **csv_row_for(selection, frame_shape),
                )

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    LOGGER.info("Quit requested")
                    if supervisor.in_air:
                        supervisor.request_land()
                    break
                if key in (ord("t"), ord("T")):
                    supervisor.request_takeoff()
                if key == ord("l"):
                    supervisor.request_land()
                if key == ord("e"):
                    supervisor.request_emergency()
                if key == ord("b"):
                    bci.trigger()
                if key == ord("r"):
                    bci.reset()
                if key == ord("c") and selector is not None:
                    selector.colour_filter_enabled = not selector.colour_filter_enabled
                    LOGGER.info("Colour filter %s", "on" if selector.colour_filter_enabled else "off")

                if cv2.getWindowProperty(config.WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    LOGGER.info("Window closed")
                    if supervisor.in_air:
                        supervisor.request_land()
                    break

        except KeyboardInterrupt:
            LOGGER.info("Interrupted from the keyboard")
            if supervisor.in_air:
                supervisor.request_land()
        except Exception:  # noqa: BLE001 - log then fall through to shutdown
            LOGGER.exception("Unhandled error in the main loop")
            return EXIT_ERROR
        finally:
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
