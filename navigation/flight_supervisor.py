"""Explicit flight state machine for takeoff, hover, yaw scan, and align.
起飛、懸停、偏航掃描、對齊的明確飛行狀態機。

The supervisor never talks to UDP. It asks a ``TelloController`` to take off,
to send RC, or to land. That keeps ``DRY_RUN`` in one place and makes the
transitions unit-testable with a fake controller.
監督器從不碰 UDP。它只請 ``TelloController`` 起飛、送 RC、或降落。
這樣 ``DRY_RUN`` 仍集中在一處，狀態轉移也可以用假控制器做單元測試。

Milestone 4 yaws on ``error_x`` until the box is centred left-right, then after
``TARGET_LOCKED`` uses ``error_y`` to climb or descend. Milestone 5 flies
forward once both axes are centred, slowing as the box grows, and hovers in
``ARRIVED`` when the box ratio reaches ``STOP_THRESHOLD``.
里程碑 4 先用 ``error_x`` 轉到左右置中，進入 ``TARGET_LOCKED`` 後再用 ``error_y`` 升降。
里程碑 5 在兩軸都置中後往前飛，框越大越慢，框比例到 ``STOP_THRESHOLD`` 就在 ``ARRIVED`` 懸停。
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Protocol

import config

LOGGER = logging.getLogger(__name__)


class FlightState(str, Enum):
    GROUND = "GROUND"
    TAKEOFF = "TAKEOFF"
    HOVER_SETTLE = "HOVER_SETTLE"
    SCANNING = "SCANNING"
    ALIGNING = "ALIGNING"
    TARGET_LOCKED = "TARGET_LOCKED"
    APPROACHING = "APPROACHING"
    ARRIVED = "ARRIVED"
    APPROACH_TIMEOUT = "APPROACH_TIMEOUT"
    SCAN_TIMEOUT = "SCAN_TIMEOUT"
    BCI_STOP = "BCI_STOP"
    VIDEO_LOST = "VIDEO_LOST"
    LANDING = "LANDING"
    ABORT = "ABORT"


class DroneLike(Protocol):
    """The slice of TelloController the supervisor needs.
    監督器需要的 TelloController 介面切片。
    """

    dry_run: bool

    @property
    def busy(self) -> bool: ...

    @property
    def airborne(self) -> bool: ...

    @property
    def last_rc(self) -> tuple[int, int, int, int]: ...

    def takeoff(self) -> bool: ...

    def land(self) -> bool: ...

    def hover(self) -> None: ...

    def send_rc(self, left_right: int, forward_back: int, up_down: int, yaw: int) -> None: ...

    def emergency(self) -> None: ...


@dataclass
class SupervisorInputs:
    now: float
    target_found: bool
    detection_tick: bool
    bci_stopped: bool
    video_ok: bool
    video_on: bool
    battery: Optional[int]
    error_x: Optional[float] = None
    error_y: Optional[float] = None
    bbox_ratio: Optional[float] = None


def _smoothed_p_command(
    error: float,
    smoothed: float,
    *,
    kp: float,
    max_speed: int,
    min_speed: int,
    invert: bool = False,
) -> tuple[int, float]:
    """Shared P + exponential smoother for one RC axis.
    單一 RC 軸共用的 P 控制加指數平滑。
    """
    if abs(error) <= config.ALIGNMENT_TOLERANCE_PX:
        # Deadband: command 0 and decay the smoother so we do not kick out.
        # 死區：指令為 0，並衰減平滑值，避免一出死區就猛轉。
        return 0, config.YAW_SMOOTHING * smoothed

    direction = -1.0 if invert else 1.0
    raw = max(-max_speed, min(max_speed, direction * kp * error))
    alpha = config.YAW_SMOOTHING
    smoothed = alpha * smoothed + (1.0 - alpha) * raw
    cmd = int(round(smoothed))
    cmd = max(-max_speed, min(max_speed, cmd))
    if abs(cmd) < min_speed:
        cmd = min_speed if raw > 0 else -min_speed
        cmd = max(-max_speed, min(max_speed, cmd))
    return cmd, smoothed


def yaw_command_from_error(error_x: float, smoothed: float) -> tuple[int, float]:
    """P-control yaw from horizontal pixel error, with exponential smoothing.
    水平像素誤差的 P 控制偏航，加上指數平滑。

    Positive ``error_x`` means the target is to the right of centre, so yaw is
    clockwise (positive in Tello ``rc``).
    ``error_x`` 為正代表目標在畫面中心右側，所以偏航順時針（Tello ``rc`` 為正）。
    """
    return _smoothed_p_command(
        error_x,
        smoothed,
        kp=config.KP_YAW,
        max_speed=config.MAX_YAW_SPEED,
        min_speed=config.ALIGN_MIN_YAW,
    )


def ud_command_from_error(error_y: float, smoothed: float) -> tuple[int, float]:
    """P-control up/down from vertical pixel error, with exponential smoothing.
    垂直像素誤差的 P 控制升降，加上指數平滑。

    Positive ``error_y`` means the target is below centre (image y grows down),
    so the drone descends (negative Tello ``rc`` ud).
    ``error_y`` 為正代表目標在畫面中心下方（影像 y 向下增加），所以下降（Tello ``rc`` ud 為負）。
    """
    return _smoothed_p_command(
        error_y,
        smoothed,
        kp=config.KP_VERTICAL,
        max_speed=config.MAX_VERTICAL_SPEED,
        min_speed=config.ALIGN_MIN_UD,
        invert=True,
    )


def forward_speed_from_ratio(bbox_ratio: float) -> int:
    """Forward RC speed from how much of the frame the box fills.
    依框佔畫面的比例決定前進 RC 速度。

    A bigger box means the target looks closer, so the drone slows down and
    finally stops. This is an image-space proxy, not a distance in metres.
    框越大代表目標看起來越近，所以減速並最終停下。這是影像空間的代理指標，不是公尺距離。
    """
    fast = min(config.FAST_APPROACH_SPEED, config.MAX_FORWARD_SPEED)
    slow = min(config.SLOW_APPROACH_SPEED, config.MAX_FORWARD_SPEED)
    if bbox_ratio >= config.STOP_THRESHOLD:
        return 0
    if bbox_ratio >= config.SLOW_DOWN_THRESHOLD:
        return slow
    if bbox_ratio <= config.APPROACH_THRESHOLD:
        return fast
    span = config.SLOW_DOWN_THRESHOLD - config.APPROACH_THRESHOLD
    t = (bbox_ratio - config.APPROACH_THRESHOLD) / span
    return int(round(fast + (slow - fast) * t))


class FlightSupervisor:
    def __init__(self, controller: DroneLike) -> None:
        self.controller = controller
        self.state = FlightState.GROUND
        self.safety_trigger: str = ""
        self._settle_started_at: Optional[float] = None
        self._scan_started_at: Optional[float] = None
        self._airborne_since: Optional[float] = None
        self._hit_streak: int = 0
        self._miss_streak: int = 0
        self._align_streak: int = 0
        self._smoothed_yaw: float = 0.0
        self._last_align_yaw: int = 0
        self._smoothed_ud: float = 0.0
        self._last_align_ud: int = 0
        self._approach_started_at: Optional[float] = None
        self._last_approach_rc: tuple[int, int, int, int] = (0, 0, 0, 0)
        self._resume_after_hold: Optional[FlightState] = None

    @property
    def in_air(self) -> bool:
        return self.state not in (FlightState.GROUND, FlightState.ABORT)

    def request_takeoff(self) -> None:
        if self.state is not FlightState.GROUND:
            LOGGER.info("Takeoff ignored in state %s", self.state.value)
            return
        if not self.controller.takeoff():
            return
        self._enter(FlightState.TAKEOFF)

    def request_land(self) -> None:
        if self.state in (FlightState.GROUND, FlightState.LANDING, FlightState.ABORT):
            return
        self.safety_trigger = "land_requested"
        self._begin_landing()

    def request_emergency(self) -> None:
        LOGGER.warning("Emergency stop")
        self.safety_trigger = "emergency"
        self.controller.emergency()
        self._enter(FlightState.ABORT)
        self._enter(FlightState.GROUND)

    def step(self, inputs: SupervisorInputs) -> None:
        self._check_abort_conditions(inputs)

        if self.state is FlightState.TAKEOFF:
            self._step_takeoff(inputs)
        elif self.state is FlightState.HOVER_SETTLE:
            self._step_hover_settle(inputs)
        elif self.state is FlightState.SCANNING:
            self._step_scanning(inputs)
        elif self.state is FlightState.ALIGNING:
            self._step_aligning(inputs)
        elif self.state is FlightState.TARGET_LOCKED:
            self._step_target_locked(inputs)
        elif self.state is FlightState.APPROACHING:
            self._step_approaching(inputs)
        elif self.state in (
            FlightState.ARRIVED,
            FlightState.APPROACH_TIMEOUT,
            FlightState.SCAN_TIMEOUT,
        ):
            self.controller.hover()
        elif self.state is FlightState.BCI_STOP:
            self._step_bci_stop(inputs)
        elif self.state is FlightState.VIDEO_LOST:
            self._step_video_lost(inputs)
        elif self.state is FlightState.LANDING:
            self._step_landing()

    def _check_abort_conditions(self, inputs: SupervisorInputs) -> None:
        if not self.in_air:
            return
        if inputs.battery is not None and inputs.battery <= config.MIN_BATTERY_ABORT:
            LOGGER.error("Battery %s%% <= abort level, landing", inputs.battery)
            self.safety_trigger = "low_battery"
            self._begin_landing()
            return
        if self._airborne_since is not None and inputs.now - self._airborne_since >= config.MAX_FLIGHT_TIME_S:
            LOGGER.error("Max flight time %.0fs reached, landing", config.MAX_FLIGHT_TIME_S)
            self.safety_trigger = "max_flight_time"
            self._begin_landing()

    def _step_takeoff(self, inputs: SupervisorInputs) -> None:
        if self.controller.busy:
            return
        if self.controller.airborne:
            self._settle_started_at = inputs.now
            self._airborne_since = inputs.now
            self._enter(FlightState.HOVER_SETTLE)
            return
        LOGGER.error("Takeoff did not leave the ground")
        self.safety_trigger = "takeoff_failed"
        self._enter(FlightState.GROUND)

    def _step_hover_settle(self, inputs: SupervisorInputs) -> None:
        if self._settle_started_at is None:
            self._settle_started_at = inputs.now
        if self._airborne_since is None:
            self._airborne_since = inputs.now
        self.controller.hover()
        if inputs.now - self._settle_started_at < config.HOVER_SETTLE_S:
            return
        if inputs.video_on and not inputs.video_ok:
            self._hold(FlightState.VIDEO_LOST, resume=FlightState.SCANNING)
            return
        self._scan_started_at = inputs.now
        self._hit_streak = 0
        self._enter(FlightState.SCANNING)

    def _step_scanning(self, inputs: SupervisorInputs) -> None:
        if inputs.bci_stopped:
            self._hold(FlightState.BCI_STOP, resume=FlightState.SCANNING)
            return
        if inputs.video_on and not inputs.video_ok:
            self._hold(FlightState.VIDEO_LOST, resume=FlightState.SCANNING)
            return

        if inputs.detection_tick:
            if inputs.target_found:
                self._hit_streak += 1
            else:
                self._hit_streak = 0

        if self._hit_streak >= config.MIN_DETECTION_FRAMES:
            LOGGER.info("Target seen after %d consecutive hits, aligning", self._hit_streak)
            self._begin_aligning()
            self._step_aligning(inputs)
            return

        if self._scan_started_at is None:
            self._scan_started_at = inputs.now
        if inputs.now - self._scan_started_at >= config.MAX_SCAN_TIME_S:
            LOGGER.warning("Scan timed out after %.1fs", config.MAX_SCAN_TIME_S)
            self.safety_trigger = "scan_timeout"
            self.controller.hover()
            self._enter(FlightState.SCAN_TIMEOUT)
            return

        yaw = min(config.SCAN_YAW_SPEED, config.MAX_YAW_SPEED)
        self.controller.send_rc(0, 0, 0, yaw)

    def _begin_aligning(self) -> None:
        self._miss_streak = 0
        self._align_streak = 0
        self._smoothed_yaw = 0.0
        self._last_align_yaw = 0
        self._smoothed_ud = 0.0
        self._last_align_ud = 0
        self._enter(FlightState.ALIGNING)

    def _step_aligning(self, inputs: SupervisorInputs) -> None:
        if inputs.bci_stopped:
            self._hold(FlightState.BCI_STOP, resume=FlightState.ALIGNING)
            return
        if inputs.video_on and not inputs.video_ok:
            self._hold(FlightState.VIDEO_LOST, resume=FlightState.ALIGNING)
            return

        if inputs.detection_tick:
            if not inputs.target_found or inputs.error_x is None:
                self._miss_streak += 1
                self._align_streak = 0
                if self._miss_streak >= config.TARGET_LOST_FRAMES:
                    LOGGER.info("Target lost while aligning, resume scan")
                    self._return_to_scan(inputs.now)
                    self._step_scanning(inputs)
                    return
            else:
                self._miss_streak = 0
                yaw, self._smoothed_yaw = yaw_command_from_error(inputs.error_x, self._smoothed_yaw)
                self._last_align_yaw = yaw
                if abs(inputs.error_x) <= config.ALIGNMENT_TOLERANCE_PX:
                    self._align_streak += 1
                else:
                    self._align_streak = 0
                if self._align_streak >= config.MIN_DETECTION_FRAMES:
                    LOGGER.info("Yaw centred, adjusting height")
                    self._begin_locked()
                    self._step_target_locked(inputs)
                    return

        # Keep the last yaw between detection ticks. Never command forward or climb.
        # 兩次偵測之間維持上一次偏航。絕不送前進或升降。
        self.controller.send_rc(0, 0, 0, self._last_align_yaw)

    def _begin_locked(self) -> None:
        self._miss_streak = 0
        self._align_streak = 0
        self._smoothed_ud = 0.0
        self._last_align_ud = 0
        self._enter(FlightState.TARGET_LOCKED)

    def _step_target_locked(self, inputs: SupervisorInputs) -> None:
        if inputs.bci_stopped:
            self._hold(FlightState.BCI_STOP, resume=FlightState.TARGET_LOCKED)
            return
        if inputs.video_on and not inputs.video_ok:
            self._hold(FlightState.VIDEO_LOST, resume=FlightState.TARGET_LOCKED)
            return

        if inputs.detection_tick:
            if not inputs.target_found or inputs.error_x is None:
                self._miss_streak += 1
                if self._miss_streak >= config.TARGET_LOST_FRAMES:
                    LOGGER.info("Target lost after lock, resume scan")
                    self._return_to_scan(inputs.now)
                    self._step_scanning(inputs)
                    return
            else:
                self._miss_streak = 0
                if abs(inputs.error_x) > config.ALIGN_REACQUIRE_PX:
                    LOGGER.info("Target drifted (err x=%.0f), realigning yaw", inputs.error_x)
                    self._begin_aligning()
                    self._step_aligning(inputs)
                    return
                if inputs.error_y is not None:
                    ud, self._smoothed_ud = ud_command_from_error(
                        inputs.error_y, self._smoothed_ud
                    )
                    self._last_align_ud = ud
                    if abs(inputs.error_y) <= config.ALIGNMENT_TOLERANCE_PX:
                        self._align_streak += 1
                    else:
                        self._align_streak = 0
                    if self._align_streak >= config.MIN_DETECTION_FRAMES:
                        LOGGER.info("Height centred, approaching")
                        self._begin_approach(inputs.now)
                        self._step_approaching(inputs)
                        return

        # Height only after yaw lock. Forward waits until height is centred too.
        # 偏航鎖定後才調高度。高度也置中之後才前進。
        self.controller.send_rc(0, 0, self._last_align_ud, 0)

    def _begin_approach(self, now: float) -> None:
        self._miss_streak = 0
        self._smoothed_yaw = 0.0
        self._smoothed_ud = 0.0
        self._approach_started_at = now
        self._last_approach_rc = (0, 0, 0, 0)
        self._enter(FlightState.APPROACHING)

    def _step_approaching(self, inputs: SupervisorInputs) -> None:
        # Holds resume at TARGET_LOCKED, never mid-flight: re-centre before moving again.
        # 暫停後一律回到 TARGET_LOCKED，不直接續飛：先重新置中再前進。
        if inputs.bci_stopped:
            self._hold(FlightState.BCI_STOP, resume=FlightState.TARGET_LOCKED)
            return
        if inputs.video_on and not inputs.video_ok:
            self._hold(FlightState.VIDEO_LOST, resume=FlightState.TARGET_LOCKED)
            return

        if self._approach_started_at is None:
            self._approach_started_at = inputs.now
        if inputs.now - self._approach_started_at >= config.MAX_APPROACH_TIME_S:
            LOGGER.warning("Approach timed out after %.1fs", config.MAX_APPROACH_TIME_S)
            self.safety_trigger = "approach_timeout"
            self.controller.hover()
            self._enter(FlightState.APPROACH_TIMEOUT)
            return

        if inputs.detection_tick:
            if not inputs.target_found or inputs.error_x is None or inputs.bbox_ratio is None:
                # Never keep flying forward on a stale box.
                # 絕不拿過期的框繼續往前飛。
                self._last_approach_rc = (0, 0, 0, 0)
                self._miss_streak += 1
                if self._miss_streak >= config.TARGET_LOST_FRAMES:
                    LOGGER.info("Target lost while approaching, resume scan")
                    self._return_to_scan(inputs.now)
                    self._step_scanning(inputs)
                    return
            else:
                self._miss_streak = 0
                if inputs.bbox_ratio >= config.STOP_THRESHOLD:
                    LOGGER.info("Arrived: bbox ratio %.3f >= %.3f",
                                inputs.bbox_ratio, config.STOP_THRESHOLD)
                    self.controller.hover()
                    self._enter(FlightState.ARRIVED)
                    return
                if abs(inputs.error_x) > config.ALIGN_REACQUIRE_PX:
                    LOGGER.info("Target drifted (err x=%.0f) while approaching, realigning",
                                inputs.error_x)
                    self._begin_aligning()
                    self._step_aligning(inputs)
                    return
                yaw, self._smoothed_yaw = yaw_command_from_error(inputs.error_x, self._smoothed_yaw)
                ud = 0
                if inputs.error_y is not None:
                    ud, self._smoothed_ud = ud_command_from_error(inputs.error_y, self._smoothed_ud)
                fb = forward_speed_from_ratio(inputs.bbox_ratio)
                self._last_approach_rc = (0, fb, ud, yaw)

        self.controller.send_rc(*self._last_approach_rc)

    def _return_to_scan(self, now: float) -> None:
        self._hit_streak = 0
        self._miss_streak = 0
        self._align_streak = 0
        self._smoothed_yaw = 0.0
        self._last_align_yaw = 0
        self._smoothed_ud = 0.0
        self._last_align_ud = 0
        self._approach_started_at = None
        self._last_approach_rc = (0, 0, 0, 0)
        self._scan_started_at = now
        self.controller.hover()
        self._enter(FlightState.SCANNING)

    def _step_bci_stop(self, inputs: SupervisorInputs) -> None:
        self.controller.hover()
        if not inputs.bci_stopped:
            self._resume_from_hold()

    def _step_video_lost(self, inputs: SupervisorInputs) -> None:
        self.controller.hover()
        if not inputs.video_on or inputs.video_ok:
            self._resume_from_hold()

    def _step_landing(self) -> None:
        if self.controller.busy:
            return
        if not self.controller.airborne:
            self._enter(FlightState.GROUND)
            return
        # land() was refused or failed while still airborne: try once more.
        # land() 被拒或失敗但還在空中：再試一次。
        self.controller.land()

    def _begin_landing(self) -> None:
        if self.state is FlightState.LANDING:
            return
        self.controller.hover()
        self.controller.land()
        self._enter(FlightState.LANDING)

    def _hold(self, hold_state: FlightState, resume: FlightState) -> None:
        self.controller.hover()
        self._resume_after_hold = resume
        self._enter(hold_state)

    def _resume_from_hold(self) -> None:
        nxt = self._resume_after_hold or FlightState.SCANNING
        self._resume_after_hold = None
        if nxt is FlightState.SCANNING:
            self._hit_streak = 0
        if nxt is FlightState.TARGET_LOCKED:
            self._begin_locked()
            return
        self._enter(nxt)

    def _enter(self, new_state: FlightState) -> None:
        if new_state is self.state:
            return
        LOGGER.info("STATE %s -> %s", self.state.value, new_state.value)
        self.state = new_state
        if new_state is FlightState.GROUND:
            self._settle_started_at = None
            self._scan_started_at = None
            self._airborne_since = None
            self._hit_streak = 0
            self._miss_streak = 0
            self._align_streak = 0
            self._smoothed_yaw = 0.0
            self._last_align_yaw = 0
            self._smoothed_ud = 0.0
            self._last_align_ud = 0
            self._approach_started_at = None
            self._last_approach_rc = (0, 0, 0, 0)
            self._resume_after_hold = None
