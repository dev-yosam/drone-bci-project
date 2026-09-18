"""Explicit flight state machine for takeoff, hover, and yaw scan.
起飛、懸停、偏航掃描的明確飛行狀態機。

The supervisor never talks to UDP. It asks a ``TelloController`` to take off,
to send RC, or to land. That keeps ``DRY_RUN`` in one place and makes the
transitions unit-testable with a fake controller.
監督器從不碰 UDP。它只請 ``TelloController`` 起飛、送 RC、或降落。
這樣 ``DRY_RUN`` 仍集中在一處，狀態轉移也可以用假控制器做單元測試。
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
    TARGET_LOCKED = "TARGET_LOCKED"
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


class FlightSupervisor:
    def __init__(self, controller: DroneLike) -> None:
        self.controller = controller
        self.state = FlightState.GROUND
        self.safety_trigger: str = ""
        self._settle_started_at: Optional[float] = None
        self._scan_started_at: Optional[float] = None
        self._airborne_since: Optional[float] = None
        self._hit_streak: int = 0
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
        elif self.state is FlightState.TARGET_LOCKED:
            self.controller.hover()
        elif self.state is FlightState.SCAN_TIMEOUT:
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
            LOGGER.info("Target locked after %d consecutive hits", self._hit_streak)
            self.controller.hover()
            self._enter(FlightState.TARGET_LOCKED)
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
            self._resume_after_hold = None
