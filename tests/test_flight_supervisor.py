"""Flight supervisor tests. No drone and no network.
飛行監督器測試。不需要無人機、也不需要網路。
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import config  # noqa: E402
from navigation.flight_supervisor import (  # noqa: E402
    FlightState,
    FlightSupervisor,
    SupervisorInputs,
    ud_command_from_error,
    yaw_command_from_error,
)


class FakeDrone:
    def __init__(self) -> None:
        self.dry_run = True
        self.busy = False
        self.airborne = False
        self.last_rc = (0, 0, 0, 0)
        self.calls: list[str] = []
        self.refuse_takeoff = False

    def takeoff(self) -> bool:
        if self.refuse_takeoff:
            self.calls.append("takeoff-refused")
            return False
        self.airborne = True
        self.calls.append("takeoff")
        return True

    def land(self) -> bool:
        if not self.airborne:
            self.calls.append("land-ignored")
            return False
        self.airborne = False
        self.calls.append("land")
        return True

    def hover(self) -> None:
        self.send_rc(0, 0, 0, 0)
        self.calls.append("hover")

    def send_rc(self, left_right: int, forward_back: int, up_down: int, yaw: int) -> None:
        self.last_rc = (left_right, forward_back, up_down, yaw)
        self.calls.append(f"rc:{left_right},{forward_back},{up_down},{yaw}")

    def emergency(self) -> None:
        self.airborne = False
        self.calls.append("emergency")


def inputs(
    now: float,
    *,
    target_found: bool = False,
    detection_tick: bool = False,
    bci_stopped: bool = False,
    video_ok: bool = True,
    video_on: bool = True,
    battery: int = 80,
    error_x: float | None = None,
    error_y: float | None = None,
) -> SupervisorInputs:
    return SupervisorInputs(
        now=now,
        target_found=target_found,
        detection_tick=detection_tick,
        bci_stopped=bci_stopped,
        video_ok=video_ok,
        video_on=video_on,
        battery=battery,
        error_x=error_x,
        error_y=error_y,
    )


class FlightSupervisorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.drone = FakeDrone()
        self.sup = FlightSupervisor(self.drone)

    def _airborne_and_scanning(self, now: float = 0.0) -> float:
        self.sup.request_takeoff()
        self.sup.step(inputs(now))
        self.assertEqual(self.sup.state, FlightState.HOVER_SETTLE)
        later = now + config.HOVER_SETTLE_S
        self.sup.step(inputs(later))
        self.assertEqual(self.sup.state, FlightState.SCANNING)
        return later

    def test_refused_takeoff_stays_on_ground(self):
        self.drone.refuse_takeoff = True
        self.sup.request_takeoff()
        self.assertEqual(self.sup.state, FlightState.GROUND)

    def test_takeoff_ignored_when_not_on_ground(self):
        self._airborne_and_scanning()
        self.drone.calls.clear()
        self.sup.request_takeoff()
        self.assertNotIn("takeoff", self.drone.calls)
        self.assertEqual(self.sup.state, FlightState.SCANNING)

    def test_scan_sends_positive_yaw(self):
        t = self._airborne_and_scanning()
        self.sup.step(inputs(t + 0.1))
        self.assertEqual(self.drone.last_rc[3], min(config.SCAN_YAW_SPEED, config.MAX_YAW_SPEED))
        self.assertGreater(self.drone.last_rc[3], 0)

    def test_lock_requires_consecutive_detection_ticks(self):
        t = self._airborne_and_scanning()
        for i in range(config.MIN_DETECTION_FRAMES - 1):
            self.sup.step(inputs(t + i * 0.1, target_found=True, detection_tick=True))
            self.assertEqual(self.sup.state, FlightState.SCANNING)

        # Loop ticks without a new inference must not finish the streak.
        # 沒有新推論的迴圈次數不該把連擊數湊滿。
        self.sup.step(inputs(t + 1.0, target_found=True, detection_tick=False))
        self.assertEqual(self.sup.state, FlightState.SCANNING)

        self.sup.step(
            inputs(t + 1.1, target_found=True, detection_tick=True, error_x=200.0)
        )
        self.assertEqual(self.sup.state, FlightState.ALIGNING)
        self.assertGreater(self.drone.last_rc[3], 0)
        self.assertEqual(self.drone.last_rc[1], 0)

    def test_missed_detection_resets_streak(self):
        t = self._airborne_and_scanning()
        self.sup.step(inputs(t + 0.1, target_found=True, detection_tick=True))
        self.sup.step(inputs(t + 0.2, target_found=False, detection_tick=True))
        for i in range(config.MIN_DETECTION_FRAMES - 1):
            self.sup.step(inputs(t + 0.3 + i * 0.1, target_found=True, detection_tick=True))
        self.assertEqual(self.sup.state, FlightState.SCANNING)

    def test_scan_timeout_hovers(self):
        t = self._airborne_and_scanning()
        self.sup.step(inputs(t + config.MAX_SCAN_TIME_S + 0.1))
        self.assertEqual(self.sup.state, FlightState.SCAN_TIMEOUT)
        self.assertEqual(self.sup.safety_trigger, "scan_timeout")
        self.assertEqual(self.drone.last_rc, (0, 0, 0, 0))

    def test_bci_stop_pauses_yaw_then_resumes(self):
        t = self._airborne_and_scanning()
        self.sup.step(inputs(t + 0.1, bci_stopped=True))
        self.assertEqual(self.sup.state, FlightState.BCI_STOP)
        self.assertEqual(self.drone.last_rc, (0, 0, 0, 0))
        self.sup.step(inputs(t + 0.2, bci_stopped=False))
        self.assertEqual(self.sup.state, FlightState.SCANNING)

    def test_video_loss_pauses_yaw_then_resumes(self):
        t = self._airborne_and_scanning()
        self.sup.step(inputs(t + 0.1, video_ok=False))
        self.assertEqual(self.sup.state, FlightState.VIDEO_LOST)
        self.assertEqual(self.drone.last_rc, (0, 0, 0, 0))
        self.sup.step(inputs(t + 0.2, video_ok=True))
        self.assertEqual(self.sup.state, FlightState.SCANNING)

    def test_low_battery_lands(self):
        t = self._airborne_and_scanning()
        self.sup.step(inputs(t + 0.1, battery=config.MIN_BATTERY_ABORT))
        self.sup.step(inputs(t + 0.2, battery=config.MIN_BATTERY_ABORT))
        self.assertEqual(self.sup.state, FlightState.GROUND)
        self.assertFalse(self.drone.airborne)
        self.assertEqual(self.sup.safety_trigger, "low_battery")

    def test_land_request_from_scan(self):
        self._airborne_and_scanning()
        self.sup.request_land()
        self.assertEqual(self.sup.state, FlightState.LANDING)
        self.sup.step(inputs(10.0))
        self.assertEqual(self.sup.state, FlightState.GROUND)

    def test_emergency_drops_to_ground(self):
        self._airborne_and_scanning()
        self.sup.request_emergency()
        self.assertEqual(self.sup.state, FlightState.GROUND)
        self.assertIn("emergency", self.drone.calls)

    def _into_aligning(self, error_x: float = 200.0) -> float:
        t = self._airborne_and_scanning()
        for i in range(config.MIN_DETECTION_FRAMES):
            self.sup.step(
                inputs(
                    t + (i + 1) * 0.1,
                    target_found=True,
                    detection_tick=True,
                    error_x=error_x,
                )
            )
        self.assertEqual(self.sup.state, FlightState.ALIGNING)
        return t + config.MIN_DETECTION_FRAMES * 0.1

    def test_aligning_yaws_clockwise_when_target_is_right(self):
        self._into_aligning(error_x=200.0)
        self.assertGreater(self.drone.last_rc[3], 0)
        self.assertLessEqual(abs(self.drone.last_rc[3]), config.MAX_YAW_SPEED)
        self.assertEqual(self.drone.last_rc[:3], (0, 0, 0))

    def test_aligning_yaws_counterclockwise_when_target_is_left(self):
        self._into_aligning(error_x=-200.0)
        self.assertLess(self.drone.last_rc[3], 0)
        self.assertEqual(self.drone.last_rc[:3], (0, 0, 0))

    def test_centred_target_locks_after_consecutive_deadband_ticks(self):
        t = self._into_aligning(error_x=200.0)
        for i in range(config.MIN_DETECTION_FRAMES):
            self.sup.step(
                inputs(
                    t + (i + 1) * 0.1,
                    target_found=True,
                    detection_tick=True,
                    error_x=0.0,
                    error_y=0.0,
                )
            )
        self.assertEqual(self.sup.state, FlightState.TARGET_LOCKED)
        self.assertEqual(self.drone.last_rc, (0, 0, 0, 0))

    def _into_locked(self, error_y: float = 0.0) -> float:
        t = self._into_aligning(error_x=200.0)
        for i in range(config.MIN_DETECTION_FRAMES):
            self.sup.step(
                inputs(
                    t + (i + 1) * 0.1,
                    target_found=True,
                    detection_tick=True,
                    error_x=0.0,
                    error_y=error_y,
                )
            )
        self.assertEqual(self.sup.state, FlightState.TARGET_LOCKED)
        return t + config.MIN_DETECTION_FRAMES * 0.1

    def test_locked_target_below_centre_descends(self):
        self._into_locked(error_y=200.0)
        lr, fb, ud, yaw = self.drone.last_rc
        self.assertEqual((lr, fb, yaw), (0, 0, 0))
        self.assertLess(ud, 0)
        self.assertLessEqual(abs(ud), config.MAX_VERTICAL_SPEED)

    def test_locked_target_above_centre_climbs(self):
        self._into_locked(error_y=-200.0)
        lr, fb, ud, yaw = self.drone.last_rc
        self.assertEqual((lr, fb, yaw), (0, 0, 0))
        self.assertGreater(ud, 0)

    def test_aligning_does_not_change_height(self):
        self._into_aligning(error_x=200.0)
        self.sup.step(
            inputs(
                10.0,
                target_found=True,
                detection_tick=True,
                error_x=200.0,
                error_y=200.0,
            )
        )
        self.assertEqual(self.sup.state, FlightState.ALIGNING)
        self.assertEqual(self.drone.last_rc[2], 0)
        self.assertGreater(self.drone.last_rc[3], 0)

    def test_lost_target_while_aligning_returns_to_scan(self):
        t = self._into_aligning(error_x=200.0)
        for i in range(config.TARGET_LOST_FRAMES):
            self.sup.step(
                inputs(t + (i + 1) * 0.1, target_found=False, detection_tick=True)
            )
        self.assertEqual(self.sup.state, FlightState.SCANNING)
        self.assertGreater(self.drone.last_rc[3], 0)

    def test_drift_after_lock_resumes_aligning(self):
        t = self._into_aligning(error_x=10.0)
        for i in range(config.MIN_DETECTION_FRAMES):
            self.sup.step(
                inputs(
                    t + (i + 1) * 0.1,
                    target_found=True,
                    detection_tick=True,
                    error_x=0.0,
                )
            )
        self.assertEqual(self.sup.state, FlightState.TARGET_LOCKED)
        later = t + 1.0
        self.sup.step(
            inputs(
                later,
                target_found=True,
                detection_tick=True,
                error_x=float(config.ALIGN_REACQUIRE_PX + 40),
            )
        )
        self.assertEqual(self.sup.state, FlightState.ALIGNING)
        self.assertGreater(self.drone.last_rc[3], 0)

    def test_bci_stop_while_aligning_resumes_aligning(self):
        t = self._into_aligning(error_x=200.0)
        self.sup.step(inputs(t + 0.1, bci_stopped=True, target_found=True, error_x=200.0))
        self.assertEqual(self.sup.state, FlightState.BCI_STOP)
        self.assertEqual(self.drone.last_rc, (0, 0, 0, 0))
        self.sup.step(inputs(t + 0.2, bci_stopped=False, target_found=True, error_x=200.0))
        self.assertEqual(self.sup.state, FlightState.ALIGNING)

    def test_takeoff_failure_returns_to_ground(self):
        self.drone.takeoff()
        self.drone.airborne = False
        self.drone.calls.clear()
        # Reconstruct: request already called takeoff on a drone that then reports not airborne.
        # 重建：已呼叫起飛，但之後回報不在空中。
        self.sup.state = FlightState.TAKEOFF
        self.sup.step(inputs(0.0))
        self.assertEqual(self.sup.state, FlightState.GROUND)
        self.assertEqual(self.sup.safety_trigger, "takeoff_failed")


class YawCommandTests(unittest.TestCase):
    def test_deadband_commands_zero(self):
        cmd, _ = yaw_command_from_error(0.0, 0.0)
        self.assertEqual(cmd, 0)
        cmd, _ = yaw_command_from_error(float(config.ALIGNMENT_TOLERANCE_PX), 0.0)
        self.assertEqual(cmd, 0)

    def test_positive_error_is_clockwise(self):
        cmd, smoothed = yaw_command_from_error(200.0, 0.0)
        self.assertGreater(cmd, 0)
        self.assertLessEqual(cmd, config.MAX_YAW_SPEED)
        self.assertGreater(smoothed, 0)

    def test_negative_error_is_counterclockwise(self):
        cmd, _ = yaw_command_from_error(-200.0, 0.0)
        self.assertLess(cmd, 0)
        self.assertGreaterEqual(cmd, -config.MAX_YAW_SPEED)

    def test_clamped_to_max_yaw(self):
        cmd, _ = yaw_command_from_error(10_000.0, 0.0)
        self.assertLessEqual(abs(cmd), config.MAX_YAW_SPEED)

    def test_smoothing_lags_the_raw_command(self):
        first, smoothed = yaw_command_from_error(200.0, 0.0)
        second, _ = yaw_command_from_error(200.0, smoothed)
        raw = min(config.MAX_YAW_SPEED, config.KP_YAW * 200.0)
        self.assertLess(abs(first), abs(raw) + 1e-6)
        self.assertGreaterEqual(abs(second), abs(first))


class UdCommandTests(unittest.TestCase):
    def test_deadband_commands_zero(self):
        cmd, _ = ud_command_from_error(0.0, 0.0)
        self.assertEqual(cmd, 0)
        cmd, _ = ud_command_from_error(float(config.ALIGNMENT_TOLERANCE_PX), 0.0)
        self.assertEqual(cmd, 0)

    def test_positive_error_descends(self):
        cmd, _ = ud_command_from_error(200.0, 0.0)
        self.assertLess(cmd, 0)
        self.assertGreaterEqual(cmd, -config.MAX_VERTICAL_SPEED)

    def test_negative_error_climbs(self):
        cmd, _ = ud_command_from_error(-200.0, 0.0)
        self.assertGreater(cmd, 0)
        self.assertLessEqual(cmd, config.MAX_VERTICAL_SPEED)

    def test_clamped_to_max_vertical(self):
        cmd, _ = ud_command_from_error(10_000.0, 0.0)
        self.assertLessEqual(abs(cmd), config.MAX_VERTICAL_SPEED)


if __name__ == "__main__":
    unittest.main(verbosity=2)
