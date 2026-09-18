"""Flight supervisor tests. No drone and no network.
飛行監督器測試。不需要無人機、也不需要網路。
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import config  # noqa: E402
from navigation.flight_supervisor import FlightState, FlightSupervisor, SupervisorInputs  # noqa: E402


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
) -> SupervisorInputs:
    return SupervisorInputs(
        now=now,
        target_found=target_found,
        detection_tick=detection_tick,
        bci_stopped=bci_stopped,
        video_ok=video_ok,
        video_on=video_on,
        battery=battery,
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

        self.sup.step(inputs(t + 1.1, target_found=True, detection_tick=True))
        self.assertEqual(self.sup.state, FlightState.TARGET_LOCKED)
        self.assertEqual(self.drone.last_rc, (0, 0, 0, 0))

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


if __name__ == "__main__":
    unittest.main(verbosity=2)
