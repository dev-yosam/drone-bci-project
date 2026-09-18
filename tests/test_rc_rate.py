"""RC send-rate helper. No drone required.
RC 送出頻率的輔助測試。不需要無人機。
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drone.tello_controller import rc_send_due  # noqa: E402


HOVER = (0, 0, 0, 0)
YAW = (0, 0, 0, 20)
INTERVAL = 0.1  # 10 Hz


class RcSendDueTests(unittest.TestCase):
    def test_first_command_is_always_due(self):
        self.assertTrue(rc_send_due(None, 0.0, HOVER, 0.0, INTERVAL))

    def test_repeat_inside_interval_is_skipped(self):
        self.assertFalse(rc_send_due(HOVER, 1.00, HOVER, 1.05, INTERVAL))

    def test_repeat_after_interval_is_keepalive(self):
        self.assertTrue(rc_send_due(HOVER, 1.00, HOVER, 1.10, INTERVAL))

    def test_value_change_is_sent_immediately(self):
        # Stopping yaw must not wait for the next 10 Hz tick.
        # 停止偏航不能再等下一個 10 Hz 週期。
        self.assertTrue(rc_send_due(YAW, 1.00, HOVER, 1.01, INTERVAL))

    def test_30hz_hover_would_send_about_every_third_frame(self):
        last_cmd = None
        last_t = 0.0
        sent = 0
        for i in range(30):
            now = i / 30.0
            if rc_send_due(last_cmd, last_t, HOVER, now, INTERVAL):
                sent += 1
                last_cmd = HOVER
                last_t = now
        # 1 second at 30 fps, 10 Hz cap -> about 10 packets, never 30.
        # 1 秒 30 幀、上限 10 Hz → 大約 10 包，絕不會 30 包。
        self.assertGreaterEqual(sent, 9)
        self.assertLessEqual(sent, 11)


if __name__ == "__main__":
    unittest.main(verbosity=2)
