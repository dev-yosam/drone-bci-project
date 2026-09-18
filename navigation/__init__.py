"""Flight-phase state machine. No UDP here - it only asks the controller to act.
飛行階段狀態機。這裡不碰 UDP，只向控制器提出動作請求。

Milestone 3 covers takeoff, a short hover to let the airframe settle, then a
slow yaw scan until the target is seen (or the scan budget runs out). Alignment
and forward approach stay later milestones; locking the target here means
"stop turning and hover", not "fly towards it".
里程碑 3 涵蓋起飛、短暫懸停讓機身穩定，然後慢速偏航掃描直到看到目標（或掃描時間用盡）。
對齊與前進接近留給後面的里程碑；這裡鎖住目標的意思是「停止旋轉並懸停」，不是「飛過去」。
"""
