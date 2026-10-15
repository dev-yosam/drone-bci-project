"""Flight-phase state machine. No UDP here - it only asks the controller to act.
飛行階段狀態機。這裡不碰 UDP，只向控制器提出動作請求。

Milestone 4 aligns yaw first (``error_x``), then height (``error_y``) after
lock. Forward approach is still later; aligned means hover, not fly closer.
里程碑 4 先用 ``error_x`` 對齊偏航，鎖定後再用 ``error_y`` 對齊高度。
前進接近仍留給後面；對齊完成只是懸停，不是飛近。
"""
