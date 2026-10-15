"""Flight-phase state machine. No UDP here - it only asks the controller to act.
飛行階段狀態機。這裡不碰 UDP，只向控制器提出動作請求。

Milestone 4 aligns yaw first (``error_x``), then height (``error_y``) after
lock. Milestone 5 then flies forward until the box ratio says "close enough".
里程碑 4 先用 ``error_x`` 對齊偏航，鎖定後再用 ``error_y`` 對齊高度。
里程碑 5 接著往前飛，直到框比例顯示「夠近了」。
"""
