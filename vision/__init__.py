"""Perception layer: object detection, colour validation, target selection.
感知層：物件偵測、顏色驗證、目標選擇。

Nothing in this package knows that a drone exists. It turns frames into
``Detection`` objects so the modules can be unit tested without hardware.
這個套件完全不知道無人機的存在。它只把影格轉成 ``Detection`` 物件，因此可以在沒有硬體的情況下做單元測試。
"""
