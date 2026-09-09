"""Central configuration for the Tello autonomous navigation prototype.
Tello 自主導航原型的集中設定檔。

This module is intentionally pure data: no imports from the rest of the
project, no logic. Every threshold, speed limit and feature flag lives here so
that experiments can be reproduced by recording a single file.
這個模組刻意只放資料：不匯入專案其他模組、不含邏輯。所有門檻、速度上限與功能開關都寫在這裡，之後只要保存這一份檔案就能重現實驗。

Parameters are grouped by the milestone that first uses them. Anything marked
"not used yet" is declared early so the schema stays stable across milestones,
but it has no effect until that milestone is implemented.
參數依「第一次會用到的里程碑」分組。標成「尚未使用」的項目提前宣告，是為了讓各里程碑的欄位結構保持穩定；在該里程碑實作之前，這些值不會生效。
"""

from pathlib import Path

# ---------------------------------------------------------------------------
# Run mode  # 執行模式
# ---------------------------------------------------------------------------

# When True, no command that can move the drone is ever sent. Video, detection,
# the state machine and logging all still run. This is the main safety net
# during development - keep it True unless you are deliberately flying.
# 為 True 時，任何會讓無人機移動的指令都不會送出。影像、偵測、狀態機與紀錄仍會跑。
# 這是開發期間最主要的安全網——除非你刻意要飛，否則維持 True。
DRY_RUN: bool = True

PROJECT_ROOT: Path = Path(__file__).resolve().parent
LOG_DIR: Path = PROJECT_ROOT / "logs"
MODEL_DIR: Path = PROJECT_ROOT / "models"

LOG_LEVEL: str = "INFO"
# djitellopy logs every single UDP command at INFO level, which drowns out our
# own output. Raise its level separately.
# djitellopy 會把每一條 UDP 指令都打成 INFO，會把我們自己的輸出淹沒。所以把它的等級單獨調高。
TELLO_LOG_LEVEL: str = "WARNING"
CSV_LOG_ENABLED: bool = True

# ---------------------------------------------------------------------------
# Connection (Milestone 1)  # 連線（里程碑 1）
# ---------------------------------------------------------------------------

# Station mode: the drone is joined to the TP-Link router and has a reserved
# DHCP address. For direct AP mode (laptop joined to the TELLO-XXXXXX hotspot)
# use "192.168.10.1" instead.
# Station 模式：無人機已加入 TP-Link 路由器，並有保留的 DHCP 位址。
# 若改用直連 AP 模式（筆電連 TELLO-XXXXXX 熱點），改填 "192.168.10.1"。
TELLO_HOST: str = "192.168.0.220"

# How many times djitellopy retries a control command before raising.
# djitellopy 在拋出例外前，會重試控制指令幾次。
CONNECT_RETRY_COUNT: int = 3

# ---------------------------------------------------------------------------
# Video (Milestone 1)  # 影像（里程碑 1）
# ---------------------------------------------------------------------------

VIDEO_ENABLED: bool = True

# setresolution / setfps / setbitrate are Tello EDU SDK 3.0 commands. Older
# firmware answers "unknown command" and djitellopy then raises. They are off
# by default so a firmware mismatch cannot abort startup; the controller warns
# instead of failing when they are enabled and rejected.
# setresolution / setfps / setbitrate 是 Tello EDU SDK 3.0 指令。較舊韌體會回
# "unknown command"，接著 djitellopy 就會拋錯。預設關閉，避免韌體不符就啟動失敗；
# 若開啟後被拒，控制器只會警告、不會中止。
APPLY_VIDEO_SETTINGS: bool = False
VIDEO_RESOLUTION: str = "720p"  # "480p" or "720p"
VIDEO_FPS: str = "30"  # "5", "15" or "30"
VIDEO_BITRATE_MBPS: int = 0  # 0 = auto, otherwise 1-5  # 0 = 自動，否則 1-5

WINDOW_NAME: str = "Tello BCI - Milestone 2"
# Tello streams 960x720. Scale the preview window without touching the frame
# that later gets fed to YOLO.
# Tello 串流解析度是 960x720。這裡只縮放預覽視窗，不會動到之後送進 YOLO 的影格。
DISPLAY_SCALE: float = 1.0

# No decoded frame for this long counts as video loss.
# 超過這段時間都沒有解出新影格，就視為影像中斷。
VIDEO_LOSS_TIMEOUT_S: float = 2.0
# How long to wait after streamon() for the first frame before giving up.
# 呼叫 streamon() 之後，等多久還沒等到第一張影格就放棄。
FIRST_FRAME_TIMEOUT_S: float = 15.0

# ---------------------------------------------------------------------------
# Safety (Milestone 6, partially enforced from Milestone 1)
# 安全（里程碑 6；部分從里程碑 1 就開始執行）
# ---------------------------------------------------------------------------

MIN_BATTERY_TAKEOFF: int = 40  # refuse to take off below this percentage  # 低於此電量拒絕起飛
MIN_BATTERY_ABORT: int = 20  # land immediately below this percentage  # 低於此電量立刻降落
MAX_FLIGHT_TIME_S: float = 120.0  # hard cap on one autonomous run  # 單次自主飛行的硬上限
COMMAND_WATCHDOG_S: float = 1.0  # no fresh command within this -> hover  # 超過這時間沒有新指令就懸停
TARGET_LOST_FRAMES: int = 10  # consecutive misses before abandoning approach  # 連續幾幀找不到目標才放棄接近

# Velocity clamps in the -100..100 units that Tello's `rc` command expects.
# Deliberately conservative for indoor testing.
# 速度限制，單位是 Tello `rc` 指令用的 -100..100。室內測試刻意保守。
MAX_FORWARD_SPEED: int = 20
MAX_YAW_SPEED: int = 30
MAX_VERTICAL_SPEED: int = 20

# ---------------------------------------------------------------------------
# YOLO detection (Milestone 2)  # YOLO 偵測（里程碑 2）
# ---------------------------------------------------------------------------

# Resolved against MODEL_DIR when it is a bare filename. Keeping the weights
# inside the project matters because the Tello router has no internet:
# ultralytics cannot download them mid-session.
# 若只寫檔名，會對 MODEL_DIR 解析。權重必須放在專案裡，因為 Tello 路由器沒有對外網路：
# 執行中 ultralytics 無法臨時下載。
MODEL_PATH: str = "yolo11n.pt"
YOLO_IMG_SIZE: int = 640
YOLO_DEVICE: str = "cpu"  # "cpu", "cuda:0", ...
TARGET_OBJECT: str = "cup"
TARGET_CONFIDENCE: float = 0.5
# Run inference on every Nth new frame; 1 = every frame. yolo11n at 640px
# measures ~45 ms per frame on this laptop's CPU (~22 fps) against a 30 fps
# video stream, so every frame would make the display lag behind reality.
# Every second frame gives ~15 Hz detection with headroom to spare.
# 每 N 張「新影格」才推論一次；1 = 每張都跑。這台筆電 CPU 上 yolo11n、640px
# 約 45 ms/張（約 22 fps），影像卻是 30 fps，每張都跑會讓畫面落後現實。
# 隔一張跑一次大約 15 Hz，還留有餘裕。
DETECT_EVERY_N_FRAMES: int = 2

# ---------------------------------------------------------------------------
# Optional colour filtering (Milestone 2)  # 可選的顏色濾鏡（里程碑 2）
# ---------------------------------------------------------------------------

COLOUR_FILTER_ENABLED: bool = False
TARGET_COLOUR: str = "red"
# Red wraps around the hue circle in OpenCV's 0-179 H range, so it needs two
# intervals. Each entry is (lower_hsv, upper_hsv).
# 紅色在 OpenCV 0-179 的色相圓上會繞過 0 度，所以需要兩段區間。
# 每一筆是 (lower_hsv, upper_hsv)。
RED_HSV_RANGES: tuple = (
    ((0, 120, 70), (10, 255, 255)),
    ((170, 120, 70), (179, 255, 255)),
)
COLOUR_MIN_RATIO: float = 0.25  # fraction of the crop that must match  # 裁切區塊中必須符合顏色的像素比例

# ---------------------------------------------------------------------------
# Takeoff and room scanning (Milestone 3) - not used yet
# 起飛與室內掃描（里程碑 3）— 尚未使用
# ---------------------------------------------------------------------------

TAKEOFF_HEIGHT_CM: int = 100
SCAN_YAW_SPEED: int = 20  # clamped by MAX_YAW_SPEED  # 會被 MAX_YAW_SPEED 夾住
MAX_SCAN_TIME_S: float = 60.0
HOVER_SETTLE_S: float = 2.0  # let the drone stabilise before acting on video  # 先讓無人機穩定，再依影像動作

# ---------------------------------------------------------------------------
# Visual servoing (Milestone 4) - not used yet
# 視覺伺服（里程碑 4）— 尚未使用
# ---------------------------------------------------------------------------

ALIGNMENT_TOLERANCE_PX: int = 40  # deadband around the frame centre  # 畫面中心附近的死區
KP_YAW: float = 0.08  # yaw_command = KP_YAW * error_x, then clamped  # 偏航指令 = KP_YAW * error_x，再夾限
YAW_SMOOTHING: float = 0.6  # 0 = no smoothing, ->1 = heavy smoothing  # 0 = 不平滑，越接近 1 越平滑
MIN_DETECTION_FRAMES: int = 3  # consecutive hits before acting on a target  # 連續命中幾幀才開始對目標動作

# ---------------------------------------------------------------------------
# Forward approach (Milestone 5) - not used yet
# 前進接近（里程碑 5）— 尚未使用
# ---------------------------------------------------------------------------

# IMPORTANT: these are ratios of bounding-box area to frame area. They are an
# image-space proxy for "the object looks closer", NOT a distance measurement.
# A large mug and a small mug stop at completely different real distances.
# 重要：這些是「框面積 / 畫面面積」的比例。它只是影像空間裡「看起來比較近」的代理指標，
# 不是真實距離。大馬克杯和小紙杯會在完全不同的實際距離停下來。
APPROACH_THRESHOLD: float = 0.02  # below this the target is considered far  # 低於此值視為目標還很遠
SLOW_DOWN_THRESHOLD: float = 0.08  # start easing off here  # 到這裡開始減速
STOP_THRESHOLD: float = 0.15  # stop forward motion and hover  # 停止前進並懸停

FAST_APPROACH_SPEED: int = 15  # clamped by MAX_FORWARD_SPEED  # 會被 MAX_FORWARD_SPEED 夾住
SLOW_APPROACH_SPEED: int = 8

# ---------------------------------------------------------------------------
# BCI / ErrP (Milestone 7 simulated, Milestone 8 real) - not used yet
# BCI / ErrP（里程碑 7 模擬，里程碑 8 實機）— 尚未使用
# ---------------------------------------------------------------------------

BCI_ENABLED: bool = False
BCI_MODE: str = "simulated"  # "simulated" | "lsl"  # 「模擬」或「LSL 串流」
ERRP_THRESHOLD: float = 0.7
ERRP_CONFIRMATION_WINDOWS: int = 2  # consecutive windows above threshold  # 連續幾個視窗超過門檻才算確認
BCI_STOP_HOLD_TIME: float = 3.0  # seconds to stay stopped after a trigger  # 觸發後維持停止的秒數
BCI_STOP_AUTO_LAND: bool = False  # hover on BCI stop, do not land by default  # BCI 停止時預設懸停，不自動降落

# ---------------------------------------------------------------------------
# Future EEG parameters - deliberately unset
# 未來的 EEG 參數 — 刻意不填
# ---------------------------------------------------------------------------
# These depend on the actual 32-channel amplifier and the experimental
# protocol. Guessing them would produce a classifier that looks plausible and
# is scientifically meaningless, so they stay None until the hardware and the
# paradigm are confirmed.
# 這些取決於實際的 32 通道放大器與實驗典範。亂猜會做出「看起來合理、科學上沒意義」
# 的分類器，所以在硬體與典範確認前一律維持 None。

EEG_SAMPLING_RATE_HZ = None
EEG_CHANNEL_NAMES = None
EEG_BANDPASS_HZ = None  # e.g. (1.0, 10.0) for ErrP, to be confirmed  # 例如 ErrP 用 (1.0, 10.0)，待確認
EEG_EPOCH_WINDOW_S = None  # e.g. (-0.2, 0.8) around the event, to be confirmed  # 例如事件前後 (-0.2, 0.8)，待確認
EEG_REFERENCE = None
ERRP_CLASSIFIER_PATH = None
LSL_STREAM_NAME = None
