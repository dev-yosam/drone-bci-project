# Tello Autonomous Target Navigation with EEG ErrP Safety Veto
# Tello 自主目標導航，搭配 EEG ErrP 安全否決

A DJI Tello EDU finds a cup indoors, aligns with it, flies towards it, and hovers about 40 cm in front of it.
一台 DJI Tello EDU 在室內自動找到杯子、對準、飛過去，並在杯子前約 40 公分懸停。

Detection uses YOLO11n, and flight control uses image-based visual servoing.
目標偵測使用 YOLO11n，飛行控制使用以影像誤差為基礎的視覺伺服。

## Status
## 目前進度

| Milestone<br>里程碑 | Scope<br>內容 | Status<br>狀態 |
| --- | --- | --- |
| 1 | Connection, video, HUD, battery, and safe shutdown.<br>連線、影像、HUD、電量，以及安全關閉。 | Done.<br>完成。 |
| 2 | YOLO detection, colour filter, and target selection.<br>YOLO 偵測、顏色濾鏡，以及目標選擇。 | Done.<br>完成。 |
| 3 | Takeoff, hover, and yaw scan.<br>起飛、懸停，以及偏航掃描。 | Done, and flight-tested.<br>完成，並已真飛驗證。 |
| 4 | Visual alignment: yaw first, then height.<br>視覺對齊：先左右，再上下。 | Done, and flight-tested.<br>完成，並已真飛驗證。 |
| 5 | Forward approach and automatic stop.<br>前進接近與自動停止。 | Done, and flight-tested.<br>完成，並已真飛驗證。 |

The project scope ends at Milestone 5.
專案範圍到里程碑 5 為止。

A few prototypes for future extensions remain in the code, but they are not required.
程式裡保留了幾個延伸用的雛型，但不是必要功能。

- Pressing `B` simulates a "something is wrong" stop signal, and the drone hovers.
  按 `B` 會模擬「感覺不對」的停止訊號，無人機會懸停。
- Pressing `R` clears that stop.
  按 `R` 會解除這個停止。
- The EEG / BCI parameters in `config.py` are all left as `None`.
  `config.py` 裡的 EEG／BCI 參數全部留空（`None`）。
- No EEG device is connected.
  沒有接任何腦波裝置。

## Flight sequence
## 飛行流程

After `T` is pressed, the whole sequence runs automatically.
按下 `T` 之後，整個流程全自動執行。

```text
GROUND ─T→ TAKEOFF → HOVER_SETTLE → SCANNING ─cup seen→ ALIGNING ─yaw centred→ TARGET_LOCKED
                                       ↑                    │                        │
                                       └──── target lost ───┘                 height centred
                                                                                     ↓
                                                    ARRIVED ←─box large enough── APPROACHING
```

"cup seen" means a cup was detected.
「cup seen」代表偵測到杯子。

"yaw centred" means the cup is horizontally centred.
「yaw centred」代表杯子左右置中。

"height centred" means the cup is vertically centred.
「height centred」代表杯子上下置中。

"target lost" means the cup disappeared.
「target lost」代表目標消失。

"box large enough" means the box ratio reached the stop threshold.
「box large enough」代表框比例已達停止門檻。

| State<br>狀態 | What the drone does<br>無人機在做什麼 |
| --- | --- |
| `TAKEOFF` / `HOVER_SETTLE` | It takes off and hovers for 2 seconds to stabilise.<br>起飛後懸停 2 秒，讓機身穩定。 |
| `SCANNING` | It rotates clockwise while looking for the cup, for up to 60 seconds.<br>原地順時針旋轉找杯子，最多 60 秒。 |
| `ALIGNING` | After 3 consecutive detections, it turns until the cup is horizontally centred.<br>連續 3 幀看到杯子後，轉到杯子在畫面左右中央。 |
| `TARGET_LOCKED` | It climbs or descends until the cup is also vertically centred.<br>升降，直到杯子也在畫面上下中央。 |
| `APPROACHING` | It flies forward while correcting yaw and height.<br>往前飛，同時持續修正左右與高度。 |
| `APPROACHING` | The larger the box, the slower it flies.<br>框越大，飛得越慢。 |
| `ARRIVED` | The box ratio has reached the stop threshold, so it hovers.<br>框比例到達停止門檻，於是懸停。 |

| State<br>狀態 | Cause<br>原因 | Behaviour<br>行為 |
| --- | --- | --- |
| `SCAN_TIMEOUT` | No cup was found within 60 seconds.<br>掃描 60 秒沒有找到杯子。 | It hovers.<br>懸停。 |
| `APPROACH_TIMEOUT` | It did not arrive within 15 seconds.<br>接近 15 秒還沒到達。 | It hovers.<br>懸停。 |
| `VIDEO_LOST` | The video stream was interrupted.<br>影像中斷。 | It hovers, then resumes when video returns.<br>先懸停，影像恢復後繼續。 |
| `BCI_STOP` | `B` was pressed.<br>按了 `B`。 | It hovers, then resumes after `R`.<br>先懸停，按 `R` 後繼續。 |
| `LANDING` | `L` was pressed, the battery is at or below 20%, or the flight reached 120 seconds.<br>按了 `L`、電量在 20% 以下，或飛行滿 120 秒。 | It lands.<br>降落。 |

## Control
## 控制方式

All errors are measured in pixels.
所有誤差都以像素計算。

They are not measured in metres.
不是以公尺計算。

`err x` and `err y` are the offsets of the cup centre from the frame centre.
`err x` 與 `err y` 是杯子中心相對於畫面中心的偏移量。

- A positive `err x` means the cup is to the right, so the drone turns clockwise.
  `err x` 為正代表杯子在右邊，所以無人機順時針轉。
- A positive `err y` means the cup is below centre, so the drone descends.
  `err y` 為正代表杯子在畫面中心下方，所以無人機下降。
- The image y-axis points down.
  影像的 y 軸朝下。

Yaw and height alignment use a proportional controller with smoothing.
左右與上下對齊使用比例控制，並加上平滑。

```text
yaw = clamp(KP_YAW       × err_x, ±30)
ud  = clamp(-KP_VERTICAL × err_y, ±20)
```

- Commands pass through exponential smoothing (`YAW_SMOOTHING = 0.6`).
  指令會經過指數平滑（`YAW_SMOOTHING = 0.6`）。
- Smoothing reduces jitter.
  平滑用來減少抖動。
- An error within 40 px counts as centred, and the command is 0.
  誤差在 40 px 以內視為置中，指令為 0。
- The Tello ignores very small RC values, so any command outside the deadband is at least 8.
  Tello 對太小的 RC 值沒有反應，所以死區外的指令至少為 8。
- If the cup drifts more than 80 px sideways after locking, the drone returns to `ALIGNING`.
  鎖定後若杯子左右偏出 80 px，無人機會回到 `ALIGNING` 重新轉正。

Forward speed depends on how much of the frame the YOLO box covers.
前進速度取決於 YOLO 框佔畫面的比例。

The HUD shows this value as `BBox ratio`.
HUD 上這個值顯示為 `BBox ratio`。

| Box ratio<br>框比例 | Forward speed<br>前進速度 | Approx. distance for an 8×10 cm cup<br>8×10 公分杯子的約略距離 |
| --- | --- | --- |
| ≤ 0.003 | 15 | More than 1.2 m.<br>超過 1.2 公尺。 |
| 0.003 – 0.008 | 15 down to 8, linearly.<br>從 15 線性降到 8。 | 1.2 m to 0.8 m.<br>1.2 公尺到 0.8 公尺。 |
| 0.008 – 0.03 | 8 | 0.8 m to 0.4 m.<br>0.8 公尺到 0.4 公尺。 |
| ≥ 0.03 | 0, then `ARRIVED`.<br>0，接著進入 `ARRIVED`。 | About 0.4 m.<br>約 0.4 公尺。 |

Safety rules during the approach:
接近過程中的安全規則：

- If a single frame has no detection, forward motion stops immediately.
  只要有一幀沒偵測到杯子，就立刻停止前進。
- It does not keep flying on an old box.
  不會拿舊的框繼續飛。
- After 10 consecutive misses, the drone returns to scanning.
  連續 10 幀找不到，就回到掃描。
- If the cup drifts more than 80 px sideways, the drone stops and re-aligns first.
  杯子左右偏出 80 px 時，會先停下並重新轉正。
- After a `B` stop or a video dropout, the drone re-centres in `TARGET_LOCKED` before moving forward again.
  按 `B` 停止或影像中斷恢復後，會先在 `TARGET_LOCKED` 重新置中，再繼續前進。

The box ratio only says how close the cup looks.
框比例只代表杯子看起來多近。

It is not a real distance.
它不是真實距離。

A larger cup stops further away.
杯子越大，停得越遠。

A small paper cup stops at about 38 cm, and a large mug stops at about 50 cm.
小紙杯約在 38 公分停下，大馬克杯約在 50 公分停下。

## Hardware and network
## 硬體與網路

- The drone is a DJI Tello EDU in station mode, joined to a Wi-Fi router.
  無人機是 DJI Tello EDU，使用 Station 模式，連到一台 Wi-Fi 路由器。
- The laptop must be on the same router as the drone.
  筆電必須和無人機連在同一台路由器上。
- The drone IP is set by `TELLO_HOST` in `config.py`, or with `--host` on the command line.
  無人機 IP 由 `config.py` 的 `TELLO_HOST` 設定，也可以用命令列的 `--host` 指定。
- In direct mode, connect the laptop to the `TELLO-XXXXXX` hotspot and use `--host 192.168.10.1`.
  直連模式時，把筆電連到 `TELLO-XXXXXX` 熱點，並使用 `--host 192.168.10.1`。
- YOLO runs on the laptop CPU.
  YOLO 在筆電的 CPU 上執行。
- No GPU is required.
  不需要 GPU。

## Installation
## 安裝

The project has been tested with Python 3.13.
本專案已在 Python 3.13 上測試過。

Run these commands from the project folder.
以下指令請在專案資料夾內執行。

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

The YOLO weights are stored in `models/yolo11n.pt`.
YOLO 權重存放在 `models/yolo11n.pt`。

The Tello router has no internet access, so download the weights beforehand while you are online.
Tello 的路由器沒有網際網路，所以請先在有網路時下載好權重。

```powershell
cd models
python -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"
```

## Running
## 執行

Run these commands from the project folder.
以下指令請在專案資料夾內執行。

```powershell
python main.py
python main.py --live
```

`python main.py` runs in DRY_RUN.
`python main.py` 以 DRY_RUN 執行。

The state machine runs, but the motors stay off.
狀態機會照常運行，但馬達不會轉動。

`python main.py --live` is a real flight.
`python main.py --live` 是真飛。

Other options:
其他選項：

```powershell
python main.py --target bottle
python main.py --colour
python main.py --no-detect
python main.py --no-video
python main.py --host 192.168.10.1
```

`--target bottle` switches to another target.
`--target bottle` 會換成另一個目標。

Any of the 80 COCO classes can be used.
80 個 COCO 類別都可以使用。

`--colour` accepts red targets only.
`--colour` 只接受紅色目標。

`--no-detect` shows video without YOLO.
`--no-detect` 只顯示影像，不跑 YOLO。

`--no-video` shows telemetry only.
`--no-video` 只顯示遙測。

`--no-video` cannot be combined with `--live`.
`--no-video` 不能和 `--live` 一起用。

`--host 192.168.10.1` selects direct hotspot mode.
`--host 192.168.10.1` 是直連熱點模式。

### Keys
### 按鍵

Keys only work while the OpenCV window has focus, so click the window first.
按鍵只有在 OpenCV 視窗有焦點時才有效，請先點一下視窗。

| Key<br>按鍵 | Action<br>功能 |
| --- | --- |
| `T` | Take off and start the automatic sequence.<br>起飛並開始自動流程。 |
| `L` | Land.<br>降落。 |
| `E` | Cut the motors. The drone drops. Use this only if it is out of control.<br>切斷馬達，無人機會直接掉落。只在失控時使用。 |
| `Q` / `Esc` | Land if it is flying, then quit.<br>若正在飛行就先降落，再結束程式。 |
| `B` | Simulate a stop signal.<br>模擬停止訊號。 |
| `R` | Clear the simulated stop.<br>解除模擬的停止。 |
| `C` | Toggle the colour filter.<br>切換顏色濾鏡。 |

### HUD

The window shows text like this.
視窗會顯示類似下面的文字。

```text
STATE: APPROACHING
Battery: 74%   H: 70 cm
Loop:  61.2 fps   Video:  29.4 fps
Link: OK
Target: cup 0.87
BBox ratio: 0.0124  err x=+12 y=-8
Colour filter: OFF
RC yaw=+0  lr=+0 fb=+8 ud=+0
MODE: LIVE
```

- **Video** is the rate at which new frames actually arrive, and it should be close to 30.
  **Video** 是實際收到新影格的速度，應該接近 30。
- A high **Loop** with **Video** at 0 means there is no image.
  **Loop** 很高但 **Video** 是 0，代表沒有影像。
- The green box is the selected target, and the line from the centre is the alignment error.
  綠框是被選中的目標，從中心連出的線就是對齊誤差。
- Amber boxes are the other detections.
  黃框是其他偵測結果。
- The drone flies only when the HUD shows `MODE: LIVE`.
  只有 HUD 顯示 `MODE: LIVE` 時，無人機才會真的飛。
- In `DRY_RUN`, pressing `T` only steps through the states.
  在 `DRY_RUN` 下按 `T`，只會走狀態，不會起飛。

### Logs
### 紀錄檔

Each run writes two files into `logs/`.
每次執行會在 `logs/` 產生兩個檔案。

- `run_*.log` records state changes and errors.
  `run_*.log` 記錄狀態切換與錯誤訊息。
- `flight_*.csv` has one row per frame.
  `flight_*.csv` 每一幀一列。
- Each row contains the state, battery, target box, box ratio, `err x`, `err y`, and the RC command that was sent.
  每一列包含狀態、電量、目標框、框比例、`err x`、`err y`，以及送出的 RC 指令。

## Pre-flight checklist
## 真飛檢查清單

1. Charge the battery fully, power the drone on, and leave it on the ground for 30 seconds.
   把電池充滿，開機後放在地上 30 秒。
   The battery reading should stay steady.
   電量讀數應該保持穩定。
2. Put the laptop and the drone on the same network, and confirm that `ping` reaches the drone.
   讓筆電和無人機連在同一個網路，並確認 `ping` 得到無人機。
3. Use a clear, well-lit room with a textured floor, and leave at least 2 m in front of the drone.
   使用淨空、燈光充足、地板有紋理的房間，機頭前方至少留 2 公尺。
4. Place the cup 1.5 to 2 m in front of the drone, at roughly the same height.
   把杯子放在機頭前方 1.5 到 2 公尺，高度大約與機身相同。
5. Run `python main.py --live`, and check that the HUD shows a real battery level and **Video** near 30 fps.
   執行 `python main.py --live`，確認 HUD 顯示真實電量，且 **Video** 接近 30 fps。
6. Click the video window, press `T`, and keep a finger ready on `L`.
   點一下影像視窗，按 `T`，手指隨時準備按 `L`。

## Tuning
## 調整參數

All parameters live in `config.py`.
所有參數都在 `config.py`。

To set the stopping distance, run in DRY_RUN and place the cup where you want the drone to stop.
要調整停下的距離，先用 DRY_RUN，把杯子放在你希望停下的位置。

Read `BBox ratio` on the HUD, and set `STOP_THRESHOLD` to that value.
讀 HUD 上的 `BBox ratio`，再把 `STOP_THRESHOLD` 設成那個值。

A smaller value makes the drone stop further away.
數值越小，無人機停得越遠。

| Parameter<br>參數 | Default<br>預設 | Purpose<br>用途 |
| --- | --- | --- |
| `TARGET_OBJECT` / `TARGET_CONFIDENCE` | `cup` / 0.5 | Target class and confidence threshold.<br>目標類別與信心門檻。 |
| `DETECT_EVERY_N_FRAMES` | 2 | Run YOLO every N frames. On CPU this is about 45 ms per frame.<br>每 N 幀跑一次 YOLO。在 CPU 上大約 45 毫秒一張。 |
| `SCAN_YAW_SPEED` | 20 | Scan rotation speed.<br>掃描轉速。 |
| `ALIGNMENT_TOLERANCE_PX` | 40 | Centring deadband, in pixels.<br>置中死區，單位是像素。 |
| `ALIGN_REACQUIRE_PX` | 80 | Sideways drift that triggers re-alignment.<br>左右偏出多少就要重新轉正。 |
| `KP_YAW` / `KP_VERTICAL` | 0.08 | Proportional gains for alignment.<br>對齊用的比例增益。 |
| `STOP_THRESHOLD` | 0.03 | Box ratio at which the drone stops, about 40 cm for an 8×10 cm cup.<br>停下時的框比例。8×10 公分的杯子大約是 40 公分。 |
| `FAST_APPROACH_SPEED` / `SLOW_APPROACH_SPEED` | 15 / 8 | Approach speeds.<br>接近速度。 |
| `MIN_BATTERY_TAKEOFF` / `MIN_BATTERY_ABORT` | 40% / 20% | Minimum battery for takeoff, and the level that forces a landing.<br>起飛所需的最低電量，以及強制降落的電量。 |
| `MAX_FLIGHT_TIME_S` | 120 | Maximum time of one flight, in seconds.<br>單次飛行的時間上限，單位是秒。 |
| `RC_SEND_HZ` | 10 | Maximum RC command rate.<br>RC 指令的頻率上限。 |

Detection tips:
偵測調整建議：

- If no box appears, check that the object is one of the 80 COCO classes, or lower `TARGET_CONFIDENCE` to 0.3.
  如果完全沒有框，請確認物件屬於 COCO 的 80 個類別之一，或把 `TARGET_CONFIDENCE` 降到 0.3。
- A water bottle is the class `bottle`.
  水瓶的類別是 `bottle`。
- It is not the class `cup`.
  它不是 `cup`。
- If the box flickers, the confidence is near the threshold.
  如果框一直閃爍，代表信心值在門檻附近。
- Improve the lighting, or move the object closer.
  請改善光線，或把物件移近一點。
- The selector picks the largest box, which is usually the nearest one.
  選擇器會挑最大的框，通常也是最近的那個。
- It does not pick the box with the highest confidence.
  它不會挑信心最高的框。
- That rule is in `vision/target_selector.py`.
  這條規則寫在 `vision/target_selector.py`。
- If the colour filter rejects a red cup, press `C` to compare, read `Colour match` on the HUD, and set `COLOUR_MIN_RATIO` below that reading.
  如果顏色濾鏡拒絕紅杯，按 `C` 比較，讀 HUD 上的 `Colour match`，再把 `COLOUR_MIN_RATIO` 設得比那個讀數低。

## Project layout
## 專案結構

```text
tello_bci_project/
├── main.py
├── config.py
├── drone/tello_controller.py
├── navigation/flight_supervisor.py
├── vision/yolo_detector.py
├── vision/target_selector.py
├── vision/colour_filter.py
├── vision/detection.py
├── utils/logger.py
├── utils/visualization.py
├── models/
├── tests/
└── logs/
```

- `main.py` runs the main loop, the HUD, and the keys.
  `main.py` 負責主迴圈、HUD 與按鍵。
- `config.py` holds every tunable parameter.
  `config.py` 存放所有可調參數。
- `drone/tello_controller.py` is the only module that talks to the drone.
  `drone/tello_controller.py` 是唯一與無人機通訊的模組。
- DRY_RUN is enforced inside that module.
  DRY_RUN 在這個模組裡生效。
- `navigation/flight_supervisor.py` is the flight state machine, including alignment and approach control.
  `navigation/flight_supervisor.py` 是飛行狀態機，包含對齊與接近控制。
- `vision/yolo_detector.py` wraps YOLO and turns a frame into detections.
  `vision/yolo_detector.py` 包裝 YOLO，把一張影格轉成偵測清單。
- `vision/target_selector.py` picks exactly one target from those detections.
  `vision/target_selector.py` 從多個偵測中選出唯一目標。
- `vision/colour_filter.py` is an optional HSV colour check.
  `vision/colour_filter.py` 是可選的 HSV 顏色驗證。
- `vision/detection.py` computes box geometry: centre, area, and error.
  `vision/detection.py` 計算框的幾何：中心、面積與誤差。
- `utils/logger.py` writes the log and the per-frame CSV.
  `utils/logger.py` 寫日誌與逐幀 CSV。
- `utils/visualization.py` draws the HUD and the boxes.
  `utils/visualization.py` 繪製 HUD 與框線。
- `models/` stores the YOLO weights offline.
  `models/` 離線保存 YOLO 權重。
- `tests/` holds unit tests, and they do not need a drone.
  `tests/` 放單元測試，不需要無人機。
- `logs/` stores the run logs.
  `logs/` 存放執行紀錄。

`vision/` never imports `drone/`.
`vision/` 不會匯入 `drone/`。

The whole detection pipeline can therefore be tested on still images, without a drone.
所以整條偵測流程可以用靜態圖片測試，不需要無人機。

## Tests
## 測試

Run this command from the project folder.
以下指令請在專案資料夾內執行。

```powershell
python -m unittest discover -s tests -t . -v
```

There are 65 tests, and none of them need a drone.
共有 65 個測試，都不需要無人機。

They cover box geometry, target selection, the colour filter, and the flight state machine.
涵蓋框的幾何計算、目標選擇、顏色濾鏡，以及飛行狀態機。

The state-machine tests include scanning, alignment, approach, stopping, and interruptions.
狀態機測試包含掃描、對齊、接近、停止，以及各種中斷。

They also cover RC send-rate limiting.
也涵蓋 RC 送出頻率的限制。

## Known limitations
## 已知限制

- There is no range sensor.
  沒有測距感測器。
- Stopping uses the box-area ratio, so cups of different sizes stop at different distances.
  停止依據的是框面積比例，所以不同大小的杯子會停在不同距離。
- There is no tracker.
  沒有追蹤器。
- Each frame is detected on its own, so two similar cups can swap which one is the target.
  每一幀各自偵測，兩個相似的杯子可能互換誰是目標。
- There is no obstacle avoidance.
  沒有避障。
- The Tello has no forward range sensor, so it cannot see walls, table legs, or people.
  Tello 前方沒有測距感測器，看不到牆、桌腳或人。
- Always watch the flight.
  飛行時一定要有人看著。
- The Tello holds position with its downward camera, so it drifts over dark, plain, or reflective floors.
  Tello 靠下方鏡頭定位，在太暗、單色或反光的地板上會漂移。
- The colour filter uses fixed thresholds, so it must be tuned again in each room.
  顏色濾鏡使用固定門檻，換房間要重新調整。
