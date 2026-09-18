# Tello Autonomous Target Navigation with EEG ErrP Safety Veto
# Tello 自主目標導航，搭配 EEG ErrP 安全否決

A research prototype: one DJI Tello EDU searches an indoor room for a target
object using YOLO, approaches it, and stops before reaching it. A 32-channel
EEG ErrP detector is added later as a human-in-the-loop safety veto.
研究原型：一台 DJI Tello EDU 用 YOLO 在室內尋找目標物件、接近它，並在到達前停下。
之後會加入 32 通道 EEG ErrP 偵測器，作為人在迴路中的安全否決。

The project is built milestone by milestone. **This repository currently
implements Milestones 1, 2 and 3.**
專案依里程碑逐步建構。**目前這個儲存庫已實作里程碑 1、2 與 3。**

## Current status
## 目前進度

| Milestone | Scope 範圍 | State 狀態 |
| --- | --- | --- |
| 1 | Connect, video stream, display, battery, safe shutdown / 連線、影像串流、顯示、電量、安全關閉 | Implemented 已完成 |
| 2 | YOLO detection, colour filter, target selection / YOLO 偵測、顏色濾鏡、目標選擇 | Implemented 已完成 |
| 3 | Takeoff, hover, yaw scan / 起飛、懸停、偏航掃描 | Implemented 已完成 |
| 4 | Visual servo yaw alignment / 視覺伺服偏航對齊 | Not started 尚未開始 |
| 5 | Forward approach, automatic stop / 前進接近、自動停止 | Not started 尚未開始 |
| 6 | Safety manager, watchdog, emergency controls / 安全管理、看門狗、緊急控制 | Not started 尚未開始 |
| 7 | Simulated BCI ErrP trigger / 模擬 BCI ErrP 觸發 | Stub only (key `B` toggles the HUD) 僅有雛型（按 `B` 切換 HUD） |
| 8 | Real EEG streaming and ErrP classifier / 真實 EEG 串流與 ErrP 分類器 | Not started 尚未開始 |

Milestones 1-2 never took off. Milestone 3 **can** take off, but only if you
pass ``--live``. The default is still ``DRY_RUN``: press `T` and the HUD walks
GROUND → TAKEOFF → HOVER_SETTLE → SCANNING without spinning the motors.
里程碑 1-2 不會起飛。里程碑 3 **可以**起飛，但只有加上 ``--live`` 才會真的動。
預設仍是 ``DRY_RUN``：按 `T`，HUD 會走 GROUND → TAKEOFF → HOVER_SETTLE → SCANNING，馬達不會轉。

## Layout
## 目錄結構

```text
tello_bci_project/
├── main.py                   entry point and main loop  # 進入點與主迴圈
├── config.py                 every tunable parameter  # 所有可調參數
├── drone/tello_controller.py the only module that talks to the drone  # 唯一跟無人機通訊的模組
├── navigation/flight_supervisor.py  takeoff / hover / yaw-scan state machine  # 起飛／懸停／掃描狀態機
├── vision/detection.py       the Detection record and its geometry  # Detection 紀錄與其幾何量
├── vision/yolo_detector.py   ultralytics wrapper, frame -> detections  # ultralytics 包裝，影格 -> 偵測清單
├── vision/colour_filter.py   optional HSV colour validation  # 可選的 HSV 顏色驗證
├── vision/target_selector.py picks exactly one target, deterministically  # 以確定性規則選出正好一個目標
├── utils/logger.py           logging + per-frame CSV  # 紀錄 + 逐幀 CSV
├── utils/visualization.py    HUD and detection overlay  # HUD 與偵測疊圖
├── models/                   YOLO weights, kept offline on purpose  # YOLO 權重，刻意離線保存
├── tests/                    hardware-free unit tests  # 不需硬體的單元測試
└── logs/                     run logs and flight CSVs  # 執行日誌與飛行 CSV
```

`vision/` never imports `drone/`. The whole perception path can therefore be
tested on a still image with no drone present, which is how the unit tests and
the offline checks below work.
`vision/` 絕不匯入 `drone/`。因此整條感知路徑可以在沒有無人機的靜態圖片上測試，單元測試與下面的離線檢查就是這樣做的。

## Installation
## 安裝

```powershell
cd C:\Users\sam84\Documents\EEG
.\EGG\Scripts\Activate.ps1
pip install -r tello_bci_project\requirements.txt
```

The weights live in `models/yolo11n.pt` (5.4 MB) and are already downloaded.
**Download them while you still have internet access.** Once the laptop joins
the Tello router there is no route to the internet, and ultralytics cannot
fetch a missing weight file mid-session. To fetch them again:
權重放在 `models/yolo11n.pt`（5.4 MB），已經下載過。
**請在還有網際網路時下載。** 筆電一旦加入 Tello 路由器，就沒有對外路由，ultralytics 無法在執行中補抓缺失權重。若要再抓一次：

```powershell
cd C:\Users\sam84\Documents\EEG\tello_bci_project\models
python -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"
```

## Running
## 執行

```powershell
cd C:\Users\sam84\Documents\EEG\tello_bci_project
python main.py
```

Options:  # 選項

```powershell
python main.py --host 192.168.10.1   # direct AP mode instead of the router  # 直連 AP 模式，不走路由器
python main.py --no-video            # telemetry only, if streaming fails  # 串流失敗時只看遙測
python main.py --no-detect           # video without YOLO  # 有影像但不跑 YOLO
python main.py --target bottle       # any of the 80 COCO classes  # 80 個 COCO 類別之一
python main.py --colour              # require the target to be red  # 要求目標必須是紅色
python main.py --live                # real takeoff / scan. DRY_RUN off.  # 真正起飛／掃描，關閉 DRY_RUN
```

An unknown `--target` exits immediately and prints the 80 class names the
model actually supports.
未知的 `--target` 會立刻結束，並列出模型實際支援的 80 個類別名。

Keys work only while the OpenCV window has focus. This is deliberate: a global
keyboard hook would fire while you are typing in another application.
按鍵只有 OpenCV 視窗在前景時才有效。這是刻意的：全域鍵盤鉤子會在你打字到別的程式時也觸發。

| Key 按鍵 | Action 動作 |
| --- | --- |
| `T` | Takeoff → hover settle → yaw scan / 起飛 → 懸停穩定 → 偏航掃描 |
| `L` | Land / 降落 |
| `E` | Emergency: motors off, the drone drops / 緊急停止：切馬達，無人機會掉下來 |
| `Q` or `Esc` | Land if flying, then quit / 若在飛先降落，再結束 |
| `B` | Simulate an ErrP trigger (hover while scanning) / 模擬 ErrP（掃描中改為懸停） |
| `R` | Clear the simulated BCI stop / 清除模擬的 BCI 停止 |
| `C` | Toggle the colour filter without restarting / 不重開程式就能切換顏色濾鏡 |

## Expected output
## 預期輸出

```text
02:45:44 INFO    main | Starting. Log file: ...\logs\run_20260824_024544.log
02:45:46 INFO    vision.yolo_detector | Loading YOLO model ...\models\yolo11n.pt on cpu
02:45:46 INFO    vision.yolo_detector | Model loaded with 80 classes
02:45:46 INFO    drone.tello_controller | Connecting to Tello at 192.168.0.220 (dry_run=True)
02:45:47 INFO    drone.tello_controller | Connected. Battery 74%, SDK 30
02:45:49 INFO    drone.tello_controller | First video frame received
```

A window opens showing the camera image, a centre crosshair, a box around every
detection, and an overlay:
會開啟視窗，顯示相機畫面、中心十字準星、每個偵測的框，以及疊圖：

```text
STATE: GROUND
Battery: 74%   H: 0 cm
Loop:  61.2 fps   Video:  29.4 fps
Link: OK
Target: cup 0.87
BBox ratio: 0.0412  err x=-118 y=+34
Colour filter: OFF
RC yaw=+0  lr=+0 fb=+0 ud=+0
BCI: NORMAL p=0.00
MODE: DRY_RUN (no movement)
T takeoff  L land  E emergency  Q quit  B ErrP  R  C
```

Two rates are shown because they mean different things. **Loop** is how fast
the display refreshes; **Video** is how fast new frames actually arrive from
the drone, and it should sit near 30. Only the video rate says anything about
how fresh the image is, which is why the CSV records that one.
畫面上有兩個幀率，因為意義不同。**Loop** 是畫面刷新多快；**Video** 是無人機真正送來新影格的速度，應接近 30。
只有 Video 能反映影像新鮮度，所以 CSV 記的是這一個。

The selected target is outlined in green with a line drawn from the frame
centre to the target centre - that line is the alignment error the yaw
controller will act on in Milestone 4. Other detections are outlined in amber.
Rejected objects are still drawn on purpose: when the drone ignores something,
you need to see whether it was never detected or detected and then rejected.
被選中的目標用綠框，並從畫面中心連一線到目標中心——這條線就是里程碑 4 偏航控制器會處理的對齊誤差。
其他偵測用黃框。被拒絕的物件也刻意畫出來：無人機忽略某物時，你必須看得出是「從沒偵測到」還是「偵測到後被拒」。

Two files appear in `logs/`: `run_*.log` (diagnostics) and `flight_*.csv` (one
row per frame, including the target class, confidence, box corners, box ratio
and centring error).
`logs/` 會出現兩個檔：`run_*.log`（診斷）與 `flight_*.csv`（每幀一列，含目標類別、信心、框角落、框比例、置中誤差）。

## Milestone 3: takeoff, hover, yaw scan
## 里程碑 3：起飛、懸停、偏航掃描

The flight supervisor is an explicit state machine in
`navigation/flight_supervisor.py`. It never talks to UDP; `TelloController`
still swallows movement when `DRY_RUN` is on.
飛行監督器是 `navigation/flight_supervisor.py` 裡的明確狀態機。它不碰 UDP；
`DRY_RUN` 開啟時，`TelloController` 仍會吞掉移動指令。

```text
GROUND --T--> TAKEOFF --> HOVER_SETTLE --> SCANNING --target--> TARGET_LOCKED
                                   |              |
                                   |              +--timeout--> SCAN_TIMEOUT
                                   |              +--B--> BCI_STOP (hover, R resumes)
                                   |              +--video loss--> VIDEO_LOST (hover)
                                   +--L / low battery / max time--> LANDING --> GROUND
```

**Always dry-run first.** Connect as usual, press `T`, confirm the HUD walks
those states and `RC yaw` becomes about `+20` during SCANNING. Put a cup in
front of the camera: after three consecutive detections the state should become
`TARGET_LOCKED` and yaw should return to 0. Press `L` to go back to `GROUND`.
**一定要先 dry-run。** 照常連線，按 `T`，確認 HUD 會走這些狀態，掃描時 `RC yaw` 大約 `+20`。
把杯子放到鏡頭前：連續三幀偵測到之後應變成 `TARGET_LOCKED`，偏航回到 0。按 `L` 回到 `GROUND`。

Only then, with a clear room, battery above 40%, textured floor, and the
OpenCV window focused:

```powershell
python main.py --live
```

`E` cuts the motors. The drone will drop. That is the last resort, not a
normal land.
`E` 會切斷馬達，無人機會掉下來。那是最後手段，不是普通降落。

Milestone 3 **does not fly towards the cup.** Locking the target only stops
the yaw scan. Alignment is Milestone 4; approach is Milestone 5.
里程碑 3 **不會朝杯子飛過去。** 鎖住目標只是停止偏航掃描。對齊是里程碑 4，接近是里程碑 5。

## Tuning detection
## 調整偵測

Do this on the ground, with the drone connected and the object in your hand.
在地上做：無人機已連線，目標物拿在手上。

- **Object never appears.** Check that it is one of the 80 COCO classes. A
  water bottle is `bottle`, not `cup`. Lower `TARGET_CONFIDENCE` in
  `config.py` to 0.3 and see whether a box appears.
  **物件完全不出現。** 先確認它屬於 80 個 COCO 類別之一。水瓶是 `bottle`，不是 `cup`。把 `config.py` 的 `TARGET_CONFIDENCE` 降到 0.3，看框會不會出現。
- **Box flickers on and off.** The confidence is hovering around the
  threshold. Lower it, improve the lighting, or move closer.
  **框一直閃爍。** 信心在門檻附近晃。把門檻調低、改善光線，或靠近一點。
- **The wrong instance is chosen.** The selector deliberately prefers the
  largest box over the most confident one, because the largest is the nearest
  and therefore the one worth flying to. Change that rule in
  `vision/target_selector.py`, not in the caller.
  **選錯實例。** 選擇器刻意偏好最大框而不是最高信心，因為最大的通常最近、才值得飛過去。要改這條規則請改 `vision/target_selector.py`，不要改呼叫端。
- **The colour filter rejects your red object.** Warm indoor lighting pushes
  red towards orange. Press `C` to compare with the filter off, then read the
  `Colour match` value on the HUD and set `COLOUR_MIN_RATIO` below it. The
  ratio counts matching pixels across the whole bounding box, so a red cup on
  a white table rarely exceeds 0.5.
  **顏色濾鏡把你的紅色物件拒掉。** 室內暖光會把紅色推向橘色。按 `C` 對照關掉濾鏡的結果，再看 HUD 上的 `Colour match`，把 `COLOUR_MIN_RATIO` 設得比它低。這個比例是整個框裡符合顏色的像素，白桌上的紅杯很少超過 0.5。
- **Detection lags behind the video.** `yolo11n` at 640 px takes about 45 ms
  per frame on this laptop's CPU, roughly 22 fps against a 30 fps stream.
  `DETECT_EVERY_N_FRAMES` is set to 2 for that reason. Raise it if the loop
  feels sluggish, or lower `YOLO_IMG_SIZE` to 480.
  **偵測跟不上影像。** 這台筆電 CPU 上 `yolo11n`、640 px 約 45 ms/張，大約 22 fps，影像卻是 30 fps。因此 `DETECT_EVERY_N_FRAMES` 設為 2。若迴圈覺得卡，再調大這個值，或把 `YOLO_IMG_SIZE` 降到 480。

## Tests
## 測試

No hardware and no drone needed:
不需要硬體、也不需要無人機：

```powershell
cd C:\Users\sam84\Documents\EEG\tello_bci_project
python -m unittest discover -s tests -t . -v
```

34 tests cover bounding-box geometry, target selection, the HSV colour filter,
the flight state machine, and RC send-rate limiting. None of them need a drone.
34 個測試涵蓋框幾何、目標選擇、HSV 顏色濾鏡、飛行狀態機，以及 RC 送出限速。都不需要無人機。

## Troubleshooting
## 疑難排解

### `UDP port 8889 is already in use`
### `UDP 埠 8889 已被佔用`

A previous run or Packet Sender still holds the control port.
上一支程式或 Packet Sender 仍佔著控制埠。

```powershell
netstat -ano | findstr :8889
taskkill /PID <PID from the last column> /F
```

Packet Sender and this program cannot run at the same time.
Packet Sender 與這個程式不能同時開。

### `WinError 10055` / the window closes mid-flight
### `WinError 10055`／飛行中視窗自己關掉

An earlier build sent Tello ``rc`` on every display frame (~30 Hz). Windows
UDP buffers filled, the main loop crashed, and land/E stopped working.
`send_rc` is now capped at `RC_SEND_HZ` (10 Hz) and UDP errors are logged
instead of tearing down the window. If a live flight still dies, land with
the official Tello app, power-cycle the drone, then retry.
舊版每個顯示幀都送 Tello ``rc``（約 30 Hz）。Windows UDP 緩衝區塞滿後主迴圈崩潰，降落／E 就失效。
現在 `send_rc` 上限是 `RC_SEND_HZ`（10 Hz），UDP 錯誤只記 log、不拆視窗。
若真飛仍當掉，用官方 Tello App 降落、重開無人機再試。

### `Tello did not respond`
### `Tello 沒有回應`

- Is the drone powered on, and is the LED not flashing red? Flashing red means
  low battery, and the drone starts ignoring commands.
  無人機有開機嗎？LED 有沒有在閃紅燈？閃紅代表電量過低，無人機會開始忽略指令。
- Is this laptop on the same network as the drone? In station mode both must be
  on the router; check with `ping 192.168.0.220`.
  這台筆電與無人機是否在同一網路？Station 模式下兩邊都必須掛在路由器上；用 `ping 192.168.0.220` 確認。
- Tello leaves SDK mode after a period of inactivity. Power-cycle it and retry.
  Tello 閒置一段時間後會離開 SDK 模式。關機再開，然後重試。

### `YOLO detector unavailable`
### `YOLO 偵測器無法使用`

The program keeps running with video only rather than aborting, because losing
detection must never leave a drone in the air with no way to land it. The two
usual causes are ultralytics not being installed and the weight file being
missing while offline. See Installation above.
程式會改成只跑影像、不會中止，因為偵測消失絕不能讓無人機停在空中卻無法降落。兩個常見原因是沒裝 ultralytics，以及離線時權重檔缺失。見上方安裝一節。

### `Video stream failed` / `No video frame within 15s`
### `影像串流失敗` / `15 秒內沒有影像`

Station mode video works on this drone's firmware, but djitellopy's own source
warns it is not guaranteed: *"Currently Tello EDUs do not support video
streaming while connected to a WiFi-network."*
這台無人機的韌體在 Station 模式下影像可用，但 djitellopy 原始碼警告這不保證：*"Currently Tello EDUs do not support video streaming while connected to a WiFi-network."*

Try in this order:
依這個順序試：

1. Allow inbound UDP 11111 through Windows Firewall, and make sure the network
   profile is **Private**, not Public.
   在 Windows 防火牆允許傳入 UDP 11111，並確認網路設定檔是**私人**，不是公用。
2. Update the drone firmware with the official Tello app.
   用官方 Tello App 更新無人機韌體。
3. Fall back to direct AP mode, where video always works:
   退回直連 AP 模式，那裡影像一定可用：
   - Connect this laptop to the `TELLO-XXXXXX` hotspot.
     把這台筆電連到 `TELLO-XXXXXX` 熱點。
   - `python main.py --host 192.168.10.1`

### Wrong colours in the video window
### 視窗裡顏色不對

The frames djitellopy returns are RGB, not BGR. `TelloController.read_frame`
already converts them. If you bypass the controller, remember to convert, or
the colour filter will match cyan when you asked for red.
djitellopy 回傳的影格是 RGB，不是 BGR。`TelloController.read_frame` 已經轉換過。
若你繞過控制器，記得自己轉，否則顏色濾鏡會在你要紅色時對到青色。

## Known limitations
## 已知限制

- **No distance measurement.** From Milestone 5 the stopping rule uses the ratio
  of bounding-box area to frame area. That is an image-space proxy for "the
  object looks closer", not metric distance. A large mug and a small mug stop at
  very different real distances.
  **沒有距離量測。** 從里程碑 5 開始，停止規則用的是「框面積 / 畫面面積」。那是影像空間裡「看起來比較近」的代理指標，不是公尺距離。大馬克杯和小紙杯會在完全不同的實際距離停下來。
- **Detection is per-frame, with no tracking.** There is no identity carried
  between frames, so two similar objects can swap which one is "the target"
  from one frame to the next. The selection rule is deterministic for a given
  frame, which bounds the problem but does not solve it. If it becomes a
  problem in Milestone 4, the fix is a tracker (ByteTrack), not a tweak to the
  selector.
  **偵測是逐幀的，沒有追蹤。** 影格之間沒有身分延續，兩個相似物件可能這一幀與下一幀互換「誰是目標」。選擇規則對單幀是確定的，這能限制問題，但不能解決它。若里程碑 4 變成問題，解法是追蹤器（ByteTrack），不是微調選擇器。
- **The colour filter is a heuristic.** Fixed HSV thresholds do not adapt to
  lighting. Treat `COLOUR_MIN_RATIO` as something to be tuned per room and
  recorded with the experiment, not as a constant.
  **顏色濾鏡是啟發式規則。** 固定 HSV 門檻不會隨光線調整。把 `COLOUR_MIN_RATIO` 當成每個房間都要調、並隨實驗記錄的值，而不是常數。
- **No obstacle avoidance.** The Tello has no forward rangefinder. Walls, table
  legs and people are invisible to this system. Always supervise the flight.
  **沒有避障。** Tello 沒有前方測距。牆壁、桌腳與人都是這個系統看不見的。飛行時必須有人監看。
- **Indoor drift.** Tello position hold relies on the downward camera. On plain,
  dark or reflective floors, translation commands fail with `error Run timeout`.
  **室內漂移。** Tello 的位置保持依賴下視相機。在單色、過暗或反光地板上，平移指令會失敗並回 `error Run timeout`。
- **EEG parameters are unset.** Sampling rate, channel names, filter bands,
  epoch windows and the classifier are all `None` in `config.py` and stay that
  way until the real amplifier and paradigm are known. Inventing them would
  produce a plausible-looking but meaningless pipeline.
  **EEG 參數尚未設定。** 取樣率、通道名稱、濾波頻帶、epoch 視窗與分類器在 `config.py` 都是 `None`，在真實放大器與典範確認前維持如此。亂填會做出看起來合理、卻沒有科學意義的管線。

## Safety checklist before the first real flight
## 第一次真飛前的安全檢查

1. Battery above 50 percent, LED not red.
   電量高於 50%，LED 不是紅燈。
2. Open space, textured floor, nothing fragile within a few metres.
   開放空間、有紋理的地板，幾公尺內沒有易碎物。
3. Keep `DRY_RUN = True` until the perception and navigation logic have been
   verified on recorded or live video without flying.
   在感知與導航邏輯已用錄影或即時影像（不飛）驗證之前，維持 `DRY_RUN = True`。
4. Have the official Tello app open on a phone as a manual override.
   手機開著官方 Tello App，作為手動接管。
