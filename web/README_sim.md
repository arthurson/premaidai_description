# PreMaid AI – MuJoCo WASM 模擬器（技術說明）

使用方法同介面導覽請睇根目錄 [README](../README.md)。呢度記錄實作細節。

## 檔案

* `premaid-sim.src.html`：原始碼（HTML／CSS／JS，資料位置係 `/*__ASSETS__*/`）。
* `premaid-sim.html`：由 `mujoco/make_premaid_mjcf.py` 生成 ＝ src ＋ MJCF ＋ 彩色 mesh（`window.PREMAID`）。**唔好直接改。**
* `../pma/library.js`：動作庫，由 `pma/make_index.py` 生成嘅普通 script（`window.PMA_LIBRARY = [{title, pma, audio?, drm?, data}]`，`data` ＝ base64 嘅 .pma），用 `<script src>` 載入，所以 `file://` 都讀到（`fetch()` 喺 `file://` 會被瀏覽器擋）。音樂唔內嵌，只記檔名，由頁面用相對路徑載入。
* `vendor/`：`three.min.js`、`OrbitControls.js`、`mujoco/mujoco.{js,wasm}`，全部相對路徑載入。

## 模型

3 頭 ＋ 5＋5 手 ＋ 6＋6 腳 ＝ 25 粒位置舵機（URDF 全部 revolute joint）。舵機扭力上限係 URDF 嘅 `effort`（腳／肩前後約 1.37 N·m，其餘約 0.91 N·m）；右欄每粒舵機下面嘅幼線係「扭力 ÷ 上限」，≥ 98% 變橙色。物理參數 μ、kp 可喺 ⚙ 調。

## .pma 回放

* 封包 `[LEN][CMD]…[XOR]`；`0x18` ＝ raw ICS 位置（`0` ＝ 脫力 → 舵機剛度 0），`payload[1]` ＝ SPD；`0x19` 速度／stretch 封包略過。
* Frame 時間 ＝ `max(SPD × 15 ms, 最大行程 / 12.7 pulse·ms⁻¹)`；`平滑` 由上一幀線性補間到當前幀，`逐幀跳` 喺 frame 開始時直接送目標。
* 角度 `θ = sign × (raw − 7500) × 270/8000°`；行程 ＝ 韌體限位平移 `home − baseline`（`PMA_CAL`）。符號用 10 個舊有 `.pma`（3089 frames）驗證；髖旋轉、頭、上臂／手腕旋轉為「推定」。
* 開環播放，冇平衡控制；跳舞建議開「吊起機身」。
* 檔案編碼：UTF-8 同 Shift-JIS 都接受。

## 音樂同步

* 音樂用 `<audio>`，時間軸跟 `P.t`（動作時鐘）：`want = P.t/1000 + offset`。
* 只喺「動作播放中 ＋ 物理冇凍結」先出聲；`playbackRate` ＝ 模擬速度；`|currentTime − want| > 0.25 s` 就重新定位（涵蓋循環、拖進度條、卡頓）。
* 音檔來源：`library.js` 嘅 `audio`、手動 🎵 載入、或者連同 `.pma` 拖放（用 object URL，唔會上傳）。
* `drm` 項（`.wsdcf`）只用嚟顯示提示，唔會嘗試解密。

## 關鍵幀錄製

`rcRaw()` 將目前 25 粒舵機嘅目標角轉成 raw（`7500 + sign × θ / K_DEG`，夾喺 PMA 範圍內，唔受「安全限位」影響）；`rcBytes()` 用 `poseBytes()` 為每個關鍵幀生成 `0x18` 封包，前後加 `RC_PRE`、`RC_STRETCH`、`RC_POST`（見原始碼）；`rcText()` 輸出 `.pma` 文字。預覽經 `loadSections()` 載入播放器。驗證：用頁面本身嘅 `decodePma`／`buildTimeline` 重新讀回匯出檔，位元組完全一致、每幀 raw 同 SPD 無損、checksum 全部通過、4 個非 `0x18` 封包被略過。

### 推擊測試、手制

* 推擊：`S.d.xfrc_applied`（WASM 有開放）嘅 `base_link` 前 3 個分量，力向量由 `base_link` 姿態換算（前 ＝ 機身 +x、左 ＝ +y），持續 0.1 秒模擬時間後清零。邏輯測試：機身轉 90° 後「前推」變成世界 +y、「左推」變成世界 −x、0.1 秒後清零。物理參考（MuJoCo 3 伺服器版，零位站立，0.1 秒）：前 10 N、後 8 N、左右 10 N 起跌。
* 手制：Gamepad API 輪詢（每幀），`axes[0..1]` ＝ 左搖桿（轉向、前後，Y 軸反向）、`axes[2..3]` ＝ 右搖桿（腰轉、前傾／蹲），死區 0.15，A 鍵（`buttons[0]`）邊緣觸發開始／停止；數值同畫面搖桿、鍵盤相加後夾喺 ±1。

### 動作檢查

`chkStats(tl)` 對 `buildTimeline()` 嘅結果逐幀逐舵機統計：用到嘅 raw 最小／最大、超出 `rawLo/rawHi`（機械）次數、超出 `SAFE`（遙控）次數、最大角速度（`|Δraw|·K_DEG / D`，D 係該幀時間）；`renderCheck(prefix, tl)` 畫摘要同 25 條柱。驗證：10 套動作各自嘅「超出遙控安全範圍舵機數」（18、18、17、16、12、18、18、17、17、15）同獨立 Python 計算一致；故意放一幀超出機械行程嘅動作，會被標出。速度上限唔做警告：官方動作對 11 粒細扭力舵機都達 429°/秒（高過 URDF 嘅 333°/秒），顯示為資訊而唔係錯誤。

### 鏡像、匯入、變速

* `A.mirror`（`mirror_table()` 輸出）：`{k, p, m}` ＝ 舵機 k 取舵機 p 嘅關節角乘 m。m 由零位（左右對稱）時嘅關節軸計出：鏡像會反轉旋轉方向，所以 `axis_左 ＝ M·axis_右` 時 m ＝ −1，`axis_左 ＝ −M·axis_右` 時 m ＝ +1（M ＝ diag(1,−1,1)）；頭部舵機以自己做對稱對象（俯仰 +1、轉向 −1、側傾 −1）。`mirrorRaw()` 喺 raw 值層面套用（`raw' ＝ 7500 + sign_k·sign_p·m·(raw_p − 7500)`，夾返 PMA 範圍；raw 0＝脫力保持脫力）。
* 驗證（MuJoCo 正向運動學，隨機關節角）：鏡像後左邊每個 body 嘅位置同右邊原來位置嘅鏡面反射一致（3300 次比較，誤差 ≤ 數值精度；如果全部 m ＝ +1，誤差 0.36 m）。最後一粒關節（手腕旋轉、踝側擺）用離軸嘅尖端點驗證：誤差等於左右 mesh 本身嘅不對稱（手 0.2 mm、腳 2 mm），符號故意寫錯就係 5 mm／36 mm。`mirrorRaw()` 套用兩次回復原狀（600 組肢體比較、最大尖端誤差 2.6 mm ＝ 腳 mesh 不對稱）。
* 匯入：`P.frames` → 關鍵幀（free 嘅舵機記做 raw 0）；「每 N 幀留 1」配合「SPD 補時間」（SPD × N）。變速：`spd ＝ round(spd / 倍數)`。

## IK 拖曳

* 運動學資料由 `mujoco/make_premaid_mjcf.py` 嘅 `kin_chains()` 輸出到 `window.PREMAID.chains`（4 條肢體：`r_leg`、`l_leg`、`r_arm`、`l_arm`，由 `base_link` 起，每個關節有 body 偏移、旋轉、軸，加上尖端點：手＝mesh 中心，腳＝腳底中心）。
* `ikFK(chain, q)` 喺 `base_link` 座標系計尖端位置同朝向；`ikSolve(chain, q0, 目標, 朝向?)` 用數值 Jacobian 嘅阻尼最小二乘法（λ² ＝ 9e-4，每步最多 2 cm），每次迭代夾返關節限位。目標由世界座標用 `base_link` 實時姿態換算，所以機身郁動時目標跟住走。
* 驗證（離線）：FK 同 MuJoCo 比較，位置誤差 ≤ 4×10⁻⁷ m、旋轉誤差 ≈ 10⁻¹⁵（隨機機身姿態、隨機關節角）。模擬拖曳測試（每條肢體 60 組）：細幅度拖曳全部 <1 mm 到達；手臂連續拖曳 60/60 到達，平均終點誤差 0.2 mm；腿部連續拖曳 40–45/60（餘下係「目標位置 ＋ 保持腳掌朝向」唔可達嘅組合，會停喺最接近位置）；每次求解 <0.2 ms。

## 步行發生器

`WALK.make().step({go, X1, Y1, X2, Y2, len, roll, pitch})` 每次回傳 17 個 raw 值（`WALK.IDS` 順序，中心 7500）。`cond` 狀態：0 待機、1 起步半步、2 步行、3 收步半步。模擬入面每 `SPD × 15 ms` 生成一個 pose，舵機目標喺兩個 pose 之間線性補間；送去真機時每個 pose 發一個直接 pose 封包，SPD ＝ 步頻 tick。來源：kazzlog/pm_sample（Apache-2.0）。

## 真機通訊

Web Serial，115200 8N1。直接 pose 封包 `LEN 18 00 SPD {pid lo hi}×25 XOR`；實際 raw 值 ＝ `7500 + sign × θ / (270/8000)`，再 clamp 到「PMA 範圍 ∩ `PreMaidServo.cs` 安全範圍」。其餘封包見根目錄 README。

## 主控台 API

`PMSIM.setTarget(k, deg)`、`PMSIM.applyPose({r_kny_joint: 40})`、`PMSIM.reset()`。

## 真機：digital twin、示教、WebSocket 橋接、監視（Robot 分頁）

* **舵機狀態（0x01）**：每粒舵機發 `07 01 00 05 ID 0E`，回覆 `[18][01][00]` ＋ 14 byte（ID、有效旗標 0x80、編碼器值、trim、指令值、陀螺旗標、陀螺倍率、speed、stretch）＋ XOR。同時最多 3 個未回覆，逾時 200 ms（或 4 倍實測延遲）；無效／冇回覆嘅舵機每 8 輪先再試。表格顯示實際／角度／指令（`free` ＝ 脫力）／speed·stretch／gyro，並顯示延遲同一輪時間。**回覆格式只對 premaiduino2 韌體驗證過，未用原廠機驗證；premaiduino2 冇獨立編碼器（實際 ＝ 指令）。**
* **Twin**：實際值轉角度（`θ = sign × (raw − 7500) × 270/8000°`）直接設做模擬器目標。同「即時控制」「串流」互斥。
* **示教**（Record 分頁）：「開始示教」開 Twin＋輪詢並脫力（揀「左→右」只脫力左邊）→ 手動擺 pose → 「記錄一幀」或「自動記錄」（每 N ms 一幀，SPD ＝ N/15）→ 「鎖住現姿勢」。記錄入關鍵幀列表，可預覽、匯出 `.pma` 或 CSV（frame、time_ms、spd、每粒舵機 raw 同角度）。
* **左右跟隨**：模擬器內拉 slider，對側按 `A.mirror`（`角度_k ＝ m × 角度_p`）跟住郁；真機揀「左→右／右→左」＋ Twin，示教邊脫力，對側以部分 `0x18` 封包（只帶對側舵機，SPD 2，變化 ≥ 6 raw 先送，70 ms 節流）驅動到鏡像位置。
* **WebSocket 橋接**：Robot 分頁嘅 `WS` 行。`python tools/ws_bridge.py /dev/ttyUSB0 --host 0.0.0.0`（`pip install aiohttp pyserial`）同時 serve 本頁（`http://<pi>:8765/web/premaid-sim.html`）同 `/ws`；二進位 frame 原樣雙向轉發。冇驗證，預設只聽 127.0.0.1。https 頁面只可以連 `wss://` 或 `ws://localhost`（瀏覽器限制），所以用 http serve。Safari／iOS 冇 Web Serial，要用呢個。
* **序列監視**：TX／RX hex ＋解碼（CMD 名、電壓、舵機回覆）、可隱藏輪詢、暫停、複製、手動送 hex（自動加 LEN＋XOR；Flash 指令要確認）。電壓每 10 秒讀一次，畫成曲線（紅虛線 ＝ 9 V）。連接時用 Screen Wake Lock 保持亮屏。
* **自動重連**：序列口拔走／WebSocket 斷線後，只要冇撳「中斷連線」就自動重試（序列口按 USB vendor／product 對返同一個裝置，退避 1→5 秒）。
* **LED（0x1E）**：只係探針。premaiduino2 入面 0x1E ＝ SaveMotionData（擦寫 Flash），LED 封包格式未知，所以要先剔「我明白」，每次送出都要確認，回覆喺序列監視睇。**唔好喺 premaiduino2 韌體上亂試。**
* **常駐 E-stop**：畫面底部紅掣（同 `Esc`）：停止回放／步行／即時控制／示教錄製，並送 `04 05 00 01` 同 `05 1F 08 01`（各重送 2 次）。唔會脫力（脫力會令機身跌倒）。
* 測試：`python -m unittest discover -s tests`（包括 bridge ↔ 假機身）；`python tests/e2e_p1.py` 用 Chromium → WebSocket → bridge → 假機身跑晒以上功能（需要 playwright；Web Serial 本身 headless 測唔到）。
* **常駐 E-stop** 而家喺舞台左邊中間（唔再壓住進度條）。內建動作係 20 粒掣，一按即送 `05 1F 00 ID`；動作庫清單只顯示名稱。

## 創作工具／模擬器（P2）

* **多語言＋快捷鍵**：介面有 中文／English／日本語（按鈕循環；日文未翻譯嘅字會用英文）。按 `?` 睇快捷鍵：Space 播放、←→ 跳 1 秒（Shift 5 秒）、`,` `.` 逐幀、`[` `]` `\` A-B 循環、N／P 上下首、R 重置、H 吊起、1–6 視角、I IK、C 重心、V 感測 HUD、T 重心軌跡、B 站穩、K 遙控、L 循環、M 音樂、S 分享、X 錄影、G GIF、F 全螢幕、Esc 緊急停止。步行模式開咗，W A S D／方向鍵留畀步行。
* **IK 拖曳**（原有）：開 `🎯 IK` 後拖手腳／頭嘅圓點。
* **動作編輯**（錄製分頁）：點 `#n` 選幀；＋插入、複製、裁前、裁後、拆開（後半入暫存）、合併；洋蔥皮（前一幀藍、後一幀橙，用第二個 MjData 做 FK）；匯出 `.pma`／CSV。播放列 `A` `B` `✕` 設 A-B 循環（`[` `]` `\`），進度條有色帶。
* **遙控**：設定→遙控模式（或 `K`）。手制：左搖桿頭、右搖桿手臂、LT／RT 手肘、LB／RB 換手、Y 站穩、X 播放；鍵盤 W A S D 頭、I J K L 手臂、U O 手肘。
* **分享／嵌入／錄影**：`🔗 分享連結` 複製 `#m=動作名&t=毫秒&cam=相機&play=0&speed=&loop=0&hang=1`；加 `&kiosk=1` 只剩舞台，`&embed=1`（隱藏所有面板同緊急停止）、`&bar=0`（隱藏進度條）；`</> 嵌入碼` 複製 iframe。`⏺ 錄影 WebM` 用 MediaRecorder 錄 canvas＋音樂（預設由頭播一次，完咗自動停）；`GIF` 錄 1–10 秒（10 fps、320 px，自家 GIF 編碼器、3-3-2 調色盤）。
* **PWA／離線／播放清單**：`manifest.webmanifest`＋`sw.js`（stale-while-revalidate，預先快取 html、three、MuJoCo wasm、動作庫），載入過一次之後可離線用、可安裝；只喺 http(s) 生效。動作庫有 ⏮ ⏭、自動下一首、隨機、播放時長亮（Wake Lock）、全螢幕。
* **感測／除錯 HUD**（設定）：IMU（由機身姿態同速度推算：roll／pitch／yaw、gyro、加速度計）、左右腳底法向力（由 MuJoCo 接觸約束力 `efc_force` 加總；靜止時兩腳加起嚟 ≈ 機重）、力矩最大 3 粒（佔力矩上限 %）、重心 x／y／z、重心地面軌跡（橙線）。冇改 MJCF，所以冇加 `<sensor>` 元素；數值係由狀態算出嚟。
* **站穩輔助**（設定）：腳踝 pitch／roll PD（IMU）＋重心側向偏差調髖／踝 roll。實測（呢個模擬器，0.1 s 推）：側推倒下限 9 N → 9.5 N，前推 9 N → 10 N，**提升好細（+0.5～1 N）**；單腳站立未解決。
* **premaidai-gym 嘅 `SimpleWalkController` 冇移植**：我喺呢個 MJCF 上試過（原版加 80 組正負號／幅度變體），全部 2.7 秒內跌倒，行唔到，所以冇放入。現有步行用 pm_sample 嘅查表步態（已喺真機驗證過）。
* **鏡頭動作捕捉（手臂）**：MediaPipe Pose 世界座標 → 人體座標系（前／左／上）→ 手腕相對肩膀嘅位置按手臂長度縮放（88 % 臂展）→ 手臂 IK。**執行期（鏡頭、由 CDN 載入模型）喺開發環境冇辦法實測**，只測咗座標→關節嘅數學（尺寸無關、方向正確、IK 誤差 < 1 mm）；只做兩隻手臂，冇做頭同腳。**WebXR／VMC 冇做**：VMC 係 UDP／OSC，瀏覽器收唔到，要另外加橋接；WebXR 要 VR 頭戴裝置先測到。
* 測試：`python tests/e2e_p2.py`（playwright＋Pillow；包括 GIF 用 Pillow 解碼、WebM EBML 檢查、離線重載、分享連結重開、站穩推力測試）。
