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
