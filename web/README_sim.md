# PreMaid AI – MuJoCo WASM 模擬器（技術說明）

使用方法同介面導覽請睇根目錄 [README](../README.md)。呢度記錄實作細節。

## 檔案

* `premaid-sim.src.html`：原始碼（HTML／CSS／JS，資料位置係 `/*__ASSETS__*/`）。
* `premaid-sim.html`：由 `mujoco/make_premaid_mjcf.py` 生成 ＝ src ＋ MJCF ＋ 彩色 mesh（`window.PREMAID`）。**唔好直接改。**
* `../pma/index.json`：動作庫清單，執行時 `fetch('../pma/index.json')`；格式 `[{title, pma, audio?, drm?}]`，由 `pma/make_index.py` 生成。
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
* 音檔來源：`index.json` 嘅 `audio`、手動 🎵 載入、或者連同 `.pma` 拖放（用 object URL，唔會上傳）。
* `drm` 項（`.wsdcf`）只用嚟顯示提示，唔會嘗試解密。

## 真機通訊

Web Serial，115200 8N1。直接 pose 封包 `LEN 18 00 SPD {pid lo hi}×25 XOR`；實際 raw 值 ＝ `7500 + sign × θ / (270/8000)`，再 clamp 到「PMA 範圍 ∩ `PreMaidServo.cs` 安全範圍」。其餘封包見根目錄 README。

## 主控台 API

`PMSIM.setTarget(k, deg)`、`PMSIM.applyPose({r_kny_joint: 40})`、`PMSIM.reset()`。
