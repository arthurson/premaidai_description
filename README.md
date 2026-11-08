# PreMaid AI · premaidai_description

PreMaid AI（プリメイドAI，DMM 桌面機械人偶像）嘅模型同工具集：**ROS 描述檔**、**瀏覽器物理模擬器**（MuJoCo WASM，25 粒舵機）、**.pma 動作＋音樂同步播放**，同埋**經 Web Serial 控制真機**。

> English in one line: URDF/xacro model of the PreMaid AI robot plus a single-file browser simulator (MuJoCo WASM) that replays `.pma` dances in sync with music and can drive the real robot over Bluetooth serial.

**線上試玩：<https://arthurson.github.io/premaidai_description/web/premaid-sim.html>**

---

## 功能

| | |
|---|---|
| 物理模擬 | three.js 畫面 ＋ MuJoCo 3.17 WASM，3 頭 ＋ 5＋5 手 ＋ 6＋6 腳 ＝ 25 粒位置舵機，有重力、摩擦、扭力上限，失衡會跌 |
| 動作庫 | 讀取 `pma/` 入面嘅 `.pma`，一撳就播；亦可以自己載入或者拖放 |
| 音樂同步 | 動作同音樂一齊播、一齊停、一齊跳時間、跟速度；可微調偏移 |
| 推擊測試 | 右欄「設定」分頁可以由前後左右推機身（0.1 秒外力，1–20 N），測試動作或者步行嘅穩定度 |
| 動作檢查 | 每套動作用到每粒舵機幾大角度、有冇超出機械行程／遙控安全範圍、最快速度；載入動作或者錄製時自動顯示 |
| 錄製 | 擺好姿勢（slider、預設、IK、步行都得）逐個加入關鍵幀；或者匯入現有動作，鏡像、變速、逐幀編輯，再匯出成 `.pma` |
| IK 拖曳 | 拖動手／腳上面嘅圓點，關節自動求解（阻尼最小二乘法）；腳會保持原本朝向 |
| 步行 | 正弦表步態（來自 @kazzlog 嘅 pm_sample）：搖桿控制前進、後退、轉向、腰轉、前傾／蹲低，可同時送去真機 |
| 真機控制 | Web Serial（藍牙 SPP）：同步 slider、串流 PMA、脫力、T 待機、電量、stretch／speed |
| 手機／平板 | 左右面板變抽屜，觸控友善 |
| ROS | `robots/`、`urdf/`、`launch/` 保留原有 URDF／xacro，可用 RViz 睇 |

## 快速開始（瀏覽器）

**線上**：直接開上面條連結。

**本機**：

```bash
git clone https://github.com/arthurson/premaidai_description
cd premaidai_description
python -m http.server            # 然後開 http://localhost:8000/web/premaid-sim.html
```

**離線 `file://`**：直接雙擊開 `web/premaid-sim.html`。動作庫經 `pma/library.js`（普通 `<script>`，唔使 fetch）載入，所以離線都睇到全部動作，音樂亦用相對路徑載入；MuJoCo 物理引擎嘅 `.wasm` 瀏覽器唔俾 `file://` 自動讀，頁面會提示你揀 `web/vendor/mujoco/mujoco.js` 同 `mujoco.wasm`（一次過揀兩個）。想唔使揀就用上面嘅本機伺服器。

**自己架 GitHub Pages**：Settings → Pages → Deploy from a branch → `master` / `/ (root)`。全部資源用相對路徑，唔使 build；`.nojekyll` 防止 Jekyll 處理。

## 介面導覽

| 位置 | 內容 |
|---|---|
| 頂部中間 | 視角（前後左右頂斜）、**↺ 重置機身**、**吊起機身**（用 weld 吊喺空中，睇舵機動作／跳舞建議開）、**🎯 IK**、**⚙**（跳去右欄「設定」分頁；唔會喺畫面中間彈出，所以唔會擋住機械人） |
| 右欄 · 設定 | 重力、顯示重心、**❄ 凍結物理**、摩擦 μ、剛度 kp、播放方式（平滑／逐幀跳）、**推擊測試**、模擬狀態 |
| 左欄 · 動作 | 載入 .pma、動作清單、音樂（載入、音量）、進階（音樂偏移、精簡 PMA 匯出、舵機對照表） |
| 左欄 · 實機 | 連接真機、控制模式、速度、送出姿勢、脫力、進階指令 |
| 左欄 · 錄製 | 動作名稱、**● 加入關鍵幀**、預設 SPD、關鍵幀清單（套用、改 SPD、用目前姿勢覆蓋、上下移、刪除）、預覽、匯出 .pma |
| 左欄 · 步行 | 開始／收步、兩個搖桿（↑↓ 前後、←→ 轉向；腰轉、前傾／蹲低）、步幅／搖擺／前後擺／步頻、同步送去真機。鍵盤：方向鍵 ＝ 左搖桿，WASD ＝ 右搖桿 |
| 右欄 | 預設姿勢、25 粒舵機 slider（粗線 ＝ 扭力負載，橙色 ＝ 飽和）、工具（掃描、複製／貼上姿勢 JSON） |
| 底部 | ▶ 播放／⏸ 暫停、■ 停止、進度條、🎵 音樂開關、🔁 循環、速度 |

> **兩個「停」唔同**：底部 ⏸ 只暫停動作播放（物理照行，公仔仍受重力）；右欄「設定」入面嘅 ❄ 凍結物理會停晒成個模擬（亦會暫停音樂）。

## 動作庫同音樂

```text
pma/
  Embrace Blade.pma          ← 動作
  Embrace Blade.mp3          ← 同名音樂（mp3 / m4a / aac / ogg / opus / wav / flac）
  library.js                 ← 動作庫（由 make_index.py 生成：清單 ＋ 全部 .pma，網頁讀呢個；http／file:// 都得）
  make_index.py
```

1. 將 `.pma` 放入 `pma/`。有音樂就放**同名**嘅音檔喺隔離。
2. 執行 `python pma/make_index.py`，重新生成 `library.js`（約 1 MB，內含所有 .pma；音樂唔內嵌，只記檔名），commit。
3. 開網頁，喺左欄「動作」撳條目：動作同音樂一齊播。

**音樂同步**：音樂跟動作時鐘走——動作播放先出聲；暫停、停止、凍結物理就停；拖進度條音樂即時跳；循環時由頭再播；速度（¼×–2×）會同步改音樂速度；偏差超過 0.25 秒會自動校正。唔準嘅話用「進階 → 音樂同步偏移」微調。

**自己嘅檔案**：📂 載入 .pma，或者將 `.pma`（可連同音檔）拖到畫面；音檔亦可以單獨用「🎵 載入音樂」加。

**關於 `.wsdcf`**：同名嘅 `.wsdcf` 係加密容器（`video/mp4`、AES-128-CBC、要向 `api.webstream.ne.jp` 取版權金鑰），瀏覽器播唔到，本專案亦唔包含解密。需要聲音嘅話請自行提供一般音檔（例如自己合法擁有嘅 mp3／m4a）。`.wsdcf` 唔使放入 repo（已加入 `.gitignore`）；如果仍然放喺 `pma/`，`make_index.py` 會標記做「加密」，介面會顯示提示。

## 動作檢查

載入動作（左欄「動作」）或者編輯關鍵幀（「錄製」）時，「動作檢查」會顯示：幀數同時間、有幾粒舵機超出**機械行程**（PMA 範圍，紅色＝可能撞到機械極限）、超出**遙控安全範圍**（`PreMaidServo.cs` 嘅保守限位，官方動作普遍會超出，只作參考）、最快速度；展開後每粒舵機有一條柱：灰＝機械行程、淺紫＝遙控安全範圍、深色＝實際用到嘅範圍。

用你 10 套官方動作檢查時發現肘部側擺同右手腕旋轉嘅標稱範圍窄咗 3–4°（官方動作會用到更大），已經按實際用到嘅極值放寬（`mujoco/make_premaid_mjcf.py` 嘅 `PMA_CAL` 有註解），之後官方動作冇再超出機械行程。

## 錄製動作（匯出 .pma）

左欄「錄製」分頁：

1. 用 slider、預設姿勢、**🎯 IK** 或者步行擺出一個姿勢，撳 **● 加入關鍵幀**。重複幾次。
2. 每個關鍵幀有自己嘅 SPD（× 15 ms ＝ 到達呢個姿勢嘅時間，數字越細越快）；可以套用呢個姿勢、用目前姿勢覆蓋、上下移、刪除。
3. **▶ 預覽**會將動作載入左邊播放器（可以加音樂，亦可以用「播放 PMA」模式串流去真機）；**⬇ 匯出 .pma** 下載檔案，放入 `pma/` 再跑 `python pma/make_index.py` 就會出現喺動作庫。

**編輯現有動作**：喺左邊「動作」分頁載入任何 `.pma`，去「錄製」撳 **⤓ 匯入播放器動作**，就變成可編輯嘅關鍵幀（可揀「每 N 幀留 1」，預設會按比例放大 SPD 保持時間）。之後可以用：

* **↔ 左右鏡像**：左右手腳對調，頭轉向／側傾反號（由模型關節軸自動計出，已用正向運動學驗證）；
* **變速 ×**：例如 `2` ＝ 快一倍（所有 SPD 減半）；
* 逐幀刪除、上下移、改 SPD、用目前姿勢覆蓋。

關鍵幀儲存喺瀏覽器本機（`localStorage`），重新開頁面仍然喺度。匯出檔格式同內置動作一致：UTF-8（BOM）、LF、`[モーション(1)]` / `番号=1` / `名前=` / `データ=`，內容係 `07 02 00 41 00 01 45`、stretch 設定（抄自 Embrace Blade）、每個關鍵幀一個 25 粒舵機嘅 `0x18` 封包、`07 02 00 41 00 0A 4E`、`04 17 00 13`、`FF FF`。模擬器回放已驗證；**真機同官方 App 是否接受未驗證**。

## IK 拖曳

頂部撳 **🎯 IK**，雙手（青色）同雙腳（綠色）會出現圓點；用滑鼠或手指拖動圓點，對應肢體嘅關節自動求解，右欄 slider 同實機（「同步 slider」模式）一齊跟住郁。

* 手：3 個自由度位置求解（5 粒舵機）；腳：位置 ＋ 保持腳掌朝向（6 粒舵機）。目標去到關節限位或者伸唔到，就停喺最接近嘅位置。
* 圓點代表「指令入面」嘅手／腳尖位置（由 slider 角度計出），所以機身未站穩時圓點同實際位置可能有少少差。
* 建議配合「吊起機身」使用；站喺地面時提起一隻腳會令重心移位而跌倒。
* 拖曳時會暫停動作播放同步行。

## 步行

左欄「步行」分頁。步態係開環（冇平衡回饋）：10 格正弦表產生 17 粒舵機（腿、肩前後、髖旋轉、頭）嘅 raw 值，每 SPD × 15 ms 一個 pose，pose 之間補間；開始時先設零位 T-pose，停止時會行完呢一步先收步。

* 模擬入面：我用 MuJoCo 喺伺服器跑過，預設參數下前進約 8 cm／秒、唔跌（13.5 秒行咗 110 cm，略向左偏約 34°），後退同原地踏步穩定，轉向搖桿可原地轉彎。
* 真機：剔「同步送去真機」就會用直接 pose 封包（SPD ＝ 步頻 tick）串流。**請先吊起機身**，手放喺電源附近。
* **手制**：接駁藍牙／USB 手制（Xbox、PS 等，需要撳一下任何掣先會被瀏覽器偵測到）：左搖桿 ＝ 步行（↑ 前進、← → 轉向），右搖桿 ＝ 腰轉／前傾，**A 鍵 ＝ 開始／停止**；同方向鍵、WASD、畫面搖桿可以同時用。
* **推擊測試**：右欄「設定」→「推擊測試」，揀方向同力度（方向跟機身朝向）。喺伺服器模擬入面，站立時約 8–10 N（≈ 半個體重）持續 0.1 秒就會推跌；步行或者播放動作時都可以推。
* 步態同常數來自 [kazzlog/pm_sample](https://github.com/kazzlog/pm_sample)（Apache-2.0），本專案重新實作。

## 連接真機（實機分頁）

1. 喺作業系統先配對 PreMaid 嘅藍牙；用 **Chrome／Edge**（桌面或 Android，需要 https 或 localhost）。
2. 左欄 → 實機 → **🔌 連接實機**，揀 PreMaid 嘅 COM 口（115200 8N1）。
3. 揀控制模式：**只睇模擬**／**同步 slider**（最多約 12 包／秒）／**播放 PMA**（每個 frame 開始時送出，機身用該 frame 嘅 SPD 自己補間）。
4. **實機速度**即 SPD，數字越細越快；**安全限位**預設開（PMA 範圍 ∩ 舵機安全範圍），只作用於 slider、IK、步行同「送出目前姿勢」。**串流 PMA 預設唔夾限位**：你 10 套官方動作每套都有 12–18 粒舵機超出呢個保守範圍，夾咗會扭曲動作；想夾可以剔「串流 PMA 都套用安全限位」。

**安全**：第一次請吊起機身、用大 SPD（慢）、手放喺電源附近。標「推定」嘅舵機方向未經驗證，可能令肢體撞入機身。「全部脫力」後再送姿勢要小心。

協議（來自 [PremaindAI_TechVerification](https://github.com/neon-izm/PremaindAI_TechVerification)）：封包 `[LEN][CMD]…[XOR]`，XOR ＝ 前面所有 byte。

| 功能 | 封包 |
|---|---|
| 直接 pose | `LEN 18 00 SPD {pid lo hi}×25 XOR`（`LEN = 5+3n`，raw `0` ＝ 脫力） |
| 電量 | `07 01 00 02 00 02 06`；回覆 `raw = byte3 \| byte4<<8`，電壓 ＝ `raw/216` V |
| stretch／speed | `36 19 10 …`／`36 19 00 …`，每粒 `pid 值` |
| 結束 pose（T 待機） | `04 05 00 01` |
| どうぞ／機內舞蹈 | `05 1F 00 3E 24`／`05 1F 00 01 1B` |
| 回覆 `04 18 08 14` | pose 出錯，網頁會自動重送最後一個 pose（最多 20 次） |

## 重新生成（改過 xacro 或 `premaid-sim.src.html` 之後）

```bash
pip install mujoco numpy scipy trimesh pycollada xacro
python mujoco/make_premaid_mjcf.py --check
```

輸出：`mujoco/premaidai.xml`（單檔 MJCF）同 `web/premaid-sim.html`（＝ `premaid-sim.src.html` ＋ MJCF ＋ 彩色 mesh）。**請改 `.src.html`，唔好直接改生成嘅 `.html`。** 動作庫唔再內嵌喺 html，所以加動作只需要跑 `pma/make_index.py`。

主控台可用：`PMSIM.setTarget(k, deg)`、`PMSIM.applyPose({r_kny_joint: 40})`、`PMSIM.reset()`。

## .pma 格式同角度換算

* 檔案：UTF-8（有 BOM）或 Shift-JIS 文字，`[モーション(N)]` / `番号=` / `名前=` / `データ=<hex>`。
* 封包 `[LEN][CMD]…[XOR]`；`0x18` ＝ 每粒舵機嘅 raw ICS 位置（`0` ＝ 脫力），`payload[1]` ＝ SPD；`0x19` 等其他封包播放時略過。
* Frame 時間 ＝ `max(SPD × 15 ms, 最大行程 / 12.7 pulse·ms⁻¹)`（韌體 `motionPlaybackUpdate()` 嘅規則）。
* 角度：`θ = sign × (raw − 7500) × 270/8000°`；行程 ＝ 韌體限位按 `home − baseline` 平移（見 `mujoco/make_premaid_mjcf.py` 嘅 `PMA_CAL`）。
* 播放係開環（冇平衡控制）。

詳細喺 [`web/README_sim.md`](web/README_sim.md)。

## ROS（原有部分）

環境：Ubuntu 18.04／ROS melodic、Ubuntu 16.04／ROS kinetic。

```bash
mkdir -p ros_ws/src && cd ros_ws && catkin init && cd src
git clone https://github.com/arthurson/premaidai_description
cd .. && catkin build
source devel/setup.bash
roslaunch premaidai_description display.launch
```

TODO：Joint limit 設定、Gazebo 對應、Controller 實作。

## 專案結構

```text
premaidai_description/
├─ index.html              跳去模擬器（GitHub Pages 入口）
├─ web/
│  ├─ premaid-sim.html     單檔模擬器（生成）
│  ├─ premaid-sim.src.html 模擬器原始碼（改呢個）
│  ├─ README_sim.md        模擬器技術說明
│  └─ vendor/              three.js、OrbitControls、MuJoCo WASM
├─ pma/                    動作庫：*.pma ＋ 音樂 ＋ library.js ＋ make_index.py
├─ mujoco/                 premaidai.xml ＋ make_premaid_mjcf.py
├─ robots/ urdf/ meshes/ launch/   ROS 描述檔
└─ package.xml CMakeLists.txt LICENSE
```

## 已知限制

* 標「推定」嘅舵機（髖旋轉、頭、上臂／手腕旋轉）絕對方向無法由資料確定；25 粒舵機嘅方向同 [chikuta/premaidai_controller](https://github.com/chikuta/premaidai_controller) 嘅 `joint_config.yaml` 完全一致（頭轉向、頭側傾兩粒參考佢反轉咗），但仍然冇真機驗證。動作鏡像咗就喺舵機對照表剔「反向」。
* 模擬同真機有差異（摩擦、重心、舵機特性），模擬站得穩唔代表真機一定穩。
* 手機瀏覽器只有部分 Android Chrome 支援 Web Serial；iOS／iPadOS 只可以用模擬同預覽。
* `.wsdcf` 加密音樂唔支援，見上。

## 致謝同授權

* ROS 描述檔源自 [chikuta/premaidai_description](https://github.com/chikuta/premaidai_description)；模型係黑イワシ（[@Schwarz_Sardine](https://twitter.com/Schwarz_Sardine)）[PremaidAI_Model](https://github.com/kuroiwasi/PremaidAI_Model) 嘅分解重組（Apache-2.0，已獲許可）。
* 通訊協議：[neon-izm/PremaindAI_TechVerification](https://github.com/neon-izm/PremaindAI_TechVerification)；步態：[kazzlog/pm_sample](https://github.com/kazzlog/pm_sample)；舵機方向交叉驗證：[chikuta/premaidai_controller](https://github.com/chikuta/premaidai_controller)（MIT）；韌體參數：[arthurson/premaiduino2](https://github.com/arthurson/premaiduino2)；其他參考：[プリメイドAIパーツ軽量](https://drive.google.com/file/d/18qxGj2l_tFbyiChA9kbnJ-sgjDKE4dCX/view)。
* 物理：[MuJoCo](https://github.com/google-deepmind/mujoco)（Apache-2.0）；畫面：[three.js](https://threejs.org/)（MIT）。
* 本專案採用 [Apache License 2.0](LICENSE)。`pma/` 入面嘅動作同音樂版權屬原權利人，只作個人驗證用途。
