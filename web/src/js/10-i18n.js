(function () {
'use strict';
var A = window.PREMAID, D2R = Math.PI / 180, $ = function (id) { return document.getElementById(id); };

// ------------------------------------------------------------------ language (zh-Hant / en)
var I18N = {
  zh: {
    title: 'PreMaid AI · 25 舵機 MuJoCo 物理模擬', intro: '拉 slider = 舵機目標角（度，URDF 關節角）。位置舵機有扭力上限，機身會受重力、摩擦影響，失衡就會跌。',
    view: '視角', vFront: '前', vBack: '後', vLeft: '左', vRight: '右', vTop: '頂', vIso: '斜',
    pause: '⏸ 暫停', resume: '▶ 繼續', tipRun: '暫停／繼續物理', reset: '↺ 重置', tipReset: '將機身擺返企喺地面，由當前舵機目標重新開始',
    pin: '吊起機身', tipPin: '用 weld 約束將機身吊喺空中（base 固定），方便單獨睇舵機動作', grav: '重力', com: '重心',
    tipMu: '腳底／地面滑動摩擦（所有 geom 一齊設）', tipKp: '位置舵機剛度 kp（N·m/rad）；扭力上限仍然係 URDF 嘅 effort', speed: '速度',
    pZero: '零位 T-pose', pDown: '手放低', pFwd: '雙手前伸', pSquat: '半蹲', sweep: '▶ 逐個掃描', sweepStop: '■ 停止掃描', tipSweep: '逐個舵機來回掃一次，睇方向啱唔啱',
    copy: '複製姿勢 JSON', paste: '貼上姿勢', copied: '已複製 ✓', pastePrompt: '貼上姿勢 JSON（{"r_kny_joint": 30, …}，單位：度）', jsonErr: 'JSON 解析失敗：',
    g_head: '頭 Head', g_r_arm: '右手 Right arm', g_l_arm: '左手 Left arm', g_r_leg: '右腳 Right leg', g_l_leg: '左腳 Left leg',
    nServos: function (n) { return n + ' 個舵機'; }, tipZero: '歸零',
    pmaTitle: 'PMA 動作回放', notLoaded: '未載入', loadBtn: '📂 載入 .pma', tipLoad: '載入自己嘅 .pma 動作檔（亦可以拖放到畫面）', tipSec: '檔內有多個動作',
    play: '▶ 播放', stop: '■ 停止', loop: '循環', smooth: '平滑', stepMode: '逐幀跳',
    tipInterp: '平滑：由上一幀線性過渡到呢一幀（SPD×15 ms，唔快過舵機全速）；逐幀跳：直接送目標，由舵機自己追',
    pmaNote: '動作係開環播放（冇平衡控制），跳舞類建議勾選上面「吊起機身」；撳 slider／預設姿勢會暫停回放。原檔 0x02/0x07 loop 會用 stack 展開，0x19 嘅 speed／stretch 會套用到模擬（stretch → 剛度，speed → 最大角速度）。',
    mapSummary: 'PMA 對照表（25 舵機，可逐個反向）',
    mapNote: 'θ = ± × (raw − 7500) × 270/8000°。標「推定」嘅舵機，絕對方向未能由資料確定（左右對稱關係係啱嘅，但成對可能反咗）；睇到動作方向反，就剔「反向」。',
    flip: '反向', inferred: '推定', verified: '已驗證', tipInferred: '絕對方向未能由資料確定', tipVerified: '由 firmware 約定、10 個 .pma 資料同開環站立測試驗證',
    drop: '放開 .pma 檔案載入', frame: '幀',
    nFrames: function (n) { return n + ' 幀'; }, csBad: function (n) { return '⚠ checksum 錯 ' + n + ' 個封包'; }, skipped: function (n) { return '略過 ' + n + ' 個未識別封包'; },
    errNo18: '無 0x18 封包', errFmt: '格式唔啱', errFmtInfo: '搵唔到 [モーション(N)] 段落／データ=',
    sTilt: '傾', sSat: '扭力飽和', sSweep: '掃描', sFell: '⚠ 跌低', sPaused: '暫停',
    loading: '載入 MuJoCo WASM（約 11 MB，首次載入）…', loading2: '載入 MuJoCo WASM …', waitVendor: '等緊 vendor/mujoco/mujoco.js + mujoco.wasm …', failed: 'MuJoCo 載入失敗',
    fileHelp: 'file:// 離線模式：瀏覽器唔俾 fetch，請揀 <code>vendor/mujoco/mujoco.js</code> 同 <code>mujoco.wasm</code> 兩個檔',
    httpHint: '\n（要用 http(s) 開，例如 python -m http.server，然後開 /web/premaid-sim.html）',
    infoLine: function (d) { return d.nb + ' bodies · ' + d.nu + ' position servos · 質量 ' + d.mass.toFixed(3) + ' kg · dt ' + d.dt + ' ms · MuJoCo ' + d.ver; },
    modelSummary: '模型同物理設定',
    model1: 'Mesh 係 cm + Y-up，要套用 .dae 嘅 scene node（×0.01）；套用後 mesh 重心同 URDF <code>&lt;inertial&gt;</code> 吻合。URDF 嘅慣量張量正好大咗 100 倍（例如 r_uarm ixx 3.57e-3，由 mesh 均勻密度算得 3.57e-5），所以生成器用 ×0.01 修正。',
    model2: '碰撞：每個 link 一個凸包；只同地面碰撞，link 之間唔碰撞（相鄰 link 嘅凸包會互相重疊）。舵機：<code>torque = kp·(目標−q) − kv·q̇</code>，上限 = URDF effort（腿同肩 1.3712 N·m，其餘 0.9114 N·m）；joint armature 0.005 kg·m²，implicitfast 積分。',
    model3: 'Slider 單位係 joint 角度，唔係 ICS 原始 position 值；ICS ID／中心值／正負號要由你嘅 firmware 對照表轉換。'
  },
  en: {
    title: 'PreMaid AI · MuJoCo physics simulation · 25 servos', intro: 'Drag a slider = servo target angle (degrees, URDF joint angle). Position servos have a torque limit, and the body is affected by gravity and friction, so it falls when unbalanced.',
    view: 'View', vFront: 'Front', vBack: 'Back', vLeft: 'Left', vRight: 'Right', vTop: 'Top', vIso: '3/4',
    pause: '⏸ Pause', resume: '▶ Resume', tipRun: 'Pause / resume physics', reset: '↺ Reset', tipReset: 'Put the body back on the floor and restart from the current servo targets',
    pin: 'Hang body', tipPin: 'Hang the body in the air with a weld constraint (base fixed) to look at the servos on their own', grav: 'Gravity', com: 'CoM',
    tipMu: 'Sliding friction of feet / floor (set on all geoms)', tipKp: 'Position servo stiffness kp (N·m/rad); the torque limit stays the URDF effort', speed: 'Speed',
    pZero: 'Zero T-pose', pDown: 'Arms down', pFwd: 'Arms forward', pSquat: 'Half squat', sweep: '▶ Sweep servos', sweepStop: '■ Stop sweep', tipSweep: 'Swing each servo back and forth once to check its direction',
    copy: 'Copy pose JSON', paste: 'Paste pose', copied: 'Copied ✓', pastePrompt: 'Paste pose JSON ({"r_kny_joint": 30, …}, degrees)', jsonErr: 'Could not parse JSON: ',
    g_head: 'Head', g_r_arm: 'Right arm', g_l_arm: 'Left arm', g_r_leg: 'Right leg', g_l_leg: 'Left leg',
    nServos: function (n) { return n + ' servos'; }, tipZero: 'Zero',
    pmaTitle: 'PMA motion replay', notLoaded: 'not loaded', loadBtn: '📂 Load .pma', tipLoad: 'Load your own .pma motion file (or drop it onto the view)', tipSec: 'This file holds several motions',
    play: '▶ Play', stop: '■ Stop', loop: 'Loop', smooth: 'Smooth', stepMode: 'Step',
    tipInterp: 'Smooth: interpolate linearly from the previous frame to this one (SPD×15 ms, never faster than servo full speed); Step: send the target at frame start and let the servo chase it',
    pmaNote: 'Playback is open loop (no balance control); for dances tick "Hang body" above. Touching a slider or a preset pauses the replay. The file\'s 0x02/0x07 loops are expanded with a stack, and 0x19 speed/stretch values are applied to the simulation (stretch → stiffness, speed → max angular velocity).',
    mapSummary: 'PMA mapping table (25 servos, flip individually)',
    mapNote: 'θ = ± × (raw − 7500) × 270/8000°. For servos marked "Inferred" the absolute direction could not be derived from the data (the left/right mirror relation is right, but the pair as a whole may be flipped); if a motion looks mirrored, tick "Flip".',
    flip: 'Flip', inferred: 'Inferred', verified: 'Verified', tipInferred: 'Absolute direction could not be derived from the data', tipVerified: 'Verified by the firmware convention, the data of the 10 .pma files and an open-loop standing test',
    drop: 'Drop a .pma file to load', frame: 'frame',
    nFrames: function (n) { return n + ' frames'; }, csBad: function (n) { return '⚠ ' + n + ' packet(s) with bad checksum'; }, skipped: function (n) { return 'skipped ' + n + ' unrecognised packet(s)'; },
    errNo18: 'No 0x18 packets', errFmt: 'Unrecognised format', errFmtInfo: 'No [モーション(N)] section / データ= found',
    sTilt: 'tilt', sSat: 'torque-sat', sSweep: 'sweep', sFell: '⚠ fallen', sPaused: 'paused',
    loading: 'Loading MuJoCo WASM (about 11 MB, first load)…', loading2: 'Loading MuJoCo WASM …', waitVendor: 'Waiting for vendor/mujoco/mujoco.js + mujoco.wasm …', failed: 'MuJoCo failed to load',
    fileHelp: 'file:// offline mode: the browser blocks fetch, so pick <code>vendor/mujoco/mujoco.js</code> and <code>mujoco.wasm</code>',
    httpHint: '\n(open the page over http(s), e.g. python -m http.server, then /web/premaid-sim.html)',
    infoLine: function (d) { return d.nb + ' bodies · ' + d.nu + ' position servos · mass ' + d.mass.toFixed(3) + ' kg · dt ' + d.dt + ' ms · MuJoCo ' + d.ver; },
    modelSummary: 'Model & physics notes',
    model1: 'The meshes are in cm with Y-up, so the .dae scene node (×0.01) must be applied; after that the mesh centre of mass matches the URDF <code>&lt;inertial&gt;</code>. The URDF inertia tensors are exactly 100× too large (e.g. r_uarm ixx 3.57e-3 vs 3.57e-5 from the mesh at uniform density), so the generator scales them by 0.01.',
    model2: 'Collision: one convex hull per link; links collide with the floor only, not with each other (hulls of neighbouring links overlap). Servo: <code>torque = kp·(target − q) − kv·q̇</code>, clipped to the URDF effort (legs and shoulder pitch 1.3712 N·m, others 0.9114 N·m); joint armature 0.005 kg·m², implicitfast integrator.',
    model3: 'Slider units are joint angles, not raw ICS positions; ICS IDs / centre values / signs need your firmware\'s conversion table.'
  }
};
Object.assign(I18N.zh, {
  rbTitle: '實機連線（Web Serial）', rbConn: '🔌 連接', rbDisc: '⏏ 斷開', rbSafe: '安全限位', rbLive: '即時同步 slider', rbStream: 'PMA 串流到實機', rbOnce: '送出目前姿勢', rbRel: '全部脫力',
  rbT: '結束 pose（T 待機）', rbDz: 'どうぞ', rbDance: '播放機內舞蹈', rbApply: '套用', rbBat: '🔋 電量', rbThinA: '每', rbThinB: '幀留 1 · 另存', rbExp: '匯出精簡 .pma',
  rbOff: '未連線', rbOn: '已連線', rbNoSerial: '呢個瀏覽器冇 Web Serial（用 Chrome／Edge，https 或 localhost）', rbErr: '連線失敗：', rbVolt: function (v) { return v.toFixed(2) + ' V'; }, rbLow: '⚠ 低於 9 V', rbPoseErr: '⚠ 實機回報 pose 錯誤',
  rbWarn: '先配對藍牙，揀 PreMaid 嘅 COM 口（115200 8N1）。標「推定」嘅舵機方向未驗證：先吊起機身、用大 SPD（慢）試，手放喺電源附近。安全限位 = PMA 範圍 ∩ TechVerification 嘅 PreMaidServo 限位。全部脫力後再送姿勢要小心。精簡 .pma 只留每 N 個 0x18 幀（首尾必留），時間會變短。',
  polTitle: 'AI 策略（PPO，瀏覽器內推論）', polLoadB: '載入示範策略', polFileB: '載入 .json…', polPoseB: '↺ 由策略姿勢開始', polOnL: '執行策略', polInfo: function (tk, n) { return '已載入：' + tk + '，' + n + ' 層'; }, polErr: function (m) { return '失敗：' + m; }, polNone: '先載入策略',
  polNote: '用 rl/train.py 訓練、rl/export_policy.py 匯出嘅 MLP（50 Hz，觀測＝陀螺、重力方向、關節誤差／速度、上一個動作）。「由策略姿勢開始」會將機身放返訓練時嘅起始姿勢（arms_down）。策略同站穩輔助互斥。'
});
Object.assign(I18N.en, {
  rbTitle: 'Real robot (Web Serial)', rbConn: '🔌 Connect', rbDisc: '⏏ Disconnect', rbSafe: 'Safe limits', rbLive: 'Live-sync sliders', rbStream: 'Stream PMA to robot', rbOnce: 'Send current pose', rbRel: 'Release all',
  rbT: 'End pose (T standby)', rbDz: 'Douzo', rbDance: 'Play built-in dance', rbApply: 'Apply', rbBat: '🔋 Battery', rbThinA: 'keep 1 of', rbThinB: 'frames · save', rbExp: 'Export thinned .pma',
  rbOff: 'disconnected', rbOn: 'connected', rbNoSerial: 'No Web Serial here (use Chrome/Edge over https or localhost)', rbErr: 'Connect failed: ', rbVolt: function (v) { return v.toFixed(2) + ' V'; }, rbLow: '⚠ below 9 V', rbPoseErr: '⚠ robot reported a pose error',
  rbWarn: 'Pair over Bluetooth first, then pick the PreMaid COM port (115200 8N1). Servos marked "Inferred" have unverified direction: hang the body, use a large SPD (slow) and keep a hand near the power switch. Safe limits = PMA range ∩ the PreMaidServo limits of TechVerification. Be careful sending poses right after Release all. Thinned .pma keeps every Nth 0x18 frame (first and last always kept), so timing gets shorter.',
  polTitle: 'AI policy (PPO, runs in the browser)', polLoadB: 'Load demo policy', polFileB: 'Load .json…', polPoseB: '↺ Start from the policy pose', polOnL: 'Run policy', polInfo: function (tk, n) { return 'loaded: ' + tk + ', ' + n + ' layers'; }, polErr: function (m) { return 'failed: ' + m; }, polNone: 'Load a policy first',
  polNote: 'An MLP trained with rl/train.py and exported with rl/export_policy.py (50 Hz; observation = gyro, gravity direction, joint error / velocity, previous action). "Start from the policy pose" puts the robot into the pose it was trained from (arms_down). Exclusive with the stand assist.'
});
Object.assign(I18N.zh, { tabSet: '設定' }); Object.assign(I18N.en, { tabSet: 'Settings' });
Object.assign(I18N.zh, { physSum: '物理參數（μ / kp）', mobL: '動作 / 實機', mobR: '舵機' });
Object.assign(I18N.en, { physSum: 'Physics (μ / kp)', mobL: 'Motion / Robot', mobR: 'Servos' });
Object.assign(I18N.zh, {
  title: 'PreMaid AI', intro: '拉 slider 改舵機目標角（度）。機身受重力同摩擦影響，失衡就會跌。', reset: '↺ 重置機身', com: '顯示重心',
  freeze: '❄ 凍結物理', tipRun: '凍結模擬：機身同動作播放全部停住，只用嚟靜態檢視；再撳一次就繼續', tipLoop: '循環播放整段（原檔 0x02/0x07 loop 會先展開）', tipSpeed: '播放速度', setTip: '模擬設定',
  tabMotion: '動作', tabRobot: '實機', tabServos: '舵機', setPhys: '物理', setPlay: '播放', muL: '摩擦 μ', kpL: '剛度 kp', interpL: '播放方式',
  presetL: '預設姿勢', toolsSum: '工具：掃描 · 複製 · 貼上', advanced: '進階', thinTitle: '精簡 PMA',
  rbConn: '🔌 連接實機', rbDisc: '⏏ 斷開連線', rbModeL: '控制模式', mOff: '只睇模擬', mLive: '同步 slider', mStream: '播放 PMA', rbSpdL: '實機速度', rbSpdTip: 'SPD：數字越細越快',
  rbOnce: '送出目前姿勢', rbT: 'T 待機', rbRel: '⚠ 全部脫力', rbSafe: '安全限位（建議開）', rbBat: '🔋 電量',
  rbWarnS: '首次連線：先吊起機身、用大 SPD（慢）測試，手放喺電源附近。先喺系統配對 PreMaid 藍牙，再揀 COM 口。'
});
Object.assign(I18N.en, {
  title: 'PreMaid AI', intro: 'Drag a slider to set a servo target angle (deg). The body feels gravity and friction and falls if it loses balance.', reset: '↺ Reset body', com: 'Show CoM',
  freeze: '❄ Freeze physics', tipRun: 'Freeze the simulation: body and motion playback both stop, for static inspection. Press again to resume', tipLoop: 'Repeat the whole motion (the file\'s own 0x02/0x07 loops are expanded first)', tipSpeed: 'Playback speed', setTip: 'Simulation settings',
  tabMotion: 'Motion', tabRobot: 'Robot', tabServos: 'Servos', setPhys: 'Physics', setPlay: 'Playback', muL: 'Friction μ', kpL: 'Stiffness kp', interpL: 'Play mode',
  presetL: 'Preset poses', toolsSum: 'Tools: sweep · copy · paste', advanced: 'Advanced', thinTitle: 'Thin PMA',
  rbConn: '🔌 Connect robot', rbDisc: '⏏ Disconnect', rbModeL: 'Control mode', mOff: 'Sim only', mLive: 'Sync sliders', mStream: 'Play PMA', rbSpdL: 'Robot speed', rbSpdTip: 'SPD: smaller = faster',
  rbOnce: 'Send current pose', rbT: 'T standby', rbRel: '⚠ Release all', rbSafe: 'Safe limits (keep on)', rbBat: '🔋 Battery',
  rbWarnS: 'First time: hang the body, use a large SPD (slow) and keep a hand near the power switch. Pair the PreMaid over Bluetooth in your OS first, then pick the COM port.'
});
Object.assign(I18N.zh, {
  drop: '放開 .pma／音樂檔案載入', audTitle: '音樂', audNone: '未有音樂', audLoad: '🎵 載入音樂', audLoadTip: '載入 mp3 / m4a / wav / ogg 等（亦可以連同 .pma 一齊拖放）', audVol: '音量', audOffL: '音樂同步偏移（+ = 音樂提早）', tipAud: '音樂開／關',
  libErr: '搵唔到 pma/library.js。請跑 python pma/make_index.py 生成，或者直接撳「載入 .pma」。', libEmpty: 'pma/ 入面未有 .pma；放檔案後跑 python pma/make_index.py。',
  audDrm: '搵到同名 .wsdcf，但佢係加密格式（AES-128 ＋ 版權伺服器），瀏覽器播唔到。請放同名 .mp3／.m4a 入 pma/ 再跑 make_index.py，或者撳「載入音樂」。', audErr: '音樂檔載入失敗（格式唔支援？）'
});
Object.assign(I18N.en, {
  drop: 'Drop a .pma and/or audio file', audTitle: 'Music', audNone: 'no music', audLoad: '🎵 Load music', audLoadTip: 'Load mp3 / m4a / wav / ogg … (or drop it together with the .pma)', audVol: 'Volume', audOffL: 'Music sync offset (+ = music earlier)', tipAud: 'Music on / off',
  libErr: 'pma/library.js not found. Run python pma/make_index.py to create it, or use "Load .pma".', libEmpty: 'No .pma in pma/ yet; add files and run python pma/make_index.py.',
  audDrm: 'A same-name .wsdcf exists, but it is an encrypted container (AES-128 + rights server) that a browser cannot play. Put a same-name .mp3/.m4a in pma/ and re-run make_index.py, or use "Load music".', audErr: 'Could not load the audio file (unsupported format?)'
});
Object.assign(I18N.zh, {
  loopModeL: '原檔 loop', loopOrig: '跟原檔 loop', loopOnce: '播一次（忽略 loop）', tipLoopMode: '跟原檔 loop：用 stack 展開 0x08 02／0x08 07 loop；播一次：忽略 loop 封包，只播原始幀',
  pmaInfo: function (n, raw, sec, loops, loopsRaw, bpm) {
    return n + ' 幀' + (loops ? '（原檔 ' + raw + ' 幀，' + loops + ' 個 loop' + (loopsRaw ? '／共 ' + loopsRaw + ' 次' : '') + '）' : '') + ' · ' + sec.toFixed(1) + ' 秒' + (bpm ? ' · ~' + bpm + ' BPM' : '');
  },
  pmaParams: function (stretch, speed) {
    var a = stretch ? ('stretch ' + stretch[0] + '–' + stretch[1]) : 'stretch —';
    return '0x19：' + a + (speed ? ' · speed ' + speed[0] + '–' + speed[1] : ' · speed —');
  },
  rbBuiltinL: '內建動作（0x1F）', rbBuiltinTip: '內建動作由機身韌體播放；播完會自動停。若機身正在播放其他動作，先按 STOP。', rbPlayBuiltin: '▶ 播放內建', rbStopMotion: '■ STOP（0x05）',
  rbMotionSent: '已送出內建動作…', rbMotionOk: '✓ 機身已開始播放', rbMotionErr: '⚠ 機身回報播放失敗（可能正在播放其他動作，先按 STOP）', rbMotionStopped: '■ 已送出停止指令',
  rbBuiltinNames: ['首頁', '請', '興奮', '手放腰', '歡迎', '拜託', '再見', '右飛吻', '左飛吻', '手槍', '敬禮', '哭哭', '愛心', '閃亮', '給你', '猴子', 'GO×2', '空氣吉他', '右轉', '左轉'],
  rbBuiltinTransferred: '已傳送動作 1（App 轉送）'
});
Object.assign(I18N.en, {
  loopModeL: 'File loops', loopOrig: 'Follow file loops', loopOnce: 'Play once (ignore loops)', tipLoopMode: 'Follow file loops: expand 0x08 02 / 0x08 07 blocks with a stack; Play once: ignore loop packets and play the source frames only',
  pmaInfo: function (n, raw, sec, loops, loopsRaw, bpm) {
    return n + ' frames' + (loops ? ' (source ' + raw + ', ' + loops + ' loop block' + (loops === 1 ? '' : 's') + (loopsRaw ? ', ' + loopsRaw + ' repeats' : '') + ')' : '') + ' · ' + sec.toFixed(1) + ' s' + (bpm ? ' · ~' + bpm + ' BPM' : '');
  },
  pmaParams: function (stretch, speed) {
    var a = stretch ? ('stretch ' + stretch[0] + '–' + stretch[1]) : 'stretch —';
    return '0x19: ' + a + (speed ? ' · speed ' + speed[0] + '–' + speed[1] : ' · speed —');
  },
  rbBuiltinL: 'Built-in motions (0x1F)', rbBuiltinTip: 'Built-in motions are played by the robot firmware. If another motion is playing, press STOP first.', rbPlayBuiltin: '▶ Play built-in', rbStopMotion: '■ STOP (0x05)',
  rbMotionSent: 'Built-in motion sent…', rbMotionOk: '✓ Robot started playing', rbMotionErr: '⚠ Robot reported playback failure (another motion may be playing; press STOP first)', rbMotionStopped: '■ Stop command sent',
  rbBuiltinNames: ['Home', 'Dozo (please)', 'Wakuwaku (excited)', 'Hands on hips', 'Welcome', 'Onegai (please)', 'Bye-bye', 'Kiss right', 'Kiss left', 'Pistol', 'Salute', 'Cry', 'Heart', 'Kira (sparkle)', 'For you', 'Monkey', 'GO×2', 'Air guitar', 'Turn right', 'Turn left'],
  rbBuiltinTransferred: 'Transferred motion 1 (app upload)'
});
Object.assign(I18N.zh, {
  tabWalk: '步行', walkStart: '▶ 開始步行', walkStop: '■ 收步停止', walkIdle: '待機', walkStarting: '起步', walkWalking: '步行中', walkStopping: '收步',
  padA: '步行：↑ 前進、↓ 後退、← → 轉向', padB: '腰轉／前傾（↑ 前傾、↓ 蹲低）', padHold: '鎖定搖桿位置', walkParams: '步態參數', wLen: '步幅', wRoll: '搖擺', wPit: '前後擺', wSpd: '步頻', wSpdTip: 'SPD（tick，1 tick ＝ 15 ms）：每個 pose 嘅時間，數字越細越快', walkRobot: '同步送去真機（需先連接）',
  walkWarn: '開環步態（正弦表），冇平衡回饋：模擬入面可能跌，真機請吊起機身先試，手放喺電源附近。鍵盤：方向鍵 ＝ 左搖桿，WASD ＝ 右搖桿。',
  walkNote: '步態來自 @kazzlog 嘅 pm_sample（Apache-2.0）：10 格正弦表，每個 pose 之間以 SPD 補間；17 粒舵機受控（腿、肩前後、髖旋轉、頭），其餘舵機保持原位。開始時會先設零位 T-pose。'
});
Object.assign(I18N.en, {
  tabWalk: 'Walk', walkStart: '▶ Start walking', walkStop: '■ Stop (finish step)', walkIdle: 'idle', walkStarting: 'starting', walkWalking: 'walking', walkStopping: 'stopping',
  padA: 'Walk: ↑ forward, ↓ back, ← → turn', padB: 'Waist yaw / lean (↑ lean, ↓ crouch)', padHold: 'Latch stick position', walkParams: 'Gait parameters', wLen: 'Stride', wRoll: 'Sway', wPit: 'Swing', wSpd: 'Cadence', wSpdTip: 'SPD (ticks, 1 tick = 15 ms): time per pose, smaller = faster', walkRobot: 'Also send to the robot (connect first)',
  walkWarn: 'Open-loop gait (sine tables) without balance feedback: it may fall in the simulator; on the real robot hang the body first and keep a hand near the power switch. Keys: arrows = left stick, WASD = right stick.',
  walkNote: 'Gait by @kazzlog (pm_sample, Apache-2.0): a 10-step sine table, poses interpolated by SPD; 17 servos are driven (legs, shoulder pitch, hip yaw, head), the rest keep their position. Starting sets the zero T-pose first.'
});
Object.assign(I18N.zh, { ikChip: '🎯 IK', tipIK: 'IK 拖曳：開啟後拖動手／腳上面嘅圓點，關節自動求解（建議同「吊起機身」一齊用）；腳會保持原本朝向' });
Object.assign(I18N.en, { ikChip: '🎯 IK', tipIK: 'IK drag: drag the dots on the hands/feet and the joints are solved automatically (best with "Hang body"); feet keep their orientation' });
Object.assign(I18N.zh, {
  tabRec: '錄製', recName: '動作名稱', recAdd: '● 加入關鍵幀（目前姿勢）', recSpd: '預設 SPD', recSpdTip: 'SPD（1 ＝ 15 ms）：到達該姿勢嘅時間，數字越細越快', recPlay: '▶ 預覽', recExp: '⬇ 匯出 .pma', recClr: '清空',
  recEmpty: '未有關鍵幀。擺好姿勢（slider、預設姿勢、IK、步行都得）再撳「加入關鍵幀」。', recGo: '套用呢個姿勢', recSet: '用目前姿勢覆蓋', recUp: '上移', recDn: '下移', recDel: '刪除', recSpdRow: '到達呢一幀嘅 SPD',
  recInfo: function (n, s) { return n + ' 個關鍵幀 · 約 ' + s.toFixed(1) + ' 秒'; },
  recNote: '預覽會載入左邊播放器（可以加音樂、串流去真機）。匯出檔跟內置動作嘅 .pma 格式（開頭／結尾封包同 stretch 設定抄自 Embrace Blade）；模擬器回放已驗證，真機／官方 App 是否接受未驗證。資料保存喺瀏覽器本機。'
});
Object.assign(I18N.en, {
  tabRec: 'Record', recName: 'Motion name', recAdd: '● Add keyframe (current pose)', recSpd: 'Default SPD', recSpdTip: 'SPD (1 = 15 ms): time to reach the pose, smaller = faster', recPlay: '▶ Preview', recExp: '⬇ Export .pma', recClr: 'Clear',
  recEmpty: 'No keyframes yet. Set a pose (sliders, presets, IK or walking all work) and press "Add keyframe".', recGo: 'Apply this pose', recSet: 'Overwrite with current pose', recUp: 'Move up', recDn: 'Move down', recDel: 'Delete', recSpdRow: 'SPD to reach this frame',
  recInfo: function (n, s) { return n + ' keyframes · about ' + s.toFixed(1) + ' s'; },
  recNote: 'Preview loads the player on the left (add music, stream to the robot). The exported file follows the .pma layout of the built-in motions (header/footer packets and the stretch setting copied from Embrace Blade); simulator playback is verified, acceptance by the robot / official app is not. Data is kept in this browser.'
});
Object.assign(I18N.zh, {
  recTools: '編輯工具', recMirror: '↔ 左右鏡像', recMirrorTip: '將所有關鍵幀嘅左右手腳對調（頭轉向／側傾反號）', recImport: '⤓ 匯入播放器動作', recImportTip: '將左邊播放器載入咗嘅動作變成關鍵幀，可以逐幀編輯、鏡像、變速、重新匯出',
  recEveryA: '每', recEveryB: '幀留 1', recComp: 'SPD 補時間', recScaleL: '變速 ×', recApply: '套用', recScaleTip: '2 ＝ 快一倍（SPD 減半）', recNoMotion: '播放器未載入動作', recSure: '會取代目前所有關鍵幀，確定？'
});
Object.assign(I18N.en, {
  recTools: 'Edit tools', recMirror: '↔ Mirror left/right', recMirrorTip: 'Swap left and right limbs in every keyframe (head yaw/roll change sign)', recImport: '⤓ Import loaded motion', recImportTip: 'Turn the motion loaded in the player into keyframes you can edit, mirror, re-time and export again',
  recEveryA: 'keep 1 of', recEveryB: 'frames', recComp: 'Scale SPD to keep timing', recScaleL: 'Speed ×', recApply: 'Apply', recScaleTip: '2 = twice as fast (SPD halved)', recNoMotion: 'No motion loaded in the player', recSure: 'This replaces all current keyframes. Continue?'
});
Object.assign(I18N.zh, {
  chkTitle: '動作檢查', chkNone: '未有動作',
  chkSum: function (n, s, a, b, v) { return n + ' 幀 · ' + s.toFixed(1) + ' 秒 · ' + (a ? '⚠ ' + a + ' 粒超出機械範圍' : '機械範圍 OK') + ' · ' + b + ' 粒超出遙控安全範圍 · 最快 ' + Math.round(v) + '°/秒'; },
  chkNote: '條柱：灰 ＝ 舵機機械行程（PMA 範圍），淺紫 ＝ 遙控安全範圍（PreMaidServo.cs 嘅保守限位），深色 ＝ 呢套動作實際用到嘅範圍（紅 ＝ 超出機械行程）。官方動作普遍超出遙控安全範圍，所以「串流 PMA」預設唔夾限位；想夾就喺實機分頁剔「串流 PMA 都套用安全限位」。',
  rbSafe: '安全限位（slider／IK／步行／送出姿勢）', rbSafeStream: '串流 PMA 都套用安全限位（官方動作會被扭曲）'
});
Object.assign(I18N.en, {
  chkTitle: 'Motion check', chkNone: 'no motion',
  chkSum: function (n, s, a, b, v) { return n + ' frames · ' + s.toFixed(1) + ' s · ' + (a ? '⚠ ' + a + ' servos beyond mechanical range' : 'mechanical range OK') + ' · ' + b + ' servos beyond remote-control safe range · fastest ' + Math.round(v) + '°/s'; },
  chkNote: 'Bar: grey = servo mechanical travel (PMA range), light purple = remote-control safe range (conservative limits from PreMaidServo.cs), dark = range this motion actually uses (red = beyond mechanical travel). Official motions usually exceed the remote-control safe range, so "Stream PMA" does not clamp by default; tick "Also apply safe limits when streaming PMA" on the Robot tab to clamp.',
  rbSafe: 'Safe limits (sliders / IK / walk / send pose)', rbSafeStream: 'Also clamp streamed PMA (distorts official motions)'
});
Object.assign(I18N.zh, {
  pushTitle: '推擊測試（0.1 秒外力）', pushFwd: '↑ 前', pushBack: '↓ 後', pushLeft: '← 左', pushRight: '→ 右', pushForce: '力度',
  pushNote: '力施喺機身中心，方向跟機身朝向（模擬測試：站立時約 8–10 N ≈ 半個體重就會推跌）。可以喺播放動作或者步行時推，睇穩唔穩。',
  padGpOff: '🎮 接駁手制（Xbox／PS 等）後撳任何掣：左搖桿 ＝ 步行，右搖桿 ＝ 腰轉／前傾，A 鍵 ＝ 開始／停止。', padGpOn: function (id) { return '🎮 ' + id; }
});
Object.assign(I18N.en, {
  pushTitle: 'Push test (0.1 s force)', pushFwd: '↑ Front', pushBack: '↓ Back', pushLeft: '← Left', pushRight: '→ Right', pushForce: 'Force',
  pushNote: 'The force acts on the torso, direction relative to the body (simulated: about 8–10 N ≈ half the body weight topples a standing robot). Push while a motion plays or while walking to see how stable it is.',
  padGpOff: '🎮 Connect a gamepad (Xbox/PS…) and press any button: left stick = walk, right stick = waist yaw / lean, A = start / stop.', padGpOn: function (id) { return '🎮 ' + id; }
});
Object.assign(I18N.zh, {
  estop: '■ 緊急停止', estopTip: '停止回放／步行／即時控制／示教錄製，並向機身送出 STOP（04 05 00 01 同 05 1F 08 01）。唔會脫力，機身維持現有姿勢。快捷鍵 Esc。',
  estopNoLink: '⚠ 已停止網頁端動作；機身未連接，冇送出 STOP',
  wsConn: '連接', wsTip: 'WebSocket 橋接（Raspberry Pi／ESP32／tools/ws_bridge.py）：Safari／iOS 冇 Web Serial，用呢個。https 頁面只可以連 wss:// 或者 ws://localhost。',
  rbWsTry: '連接 WebSocket 中…', rbLost: '⚠ 連線中斷，自動重連中…', rbWsMixed: '⚠ https 頁面唔可以連 ws://（瀏覽器會封）：用 wss://，或者用 http:// 開呢個頁面（例如由 tools/ws_bridge.py serve）。ws://localhost 例外。',
  stTitle: '舵機狀態（0x01）· digital twin', stPoll: '輪詢舵機狀態', stTwin: 'Twin：真機 → 模擬', stTwinTip: '真機手動郁（要先脫力）→ 模擬同步。同「即時控制」「串流」互斥。',
  mirL: '左右跟隨', mirOff: '關', mirAny: '模擬：郁邊邊，另一邊跟', mirLR: '左 → 右（左邊手動示教）', mirRL: '右 → 左（右邊手動示教）',
  mirTip: '模擬：拉 slider 時對側跟住郁。真機：揀方向 ＋ Twin 開，示教邊脫力、對側會被驅動去鏡像位置（要先撳「開始示教」）。',
  stColJ: '關節', stColAct: '實際', stColCmd: '指令',
  stSum: function (ok, lat, cyc, miss, bad) { return ok + ' 回覆 · ' + lat.toFixed(0) + ' ms · 一輪 ' + (cyc ? (cyc / 1000).toFixed(2) + ' s' : '—') + ' · 逾時 ' + miss + (bad ? ' · 錯誤 ' + bad : ''); },
  stNote: '每粒舵機發一個 07 01 00 05 ID 0E 查詢，同時最多 3 個未回覆，收到回覆先補發下一個（逾時 200 ms，慢速連線會自動加長）；冇接線／無效嘅舵機每 8 輪先再試。實際 ＝ 編碼器值；指令 free ＝ 脫力。注意：premaiduino2 韌體冇獨立編碼器，實際值 ＝ 指令值，真機手動郁要用有回讀嘅韌體（原廠）。回覆格式未用原廠機驗證。',
  teachL: '示教（真機）', tcStart: '● 開始示教（脫力）', tcStartTip: '開 Twin ＋ 輪詢，並將舵機脫力（揀咗「左→右」就只脫力左邊）。', tcHold: '▣ 鎖住現姿勢', tcHoldTip: '將每粒舵機鎖喺佢而家嘅實際位置（停止脫力）',
  tcCap: '● 記錄一幀', tcAuto: '⏺ 自動記錄', tcAutoStop: '■ 停止自動記錄',
  tcCaptured: function (n, live) { return '已記錄第 ' + n + ' 幀（' + live + ' 粒舵機用真機讀數）'; }, tcNoData: '未有有效舵機讀數：先開「輪詢舵機狀態」', tcNeedPoll: '要先連接機身並開「輪詢舵機狀態」', tcHeld: function (n) { return '已鎖住 ' + n + ' 粒舵機'; },
  teachConfirm: '機身會全部脫力：腿會即刻軟晒。請先托住／吊起機身，之後先確定。', teachConfirmSide: '只有示教邊（左或右）會脫力：嗰邊嘅腿會軟晒，請先托住／吊起機身，之後先確定。',
  teachNote: '流程：吊起機身 → 開始示教 → 手動擺 pose → 記錄一幀（或自動記錄）→ 鎖住現姿勢。記錄會加入上面嘅關鍵幀列表，可以預覽、匯出 .pma 或 CSV。',
  recCsv: '⬇ 匯出 CSV', recCsvTip: '每幀一行：frame、time_ms、spd、每粒舵機 raw 同角度',
  mnTitle: '序列監視 · 電量 · Wake Lock', bvL: '電壓（每 10 秒讀一次；紅虛線 ＝ 9 V）', bvTxt: function (v, mn, mx, n, min) { return v.toFixed(2) + ' V · min ' + mn.toFixed(2) + ' · max ' + mx.toFixed(2) + ' · ' + n + ' 點 / ' + min.toFixed(1) + ' 分鐘'; },
  wakeL: '連接時保持螢幕長亮', wakeNo: '呢個瀏覽器冇 Wake Lock', wakeOn: '✓ 長亮中', mnHide: '隱藏狀態輪詢', mnPause: '暫停', mnClear: '清除', mnCopy: '複製', mnSend: '送出', mnAutoTip: '自動加 LEN 同 XOR：只輸入 CMD 同 payload',
  mnCnt: function (tx, rx) { return 'TX ' + tx + ' · RX ' + rx + ' 個封包'; }, mnBad: '請輸入十六進位 byte（例如 01 00 02 00 02）', mnFlash: '呢個 CMD 係 Flash 管理指令（0x04／0x1C／0x1D／0x1E）：喺 premaiduino2 韌體會擦寫動作儲存區。真係要送？',
  ledTitle: 'LED（0x1E，實驗）', ledWarn: '⚠ LED 封包格式未經實機驗證。premaiduino2 韌體入面 0x1E ＝ SaveMotionData（擦寫 Flash 動作頁），亂送會毀動作。呢度淨係俾你用原廠機逐個試 payload，並喺序列監視睇回覆。', ledOk: '我明白，呢度送嘅 0x1E 可能寫入 Flash', ledSend: '送出',
  ledConfirm: function (hex) { return '送出 ' + hex + ' ？'; }
});
Object.assign(I18N.en, {
  estop: '■ E-STOP', estopTip: 'Stops playback / walking / live control / teach capture and sends STOP to the robot (04 05 00 01 and 05 1F 08 01). The servos are NOT released: the robot holds its pose. Shortcut: Esc.',
  estopNoLink: '⚠ Page-side motion stopped; robot not connected, no STOP sent',
  wsConn: 'Connect', wsTip: 'WebSocket bridge (Raspberry Pi / ESP32 / tools/ws_bridge.py): Safari and iOS have no Web Serial, use this. An https page can only reach wss:// or ws://localhost.',
  rbWsTry: 'Connecting WebSocket…', rbLost: '⚠ Link lost, reconnecting…', rbWsMixed: '⚠ An https page cannot open ws:// (the browser blocks it): use wss://, or open this page over http:// (for example served by tools/ws_bridge.py). ws://localhost is the exception.',
  stTitle: 'Servo status (0x01) · digital twin', stPoll: 'Poll servo status', stTwin: 'Twin: robot → simulator', stTwinTip: 'Move the robot by hand (released first) and the simulator follows. Exclusive with Live control and Stream.',
  mirL: 'L/R follow', mirOff: 'Off', mirAny: 'Simulator: either side leads', mirLR: 'Left → right (teach by hand on the left)', mirRL: 'Right → left (teach by hand on the right)',
  mirTip: 'Simulator: the opposite servo follows a slider. Robot: pick a direction and turn Twin on; the leader side is released and the other side is driven to the mirrored position (press "Start teaching" first).',
  stColJ: 'joint', stColAct: 'actual', stColCmd: 'command',
  stSum: function (ok, lat, cyc, miss, bad) { return ok + ' replies · ' + lat.toFixed(0) + ' ms · round ' + (cyc ? (cyc / 1000).toFixed(2) + ' s' : '—') + ' · timeouts ' + miss + (bad ? ' · bad ' + bad : ''); },
  stNote: 'One 07 01 00 05 ID 0E query per servo; up to 3 are in flight and the next goes out as each reply arrives (timeout 200 ms, longer on a slow link). Unwired / invalid servos are probed every 8th round. Actual = encoder value; command "free" = released. Note: the premaiduino2 firmware has no separate encoder, so actual = command there; moving the robot by hand needs firmware that reads positions back (factory). The reply layout is not verified on a factory robot.',
  teachL: 'Teach (real robot)', tcStart: '● Start teaching (release)', tcStartTip: 'Turns Twin and polling on and releases the servos (with "left → right" only the left side).', tcHold: '▣ Hold current pose', tcHoldTip: 'Stiffen every servo at its actual position (ends the limp state)',
  tcCap: '● Capture keyframe', tcAuto: '⏺ Auto capture', tcAutoStop: '■ Stop auto capture',
  tcCaptured: function (n, live) { return 'Captured keyframe ' + n + ' (' + live + ' servos from robot readings)'; }, tcNoData: 'No valid servo readings yet: turn on "Poll servo status"', tcNeedPoll: 'Connect the robot and turn on "Poll servo status" first', tcHeld: function (n) { return 'Holding ' + n + ' servos'; },
  teachConfirm: 'Every servo is released: the legs go limp at once. Support or hang the robot first, then confirm.', teachConfirmSide: 'Only the teaching side (left or right) is released: that leg goes limp. Support or hang the robot first, then confirm.',
  teachNote: 'Flow: hang the robot → start teaching → pose it by hand → capture keyframe (or auto capture) → hold current pose. Captures land in the keyframe list above: preview, export .pma or CSV.',
  recCsv: '⬇ Export CSV', recCsvTip: 'One row per keyframe: frame, time_ms, spd, raw and angle of every servo',
  mnTitle: 'Serial monitor · battery · Wake Lock', bvL: 'Voltage (read every 10 s; dashed red = 9 V)', bvTxt: function (v, mn, mx, n, min) { return v.toFixed(2) + ' V · min ' + mn.toFixed(2) + ' · max ' + mx.toFixed(2) + ' · ' + n + ' samples / ' + min.toFixed(1) + ' min'; },
  wakeL: 'Keep the screen on while connected', wakeNo: 'No Wake Lock in this browser', wakeOn: '✓ screen kept on', mnHide: 'Hide status polls', mnPause: 'Pause', mnClear: 'Clear', mnCopy: 'Copy', mnSend: 'Send', mnAutoTip: 'Adds LEN and XOR for you: type only CMD and payload',
  mnCnt: function (tx, rx) { return 'TX ' + tx + ' · RX ' + rx + ' packets'; }, mnBad: 'Enter hex bytes (for example 01 00 02 00 02)', mnFlash: 'This CMD is a flash-management command (0x04 / 0x1C / 0x1D / 0x1E): on the premaiduino2 firmware it erases and writes the motion storage. Really send it?',
  ledTitle: 'LED (0x1E, experimental)', ledWarn: '⚠ The LED packet format is not verified on hardware. On the premaiduino2 firmware 0x1E = SaveMotionData (erases/programs a flash motion page), so a wrong packet destroys stored motions. This box is only for trying payloads one by one on a factory robot while watching the replies in the serial monitor.', ledOk: 'I understand: a 0x1E sent here may write flash', ledSend: 'Send',
  ledConfirm: function (hex) { return 'Send ' + hex + ' ?'; }
});
Object.assign(I18N.zh, {
  balTitle: '站穩模式（腳踝 PD）', balOn: '站穩輔助', balNote: '用機身 IMU 令腳踝 pitch／roll 做 PD，再用重心側向偏差調髖／踝 roll。量度（呢個模擬器，推 0.1 s）：側推 9 N 原本跌、開咗要 9.5 N 先跌；前推 9 N → 10 N。提升好細（約 +0.5～1 N，5–10%）；單腳站立未解決；吊起（Hang body）時無效。', balInfo: function (f, s, tr) { return '前傾 ' + f.toFixed(1) + '° 側傾 ' + s.toFixed(1) + '° trim ' + tr.toFixed(1) + '°'; },
  senTitle: '感測 / 除錯 HUD', cSenL: '感測 HUD', cTrailL: '重心軌跡', senFoot: '腳底力', senTq: '力矩',
  shareTitle: '分享 · 錄影', shBtn: '🔗 分享連結', emBtn: '</> 嵌入碼', fsBtn: '⛶ 全螢幕', recBtn: '⏺ 錄影 WebM', recStop: '■ 停止錄影', gifBtn: 'GIF', copiedL: '已複製 ✓', noRec: '呢個瀏覽器唔支援錄影', recOnceL: '由頭播一次', recOnceTip: '錄影時由頭播放動作一次，完咗自動停',
  plPrev: '上一個', plNext: '下一個', plAutoL: '自動下一首', plShufL: '隨機', plWakeL: '播放時長亮',
  abATip: '設定 A 點（[）', abBTip: '設定 B 點（]）', abXTip: '清除 A-B 循環（\\）', abTxt: function (a, b) { return 'A ' + a + ' – B ' + b; },
  rcIns: '＋插入', rcInsTip: '喺選中幀之後插入目前姿勢', rcDup: '⧉ 複製', rcTrimB: '⇤ 裁前', rcTrimBTip: '刪走選中幀之前嘅所有幀', rcTrimA: '裁後 ⇥', rcTrimATip: '刪走選中幀之後嘅所有幀', rcSplit: '✂ 拆開', rcSplitTip: '由選中幀拆開，後半放入暫存', rcMerge: '⊕ 合併', rcMergeTip: '將暫存嘅後半接返落尾', onionL: '洋蔥皮（前一幀藍／後一幀橙）', rcAltTxt: function (n) { return '暫存 ' + n + ' 幀'; },
  teleTitle: '遙控（手制／鍵盤）', teleOn: '遙控模式', teleNote: '左搖桿＝頭，右搖桿＝手臂（左右＝側舉，上下＝前舉），LT／RT＝手肘，LB／RB 或 Q／E＝切換左右手，Y＝站穩，X＝播放。鍵盤：W A S D 頭、I J K L 手臂、U O 手肘。K 開關。', teleInfo: function (a) { return '而家控制：' + a; }, armR: '右手', armL: '左手',
  mcTitle: '鏡頭動作捕捉（手臂）', mcStart: '📷 開始', mcStop: '■ 停止', mcLoading: '載入 MediaPipe…', mcOn: '捕捉中', mcErr: function (m) { return '失敗：' + m; }, mcNote: '用 MediaPipe Pose 嘅世界座標，經 IK 帶動兩隻手臂。需要鏡頭權限同網絡（由 CDN 載入模型）；呢部分喺開發環境冇辦法實測，只測咗座標→關節嘅數學。',
  pwaOn: '✓ Service worker 已註冊：載入過一次之後可以離線用（可安裝成 App）', pwaOff: '只有 http(s) 先有離線／安裝功能',
  kbHelp: '<b>鍵盤快捷鍵</b><table><tr><td><kbd>Space</kbd></td><td>播放／暫停</td></tr><tr><td><kbd>←</kbd> <kbd>→</kbd></td><td>跳 ∓1 秒（Shift：5 秒）</td></tr><tr><td><kbd>,</kbd> <kbd>.</kbd></td><td>上一幀／下一幀</td></tr><tr><td><kbd>[</kbd> <kbd>]</kbd> <kbd>\\</kbd></td><td>A 點／B 點／清除循環</td></tr><tr><td><kbd>N</kbd> <kbd>P</kbd></td><td>下一首／上一首</td></tr><tr><td><kbd>R</kbd> <kbd>H</kbd></td><td>重置機身／吊起</td></tr><tr><td><kbd>1</kbd>–<kbd>6</kbd></td><td>視角（前、後、左、右、頂、3/4）</td></tr><tr><td><kbd>I</kbd> <kbd>C</kbd> <kbd>V</kbd> <kbd>T</kbd></td><td>IK、重心、感測 HUD、重心軌跡</td></tr><tr><td><kbd>B</kbd> <kbd>K</kbd></td><td>站穩輔助、遙控模式</td></tr><tr><td><kbd>L</kbd> <kbd>M</kbd></td><td>循環、音樂</td></tr><tr><td><kbd>S</kbd> <kbd>X</kbd> <kbd>G</kbd> <kbd>F</kbd></td><td>分享連結、錄影、GIF、全螢幕</td></tr><tr><td><kbd>Esc</kbd></td><td>緊急停止（關閉本說明）</td></tr><tr><td><kbd>?</kbd></td><td>顯示／隱藏本說明</td></tr></table><p class="sub">步行模式開啟時，W A S D 同方向鍵留畀步行，其他快捷鍵暫停。撳任何地方關閉。</p>'
});
Object.assign(I18N.en, {
  balTitle: 'Stand assist (ankle PD)', balOn: 'Stand assist', balNote: 'Ankle pitch/roll PD from the base IMU plus a lateral CoM trim on hip/ankle roll. Measured in this simulator (0.1 s pushes): sideways tipping goes from 9 N to 9.5 N, forward from 9 N to 10 N. The gain is small (+0.5..1 N, 5-10 %); single-leg stance is not solved; no effect while the body is hanging.', balInfo: function (f, s, tr) { return 'pitch ' + f.toFixed(1) + '° roll ' + s.toFixed(1) + '° trim ' + tr.toFixed(1) + '°'; },
  senTitle: 'Sensors / debug HUD', cSenL: 'Sensor HUD', cTrailL: 'CoM trail', senFoot: 'foot force', senTq: 'torque',
  shareTitle: 'Share · record', shBtn: '🔗 Share link', emBtn: '</> Embed code', fsBtn: '⛶ Fullscreen', recBtn: '⏺ Record WebM', recStop: '■ Stop recording', gifBtn: 'GIF', copiedL: 'Copied ✓', noRec: 'Recording is not supported here', recOnceL: 'From start, once', recOnceTip: 'Record plays the motion once from the start and stops by itself',
  plPrev: 'Previous', plNext: 'Next', plAutoL: 'Auto-next', plShufL: 'Shuffle', plWakeL: 'Keep screen on',
  abATip: 'Set point A ([)', abBTip: 'Set point B (])', abXTip: 'Clear the A-B loop (\\)', abTxt: function (a, b) { return 'A ' + a + ' – B ' + b; },
  rcIns: '＋ Insert', rcInsTip: 'Insert the current pose after the selected frame', rcDup: '⧉ Duplicate', rcTrimB: '⇤ Trim before', rcTrimBTip: 'Delete every frame before the selected one', rcTrimA: 'Trim after ⇥', rcTrimATip: 'Delete every frame after the selected one', rcSplit: '✂ Split', rcSplitTip: 'Split at the selected frame; the second half goes to a clipboard', rcMerge: '⊕ Merge', rcMergeTip: 'Append the clipboard half to the end', onionL: 'Onion skin (previous blue / next orange)', rcAltTxt: function (n) { return 'clipboard: ' + n + ' frames'; },
  teleTitle: 'Teleop (gamepad / keyboard)', teleOn: 'Teleop mode', teleNote: 'Left stick = head, right stick = arm (left/right = raise sideways, up/down = raise forward), LT/RT = elbow, LB/RB or Q/E = switch arm, Y = stand assist, X = play. Keyboard: W A S D head, I J K L arm, U O elbow. K toggles.', teleInfo: function (a) { return 'controlling: ' + a; }, armR: 'right arm', armL: 'left arm',
  mcTitle: 'Webcam mocap (arms)', mcStart: '📷 Start', mcStop: '■ Stop', mcLoading: 'Loading MediaPipe…', mcOn: 'tracking', mcErr: function (m) { return 'failed: ' + m; }, mcNote: 'MediaPipe Pose world landmarks drive both arms through IK. Needs camera permission and network (the model comes from a CDN); the runtime could not be tested in the development environment, only the landmark → joint math.',
  pwaOn: '✓ Service worker registered: works offline after the first load (installable as an app)', pwaOff: 'Offline / install only works over http(s)',
  kbHelp: '<b>Keyboard shortcuts</b><table><tr><td><kbd>Space</kbd></td><td>play / pause</td></tr><tr><td><kbd>←</kbd> <kbd>→</kbd></td><td>seek ∓1 s (Shift: 5 s)</td></tr><tr><td><kbd>,</kbd> <kbd>.</kbd></td><td>previous / next frame</td></tr><tr><td><kbd>[</kbd> <kbd>]</kbd> <kbd>\\</kbd></td><td>A point / B point / clear loop</td></tr><tr><td><kbd>N</kbd> <kbd>P</kbd></td><td>next / previous motion</td></tr><tr><td><kbd>R</kbd> <kbd>H</kbd></td><td>reset body / hang</td></tr><tr><td><kbd>1</kbd>–<kbd>6</kbd></td><td>views (front, back, left, right, top, 3/4)</td></tr><tr><td><kbd>I</kbd> <kbd>C</kbd> <kbd>V</kbd> <kbd>T</kbd></td><td>IK, CoM, sensor HUD, CoM trail</td></tr><tr><td><kbd>B</kbd> <kbd>K</kbd></td><td>stand assist, teleop mode</td></tr><tr><td><kbd>L</kbd> <kbd>M</kbd></td><td>loop, music</td></tr><tr><td><kbd>S</kbd> <kbd>X</kbd> <kbd>G</kbd> <kbd>F</kbd></td><td>share link, record, GIF, fullscreen</td></tr><tr><td><kbd>Esc</kbd></td><td>E-stop (also closes this help)</td></tr><tr><td><kbd>?</kbd></td><td>show / hide this help</td></tr></table><p class="sub">While walking is on, W A S D and the arrow keys belong to the gait and the other shortcuts pause. Click anywhere to close.</p>'
});
I18N.ja = {
  title: 'プリメイドAI · MuJoCo 物理シミュレーション · 25サーボ', loading: 'MuJoCo WASM を読み込み中（約11 MB・初回のみ）…', tabMotion: 'モーション', tabRobot: '実機', tabServos: 'サーボ', tabSet: '設定', tabWalk: '歩行', tabRec: '録画',
  vFront: '前', vBack: '後', vLeft: '左', vRight: '右', vTop: '上', vIso: '斜め', reset: '↺ リセット', tipReset: '機体を床に戻して現在のポーズからやり直す', pin: '吊り下げ', tipPin: '機体を空中に固定する（ベース固定）', ikChip: '🎯 IK', setTip: 'シミュレーション設定',
  estop: '■ 緊急停止', estopTip: '再生・歩行・ライブ制御・ティーチ録画を止め、実機へ STOP を送ります（サーボは脱力しません）。ショートカット: Esc',
  loadBtn: '📂 .pma を読み込む', loopOrig: '元ファイルのループに従う', loopOnce: '1回だけ再生', audTitle: '音楽', audLoad: '🎵 音楽を読み込む', audVol: '音量', advanced: '詳細',
  chkTitle: 'モーションチェック', walkParams: '歩容パラメータ', wLen: '歩幅', wRoll: '横揺れ', wPit: '振り', wSpd: 'ケイデンス', padHold: 'スティック位置を保持',
  recAdd: '● キーフレーム追加（現在のポーズ）', recSpd: '既定 SPD', recPlay: '▶ プレビュー', recExp: '⬇ .pma 書き出し', recClr: 'クリア', recTools: '編集ツール', recMirror: '↔ 左右反転', recImport: '⤓ 読み込み中のモーションを取り込む', recCsv: '⬇ CSV 書き出し',
  teachL: 'ティーチ（実機）', tcStart: '● ティーチ開始（脱力）', tcHold: '▣ 現在のポーズを保持', tcCap: '● キーフレーム取得', tcAuto: '⏺ 自動取得', tcAutoStop: '■ 自動取得を停止',
  rbBuiltinL: '内蔵モーション（0x1F）', rbStopMotion: '■ STOP（0x05）', rbModeL: '制御モード', mOff: 'シミュのみ', mLive: 'スライダー同期', mStream: 'PMA 再生', rbSpdL: '実機の速度', rbOnce: '現在のポーズを送信', rbT: 'T 待機', rbRel: '⚠ 全脱力', rbSafe: '安全リミット',
  rbConn: '🔌 実機に接続', rbDisc: '⏏ 切断', rbOff: '未接続', rbOn: '接続中', rbLost: '⚠ 接続が切れました。再接続中…', wsConn: '接続',
  stTitle: 'サーボ状態（0x01）· デジタルツイン', stPoll: 'サーボ状態をポーリング', stTwin: 'ツイン：実機 → シミュ', mirL: '左右フォロー', mirOff: 'オフ', mirAny: 'シミュ：どちら側でも', mirLR: '左 → 右（左を手で動かす）', mirRL: '右 → 左（右を手で動かす）', stColJ: '関節', stColAct: '実測', stColCmd: '指令',
  mnTitle: 'シリアルモニタ · バッテリー · Wake Lock', mnHide: 'ポーリングを隠す', mnPause: '一時停止', mnClear: 'クリア', mnCopy: 'コピー', mnSend: '送信', wakeL: '接続中は画面を点灯したままにする',
  intro: 'スライダーでサーボの目標角（度）を設定します。機体は重力と摩擦の影響を受けるので、バランスを崩すと倒れます。', presetL: 'プリセットポーズ', pZero: 'ゼロ T ポーズ', pDown: '腕を下げる', pFwd: '腕を前へ', pSquat: '半スクワット', toolsSum: 'ツール：スイープ · コピー · ペースト',
  copy: 'ポーズ JSON をコピー', paste: 'ポーズを貼り付け', setPhys: '物理', grav: '重力', com: '重心を表示', muL: '摩擦 μ', kpL: '剛性 kp', setPlay: '再生', interpL: '再生方式', smooth: 'なめらか', stepMode: 'ステップ',
  pushTitle: '押しテスト（0.1 秒）', pushFwd: '↑ 前', pushBack: '↓ 後', pushLeft: '← 左', pushRight: '→ 右', pushForce: '力', modelSummary: 'モデルと物理のメモ', stop: '■ 停止', tipAud: '音楽 オン／オフ', tipLoop: 'モーション全体を繰り返す', tipSpeed: '再生速度',
  g_head: '頭', g_r_arm: '右腕', g_l_arm: '左腕', g_r_leg: '右脚', g_l_leg: '左脚', play: '▶ 再生', pause: '⏸ 一時停止', resume: '▶ 再開', frame: 'フレーム', copied: 'コピーしました ✓', freeze: '❄ 物理を凍結', drop: '.pma／音楽ファイルをドロップ', notLoaded: '未読み込み',
  balTitle: '安定化モード（足首 PD）', balOn: '安定化アシスト', balNote: '機体 IMU から足首の pitch／roll を PD 制御し、重心の横ずれで股関節／足首の roll を補正します。実測（このシミュレータ、0.1 秒の押し）：横押しの転倒限界が 9 N から 9.5 N、前押しが 9 N から 10 N に上がります。効果は小さく（+0.5〜1 N、5〜10%）、片脚立ちは未解決。吊り下げ中は無効。', balInfo: function (f, s, tr) { return '前傾 ' + f.toFixed(1) + '° 横傾 ' + s.toFixed(1) + '° trim ' + tr.toFixed(1) + '°'; },
  senTitle: 'センサー／デバッグ HUD', cSenL: 'センサー HUD', cTrailL: '重心の軌跡', senFoot: '足裏の力', senTq: 'トルク',
  shareTitle: '共有 · 録画', shBtn: '🔗 共有リンク', emBtn: '</> 埋め込みコード', fsBtn: '⛶ 全画面', recBtn: '⏺ WebM 録画', recStop: '■ 録画停止', gifBtn: 'GIF', copiedL: 'コピーしました ✓', noRec: 'このブラウザは録画に対応していません', recOnceL: '最初から1回', recOnceTip: '録画時にモーションを最初から1回だけ再生し、終わると自動で停止します',
  plPrev: '前へ', plNext: '次へ', plAutoL: '自動で次へ', plShufL: 'シャッフル', plWakeL: '再生中は画面を消さない',
  abATip: 'A 点を設定（[）', abBTip: 'B 点を設定（]）', abXTip: 'A-B ループを解除（\\）', abTxt: function (a, b) { return 'A ' + a + ' – B ' + b; },
  rcIns: '＋ 挿入', rcInsTip: '選択フレームの後ろに現在のポーズを挿入', rcDup: '⧉ 複製', rcTrimB: '⇤ 前を削除', rcTrimBTip: '選択フレームより前をすべて削除', rcTrimA: '後を削除 ⇥', rcTrimATip: '選択フレームより後をすべて削除', rcSplit: '✂ 分割', rcSplitTip: '選択フレームで分割し、後半をクリップボードへ', rcMerge: '⊕ 結合', rcMergeTip: 'クリップボードの後半を末尾に追加', onionL: 'オニオンスキン（前＝青／次＝橙）', rcAltTxt: function (n) { return 'クリップボード: ' + n + ' フレーム'; },
  teleTitle: 'テレオペ（ゲームパッド／キーボード）', teleOn: 'テレオペモード', teleNote: '左スティック＝頭、右スティック＝腕（左右＝横に上げる、上下＝前に上げる）、LT/RT＝肘、LB/RB または Q/E＝腕の切り替え、Y＝安定化、X＝再生。キーボード：W A S D 頭、I J K L 腕、U O 肘。K で切り替え。', teleInfo: function (a) { return '操作中: ' + a; }, armR: '右腕', armL: '左腕',
  mcTitle: 'Web カメラ モーキャプ（腕）', mcStart: '📷 開始', mcStop: '■ 停止', mcLoading: 'MediaPipe を読み込み中…', mcOn: 'トラッキング中', mcErr: function (m) { return '失敗: ' + m; }, mcNote: 'MediaPipe Pose のワールド座標から IK で両腕を動かします。カメラ権限とネットワーク（モデルは CDN から）が必要です。開発環境では実行できず、ランドマーク→関節の計算のみテストしました。',
  pwaOn: '✓ Service worker 登録済み：一度読み込めばオフラインで動作（アプリとしてインストール可）', pwaOff: 'オフライン／インストールは http(s) のみ',
  kbHelp: '<b>キーボードショートカット</b><table><tr><td><kbd>Space</kbd></td><td>再生／一時停止</td></tr><tr><td><kbd>←</kbd> <kbd>→</kbd></td><td>∓1 秒シーク（Shift: 5 秒）</td></tr><tr><td><kbd>,</kbd> <kbd>.</kbd></td><td>前／次のフレーム</td></tr><tr><td><kbd>[</kbd> <kbd>]</kbd> <kbd>\\</kbd></td><td>A 点／B 点／ループ解除</td></tr><tr><td><kbd>N</kbd> <kbd>P</kbd></td><td>次／前のモーション</td></tr><tr><td><kbd>R</kbd> <kbd>H</kbd></td><td>リセット／吊り下げ</td></tr><tr><td><kbd>1</kbd>–<kbd>6</kbd></td><td>視点（前・後・左・右・上・斜め）</td></tr><tr><td><kbd>I</kbd> <kbd>C</kbd> <kbd>V</kbd> <kbd>T</kbd></td><td>IK、重心、センサー HUD、重心軌跡</td></tr><tr><td><kbd>B</kbd> <kbd>K</kbd></td><td>安定化アシスト、テレオペ</td></tr><tr><td><kbd>L</kbd> <kbd>M</kbd></td><td>ループ、音楽</td></tr><tr><td><kbd>S</kbd> <kbd>X</kbd> <kbd>G</kbd> <kbd>F</kbd></td><td>共有リンク、録画、GIF、全画面</td></tr><tr><td><kbd>Esc</kbd></td><td>緊急停止（このヘルプも閉じる）</td></tr><tr><td><kbd>?</kbd></td><td>このヘルプの表示／非表示</td></tr></table><p class="sub">歩行モード中は W A S D と矢印キーが歩行用になり、他のショートカットは止まります。どこかをクリックすると閉じます。</p>',
  polTitle: 'AI ポリシー（PPO・ブラウザ内推論）', polLoadB: 'デモポリシーを読み込む', polFileB: '.json を読み込む…', polPoseB: '↺ ポリシーの姿勢から開始', polOnL: 'ポリシーを実行', polInfo: function (tk, n) { return '読み込み済み: ' + tk + '・' + n + ' 層'; }, polErr: function (m) { return '失敗: ' + m; }, polNone: '先にポリシーを読み込んでください',
  polNote: 'rl/train.py で学習し rl/export_policy.py で書き出した MLP（50 Hz、観測＝ジャイロ・重力方向・関節誤差／速度・前回の行動）。「ポリシーの姿勢から開始」は学習時の初期姿勢（arms_down）に戻します。安定化アシストとは排他です。'
};
var LANG = 'zh';
try { var _l = localStorage.getItem('pmsim.lang'); if (_l === 'zh' || _l === 'en' || _l === 'ja') LANG = _l; } catch (e) {}
function t(key) {
  var d = I18N[LANG][key]; if (d === undefined && LANG !== 'zh') d = I18N.en[key]; if (d === undefined) d = I18N.zh[key];
  return typeof d === 'function' ? d.apply(null, Array.prototype.slice.call(arguments, 1)) : d;
}
var GROUPS = ['head', 'r_arm', 'l_arm', 'r_leg', 'l_leg'], groupEls = [];
var ARMS_DOWN = { r_shx_joint: 80, l_shx_joint: -80 };
var PRESETS = {
  zero: {},
  armsdown: ARMS_DOWN,
  armsfwd: { r_shy_joint: -90, l_shy_joint: -90, r_shx_joint: 80, l_shx_joint: -80 },
  squat: Object.assign({ r_hpy_joint: -20, l_hpy_joint: -20, r_kny_joint: 40, l_kny_joint: 40, r_aky_joint: -20, l_aky_joint: -20 }, ARMS_DOWN)
};
var N = A.servos.length;                                   // 25
var S = { mj: null, m: null, d: null, ready: false, run: true, pin: false, mu: A.params.mu, kp: A.params.kp, speed: 1, acc: 0, simT: 0, last: 0, statT: 0, tqT: 0, svSpeed: [], svStretch: [], cmd: [] };
var target = A.servos.map(function () { return 0; });      // degrees, one per servo (UI order)
for (var _k0 = 0; _k0 < N; _k0++) { S.svSpeed[_k0] = 0; S.svStretch[_k0] = 60; S.cmd[_k0] = 0; }   // 0x19 defaults: speed unlimited, stretch neutral
var SV_SPEED_LO = 30, SV_SPEED_HI = 600;                   // deg/s at speed 1 and 127 (ICS: larger = faster)
function stretchFactor(v) {                                // 1..127 (harder); 60 = neutral, same as the sample motions
  v = +v; if (!(v >= 1 && v <= 127)) v = 60;
  return v <= 60 ? 0.55 + 0.45 * (v / 60) : 1 + 0.5 * ((v - 60) / 67);
}
function touch() { if (SW.on) stopSweep(); pausePlay(); clearServoParams(); }
var jid = [], ui = [], SW = { on: false, k: 0, t: 0, base: 0 }, bodyMesh = {}, bid = {}, act = [], qadr = [], world, com;

