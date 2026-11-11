# PreMaid AI serial protocol (PMA) and `.pma` files

What is written here is what the web simulator (`web/src/js/60-robot-serial.js`, `70-robot-tools.js`, `pma/make_index.py`) and the custom
firmware [premaiduino2](https://github.com/arthurson/premaiduino2) implement. **"Verified" means checked against a factory PreMaid AI or the
community analysis memo; everything else is as implemented by premaiduino2 and not yet checked on a factory robot.**

## 1. Transport

Bluetooth SPP (RN42 module) or a USB serial adapter, 115200 8N1. The simulator talks to it with Web Serial (Chrome / Edge) or through a WebSocket
bridge (`tools/ws_bridge.py`, a Raspberry Pi, an ESP32 serial bridge) — the bytes are the same.

## 2. Packet framing

```
[LEN] [CMD] [payload ...] [XOR]
```

* `LEN` counts **all** bytes of the packet: `LEN`, `CMD`, the payload and `XOR`. Minimum 4.
* `XOR` = XOR of all preceding bytes (so the XOR of the whole packet is 0).
* A receiver that sees a bad `LEN` or checksum drops one byte and resynchronises (`web/src/js/60-robot-serial.js: rbRx`).

## 3. Commands

| CMD | payload | meaning | reply |
|:-:|---|---|---|
| `0x01` | `00 02 00 02` | battery voltage query (example packet `07 01 00 02 00 02 06`) | `06 01 00 lo hi xor`, volts = `(lo + 256·hi) / 216` |
| `0x01` | `00 05 PID 0E` | one servo's state (example `07 01 00 05 0B 0E xor`) | `12 01 00` + 14 bytes + `xor` (see 3.1) |
| `0x18` | `00 SPD (PID lo hi)×n` | move n servos to raw positions; duration = `SPD × 15 ms`; raw `0` = release the servo (free) | `04 18 00 1C` ok, `04 18 08 14` error |
| `0x19` | `TYP (PID val)×n` | servo parameters: `TYP 0x00` speed, `0x10` stretch (1…127) | — |
| `0x1F` | `00 ID` | play built-in motion `ID` (table in 3.3) — `05 1F 00 ID xor` | `04 1F 00 xor`; non-zero status = refused (another motion is running) |
| `0x1F` | `08 01` | stop the running motion — `05 1F 08 01 xor` | — |
| `0x05` | `00 01` | stop (the form the simulator sends first: `04 05 00 01`) | — |
| `0x02` `0x07` | see `pma/make_index.py` | loop start (iterations) / loop end inside a `.pma` motion stream | — |
| `0x04` `0x1C` `0x1D` `0x1E` | | motion-storage management of premaiduino2 (SaveMotionId, ReadFlash, WriteMotionData, SaveMotionData). **`0x1E` writes flash on premaiduino2; its factory meaning (LED?) is not verified — the simulator only offers a locked, confirm-every-time probe.** | |

The simulator sends **both** stop forms (`04 05 00 01` and `05 1F 08 01`, three times) on E-stop. It never releases the servos on E-stop: the robot would collapse.

### 3.1 Servo state reply (`0x01`, 18 bytes) — layout from the community memo, implemented by premaiduino2

| byte | meaning |
|:-:|---|
| 0 | `12` (length) |
| 1 | `01` |
| 2 | `00` status |
| 3 | PMA id |
| 4 | flags; `0x80` = servo present / valid |
| 5–6 | actual position (encoder), little endian |
| 7–8 | trim, signed |
| 9–10 | commanded position, little endian (`0` = released) |
| 11–12 | gyro-direction flag: `FF FF` none, `02 05` pitch, `02 07` roll |
| 13–14 | gyro gain, signed |
| 15 | speed |
| 16 | stretch |
| 17 | XOR |

premaiduino2 has no separate encoder: it reports actual = commanded. Reading the real position back (teach by hand) needs the factory firmware. The web simulator polls this reply for its servo-status table and digital twin.

### 3.2 Positions

`raw 7500` is the servo centre; `angle (deg) = sign × (raw − 7500) × 270 / 8000` (one raw step = 0.03375°). `sign` is per servo (all 25 directions have been verified on a real robot).
The raw range is per servo (table below). `0` is not a position: it releases the servo.

### 3.3 Servos and ranges

<!-- servo-table:begin -->
| # | joint | name | PMA id | sign | raw range | angle range (deg) | torque (N·m) |
|--:|---|---|:-:|:-:|---|---|---|
| 1 | `head_ny_joint` | neck pitch | 0x03 | + | 7200 … 8400 | -10 … 30 | 0.91 |
| 2 | `head_nz_joint` | head yaw | 0x05 | − | 5000 … 10000 | -84 … 84 | 0.91 |
| 3 | `head_nx_joint` | head roll | 0x07 | − | 6900 … 8100 | -20 … 20 | 0.91 |
| 4 | `r_shy_joint` | shoulder pitch | 0x02 | + | 1600 … 8800 | -199 … 44 | 1.37 |
| 5 | `r_shx_joint` | shoulder roll | 0x09 | + | 7450 … 10350 | -2 … 96 | 0.91 |
| 6 | `r_ely_joint` | upper-arm twist | 0x0D | + | 4000 … 11000 | -118 … 118 | 0.91 |
| 7 | `r_elx_joint` | elbow | 0x11 | + | 7100 … 11130 | -14 … 123 | 0.91 |
| 8 | `r_wry_joint` | wrist twist | 0x15 | + | 5900 … 14000 | -54 … 219 | 0.91 |
| 9 | `l_shy_joint` | shoulder pitch | 0x04 | − | 6300 … 13500 | -202 … 40 | 1.37 |
| 10 | `l_shx_joint` | shoulder roll | 0x0B | + | 4550 … 7550 | -100 … 2 | 0.91 |
| 11 | `l_ely_joint` | upper-arm twist | 0x0F | − | 4000 … 11000 | -118 … 118 | 0.91 |
| 12 | `l_elx_joint` | elbow | 0x13 | + | 3870 … 7900 | -123 … 14 | 0.91 |
| 13 | `l_wry_joint` | wrist twist | 0x17 | − | 1000 … 9000 | -51 … 219 | 0.91 |
| 14 | `r_hpz_joint` | hip yaw | 0x06 | + | 6250 … 8750 | -42 … 42 | 1.37 |
| 15 | `r_hpx_joint` | hip roll | 0x0A | + | 6800 … 8400 | -24 … 30 | 1.37 |
| 16 | `r_hpy_joint` | hip pitch | 0x0E | + | 4700 … 10200 | -94 … 91 | 1.37 |
| 17 | `r_kny_joint` | knee | 0x12 | − | 3950 … 7600 | -3 … 120 | 1.37 |
| 18 | `r_aky_joint` | ankle pitch | 0x16 | + | 5700 … 8300 | -61 … 27 | 1.37 |
| 19 | `r_akx_joint` | ankle roll | 0x1A | + | 6475 … 8825 | -35 … 45 | 1.37 |
| 20 | `l_hpz_joint` | hip yaw | 0x08 | + | 6250 … 8750 | -42 … 42 | 1.37 |
| 21 | `l_hpx_joint` | hip roll | 0x0C | + | 6600 … 8200 | -30 … 24 | 1.37 |
| 22 | `l_hpy_joint` | hip pitch | 0x10 | − | 4700 … 10200 | -91 … 94 | 1.37 |
| 23 | `l_kny_joint` | knee | 0x14 | + | 7400 … 11050 | -3 … 120 | 1.37 |
| 24 | `l_aky_joint` | ankle pitch | 0x18 | − | 6700 … 9300 | -61 … 27 | 1.37 |
| 25 | `l_akx_joint` | ankle roll | 0x1C | + | 6250 … 8500 | -42 … 34 | 1.37 |
<!-- servo-table:end -->

### 3.4 Built-in motions (`0x1F`)

<!-- builtin-table:begin -->
| id | motion | id | motion | id | motion | id | motion |
|:-:|---|:-:|---|:-:|---|:-:|---|
| 0x3D | ホーム | 0x3E | どうぞ | 0x3F | ワクワク | 0x40 | コシニテ |
| 0x41 | ようこそ | 0x42 | おねがい | 0x43 | バイバイ | 0x44 | 右にキス |
| 0x45 | 左にキス | 0x46 | ピストル | 0x47 | 敬礼 | 0x48 | えーん |
| 0x49 | ハート | 0x4A | キラッ | 0x4B | あなたへ | 0x4C | もんきー |
| 0x4D | GO×2 | 0x4E | エアギター | 0x4F | 右ターン | 0x50 | 左ターン |
<!-- builtin-table:end -->

## 4. `.pma` files (Kondo PreMaid motion export)

Text file, one or more motions:

```
[モーション(0)]
番号=0
名前=Embrace Blade
データ=18 ... (hex bytes of consecutive packets)
```

`データ` is the concatenation of the packets above (`0x18` poses, `0x19` parameter frames, loop markers `0x02`/`0x07`). `pma/make_index.py` parses it
(frames, seconds, sections, loops, BPM) and writes `pma/index.json` / `pma/library.js`; the simulator plays the poses through its own servo model, and streams the same bytes to a real robot.
