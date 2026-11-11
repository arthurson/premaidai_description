#!/usr/bin/env python3
"""Scan this folder and write pma/library.js + pma/index.json.

    python pma/make_index.py           # regenerate
    python pma/make_index.py --check   # exit 1 if the generated files are stale

library.js is a plain script ( window.PMA_LIBRARY = [...] ) holding every .pma
base64-encoded, so the page can read the library from http(s) AND from file://
(browsers block fetch() there, but allow <script src>).  Music is NOT embedded:
it is referenced by file name and loaded by the page from this folder.  A
same-name .wsdcf (encrypted web-stream container) is only listed as "drm"; a
browser cannot play it.

The metadata is computed from the packets, using exactly the same rules as the
web player (see web/src/js/50-pma-replay.js):
  * 0x18 pose frames, duration = max(SPD * 15 ms, longest travel / 12.7 pulse/ms)
  * 0x08 0x02 / 0x08 0x07 loop blocks are expanded with a stack (iterations at
    payload byte 5, start = previous keyframe, as in the TechVerification
    MotionDataLoader.cs)
  * 0x19 speed/stretch packets are applied to the following frames
  * BPM is estimated from the dominant keyframe interval (an approximation;
    the official songs are not distributed with the .pma files)
"""
import base64
import json
import math
import os
import re
import sys
import unicodedata
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
AUDIO = ('.mp3', '.m4a', '.aac', '.ogg', '.opus', '.wav', '.flac')

# Servo IDs used by the 25 position frames.  The order only matters for the
# travel calculation (one slot per servo); it does not affect the UI mapping.
PMA_PIDS = (0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x14,
            0x16, 0x18, 0x1A, 0x1C, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D,
            0x0F, 0x11, 0x13, 0x15, 0x17)
PID2IDX = {pid: i for i, pid in enumerate(PMA_PIDS)}
PULSE_PER_MS = (60.0 / 140.0) * (8000.0 / 270.0)   # 12.698 pulse/ms, same as the page
TICK_MS = 15.0

# Japanese literals are written as escapes so the script stays readable even if
# the checkout / console uses a non-UTF-8 code page.
SECTION_RE = re.compile(r'\[\u30e2\u30fc\u30b7\u30e7\u30f3\((\d+)\)\]')
NAME_RE = re.compile(r'\u540d\u524d=([^\r\n]*)')
NUM_RE = re.compile(r'\u756a\u53f7=(\d+)')
DATA_MARK = '\u30c7\u30fc\u30bf='

HEX_RE = re.compile(r'^[0-9A-Fa-f]{2}$')


def key(name):
    return unicodedata.normalize('NFC', os.path.splitext(name)[0]).casefold()


def decode_text(raw):
    """UTF-8 (with or without BOM) first, Shift-JIS as fallback for old files."""
    for enc in ('utf-8-sig', 'shift_jis'):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            pass
    return raw.decode('utf-8', errors='replace')


def split_sections(text):
    """Return [{idx, num, name, bytes}] for every [モーション(n)] block."""
    starts = [m.start() for m in SECTION_RE.finditer(text)]
    out = []
    for n, start in enumerate(starts):
        end = starts[n + 1] if n + 1 < len(starts) else len(text)
        block = text[start:end]
        head = SECTION_RE.search(block)
        data_at = block.find(DATA_MARK)
        if data_at < 0:
            continue
        toks = block[data_at + len(DATA_MARK):].split()
        data = bytearray()
        for tok in toks:
            if not HEX_RE.match(tok):
                break
            data.append(int(tok, 16))
        if not data:
            continue
        nm = NAME_RE.search(block)
        num = NUM_RE.search(block)
        out.append({
            'idx': head.group(1) if head else str(n + 1),
            'num': num.group(1) if num else str(n + 1),
            'name': nm.group(1).strip() if nm else '',
            'bytes': bytes(data),
        })
    return out


def clone_frame(f):
    return {
        'raw': f['raw'][:],
        'free': f['free'][:],
        'spd': f['spd'],
        'speed': f['speed'][:],
        'stretch': f['stretch'][:],
    }


def parse_section(data, expand=True):
    """Packets -> expanded keyframes + loop metadata (mirrors buildTimeline())."""
    cur = [7500] * len(PMA_PIDS)
    free = [0] * len(PMA_PIDS)
    speed = [0] * len(PMA_PIDS)       # 0 = not specified / unlimited
    stretch = [60] * len(PMA_PIDS)    # 60 = neutral, as in the sample motions
    frames = []
    source_frames = []
    loops = []
    stack = []
    bad = other = 0
    i = 0

    while i < len(data):
        length = data[i]
        if length in (0, 0xFF) or i + length > len(data):
            break
        x = 0
        for j in range(length - 1):
            x ^= data[i + j]
        if x != data[i + length - 1]:
            bad += 1

        cmd = data[i + 1]
        if cmd == 0x18 and length >= 7:
            for q in range(4, length - 1, 3):
                k = PID2IDX.get(data[i + q])
                if k is None:
                    continue
                v = data[i + q + 1] | (data[i + q + 2] << 8)
                if v == 0:
                    free[k] = 1
                else:
                    free[k] = 0
                    cur[k] = v
            f = {
                'raw': cur[:],
                'free': free[:],
                'spd': data[i + 3],
                'speed': speed[:],
                'stretch': stretch[:],
            }
            frames.append(f)
            source_frames.append(f)
        elif cmd == 0x19 and length >= 6:
            typ = data[i + 2]
            for q in range(3, length - 1, 2):
                k = PID2IDX.get(data[i + q])
                if k is None:
                    continue
                v = data[i + q + 1]
                if typ == 0x10:
                    stretch[k] = v
                elif typ == 0x00:
                    speed[k] = v
            # An unknown 0x19 subtype is ignored (not fatal).
        elif cmd == 0x02 and length == 8:
            # 08 02 00 06 [id] [iterations] 00 xor
            stack.append({
                'start': max(0, len(frames) - 1),
                'iterations': max(1, data[i + 5]),
                'depth': len(stack) + 1,
            })
        elif cmd == 0x07 and length == 8:
            # 08 07 02 06 [id] 01 00 xor
            if stack:
                lp = stack.pop()
                lp['end'] = len(frames) - 1
                loops.append(lp)
                if expand and lp['iterations'] > 1 and lp['end'] >= lp['start']:
                    base = frames[lp['start']:lp['end'] + 1]
                    for _ in range(lp['iterations'] - 1):
                        frames.extend(clone_frame(f) for f in base)
        else:
            other += 1
        i += length

    return {
        'frames': frames,
        'sourceFrames': source_frames,
        'loops': loops,
        'bad': bad,
        'other': other,
    }


def timeline(frames):
    """Return (start times, durations, total) in ms, same as buildTimeline()."""
    times, durations = [], []
    t = 0.0
    for n, f in enumerate(frames):
        prev = frames[n - 1] if n else f
        travel = 0
        for s in range(len(PMA_PIDS)):
            if f['free'][s]:
                continue
            travel = max(travel, abs(f['raw'][s] - prev['raw'][s]))
        dur = max(max(f['spd'], 1) * TICK_MS, travel / PULSE_PER_MS)
        times.append(t)
        durations.append(dur)
        t += dur
    return times, durations, t


def estimate_bpm(source_frames):
    """Approximate the song tempo from the dominant keyframe interval.

    A .pma file has no BPM field and the music is not distributed with it, so
    this is a best-effort estimate: treat the most common SPD interval as an
    eighth note, normalise the resulting tempo to 90..200 BPM and prefer values
    around 140 BPM.  It is intentionally conservative and is marked as an
    estimate in the UI.  A hand-maintained pma/bpm.json can override it.
    """
    counts = Counter(f['spd'] for f in source_frames if f['spd'] >= 2)
    if not counts:
        return 0
    scores = Counter()
    for spd, count in counts.items():
        raw_bpm = 60.0 / (spd * TICK_MS / 1000.0)
        for octave in range(-4, 5):
            bpm = raw_bpm * (2.0 ** octave)
            if 90.0 <= bpm <= 200.0:
                scores[int(round(bpm))] += count * math.exp(-((bpm - 140.0) / 50.0) ** 2)
    return scores.most_common(1)[0][0] if scores else 0


def range_of(source_frames, field, ignore_zero=False):
    vals = [v for f in source_frames for v in f[field] if not (ignore_zero and v == 0)]
    return [min(vals), max(vals)] if vals else None


def analyse_section(data, expand=True):
    parsed = parse_section(data, expand=expand)
    times, durations, total = timeline(parsed['frames'])
    depth = max([lp['depth'] for lp in parsed['loops']] or [0])
    repeats = sum(max(0, lp['iterations'] - 1) for lp in parsed['loops'])
    return {
        'frames': len(parsed['frames']),
        'rawFrames': len(parsed['sourceFrames']),
        'milliseconds': total,
        'seconds': round(total / 1000.0, 2),
        'loops': len(parsed['loops']),
        'loopRepeats': repeats,
        'maxLoopDepth': depth,
        'bpm': estimate_bpm(parsed['sourceFrames']),
        'badChecksums': parsed['bad'],
        'otherPackets': parsed['other'],
        'stretch': range_of(parsed['sourceFrames'], 'stretch'),
        'speed': range_of(parsed['sourceFrames'], 'speed', ignore_zero=True),
    }


def analyse_file(path):
    sections = split_sections(decode_text(open(path, 'rb').read()))
    details = []
    for n, sec in enumerate(sections):
        st = analyse_section(sec['bytes'])
        st.update({'index': sec['idx'], 'num': sec['num'], 'name': sec['name'] or ('section %d' % (n + 1))})
        details.append(st)
    if not details:
        return None
    stretch_vals = [v for d in details if d['stretch'] for v in d['stretch']]
    speed_vals = [v for d in details if d['speed'] for v in d['speed']]
    timing = max(details, key=lambda d: d['milliseconds'])
    return {
        'frames': sum(d['frames'] for d in details),
        'rawFrames': sum(d['rawFrames'] for d in details),
        'seconds': round(sum(d['seconds'] for d in details), 2),
        'sections': len(details),
        'loops': sum(d['loops'] for d in details),
        'loopRepeats': sum(d['loopRepeats'] for d in details),
        'maxLoopDepth': max(d['maxLoopDepth'] for d in details),
        'bpm': next((d['bpm'] for d in details if d['bpm']), 0),
        'stretch': [min(stretch_vals), max(stretch_vals)] if stretch_vals else None,
        'speed': [min(speed_vals), max(speed_vals)] if speed_vals else None,
        'sectionInfo': details,
    }


def library_entry(filename, path):
    meta = analyse_file(path)
    if meta is None:
        return None
    entry = {'title': os.path.splitext(filename)[0], 'pma': filename}
    entry.update(meta)
    with open(path, 'rb') as f:
        entry['data'] = base64.b64encode(f.read()).decode('ascii')
    return entry


def public_entry(entry):
    return {k: v for k, v in entry.items() if k != 'data'}


def render(entries):
    js = 'window.PMA_LIBRARY=' + json.dumps(
        entries, ensure_ascii=False, separators=(',', ':')
    ).replace('</', '<\\/') + ';\n'
    js = '/* generated by pma/make_index.py - do not edit */\n' + js
    idx = json.dumps([public_entry(e) for e in entries], ensure_ascii=False, indent=1) + '\n'
    return js, idx


def main(argv):
    files = sorted(os.listdir(HERE), key=lambda n: unicodedata.normalize('NFC', n).casefold())
    audio, drm = {}, {}
    for fn in files:
        ext = os.path.splitext(fn)[1].lower()
        if ext in AUDIO:
            audio.setdefault(key(fn), fn)
        elif ext == '.wsdcf':
            drm.setdefault(key(fn), fn)

    overrides = {}
    bpm_path = os.path.join(HERE, 'bpm.json')
    if os.path.exists(bpm_path):
        try:
            overrides = json.load(open(bpm_path, encoding='utf-8'))
        except (OSError, ValueError) as exc:
            print('warning: could not read pma/bpm.json: %s' % exc)

    entries = []
    for fn in files:
        if not fn.lower().endswith('.pma'):
            continue
        path = os.path.join(HERE, fn)
        entry = library_entry(fn, path)
        if entry is None:
            print('warning: no motion data in %s' % fn)
            continue
        if key(fn) in audio:
            entry['audio'] = audio[key(fn)]
        elif key(fn) in drm:
            entry['drm'] = drm[key(fn)]
        if entry['title'] in overrides:
            try:
                entry['bpm'] = int(round(float(overrides[entry['title']])))
            except (TypeError, ValueError):
                pass
        entries.append(entry)

    js, idx = render(entries)
    js_path = os.path.join(HERE, 'library.js')
    idx_path = os.path.join(HERE, 'index.json')
    check = '--check' in argv
    stale = []
    for path, text in ((js_path, js), (idx_path, idx)):
        old = None
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8', newline='') as f:
                old = f.read()
        if old != text:
            stale.append(os.path.basename(path))
            if not check:
                with open(path, 'w', encoding='utf-8', newline='\n') as f:
                    f.write(text)
    if check:
        if stale:
            print('stale generated files: %s (run python pma/make_index.py)' % ', '.join(stale))
            return 1
        print('pma/index.json and pma/library.js are up to date')
        return 0

    print('pma/library.js: %d motions, %d with audio, %d encrypted-only, %d KB' % (
        len(entries), sum('audio' in e for e in entries), sum('drm' in e for e in entries),
        os.path.getsize(js_path) // 1024))
    print('pma/index.json: %d motions, metadata only' % len(entries))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
