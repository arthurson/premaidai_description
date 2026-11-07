#!/usr/bin/env python3
"""Scan this folder and write pma/index.json, the library list shown by web/premaid-sim.html.

    python pma/make_index.py

One entry per *.pma.  Music is matched by file name: drop  <same name>.mp3 / .m4a / .aac / .ogg / .opus / .wav / .flac
next to the .pma and re-run this script.  A same-name .wsdcf (encrypted web-stream container) is listed as "drm" only:
a browser cannot play it, the page just tells the user to supply a normal audio file.
"""
import json, os, unicodedata

HERE = os.path.dirname(os.path.abspath(__file__))
AUDIO = ('.mp3', '.m4a', '.aac', '.ogg', '.opus', '.wav', '.flac')

def key(name):
    return unicodedata.normalize('NFC', os.path.splitext(name)[0]).casefold()

def main():
    files = sorted(os.listdir(HERE), key=lambda n: unicodedata.normalize('NFC', n).casefold())
    audio, drm = {}, {}
    for fn in files:
        ext = os.path.splitext(fn)[1].lower()
        if ext in AUDIO: audio.setdefault(key(fn), fn)
        elif ext == '.wsdcf': drm.setdefault(key(fn), fn)
    out = []
    for fn in files:
        if not fn.lower().endswith('.pma'): continue
        e = {'title': os.path.splitext(fn)[0], 'pma': fn}
        if key(fn) in audio: e['audio'] = audio[key(fn)]
        elif key(fn) in drm: e['drm'] = drm[key(fn)]
        out.append(e)
    with open(os.path.join(HERE, 'index.json'), 'w', encoding='utf-8', newline='\n') as f:
        json.dump(out, f, ensure_ascii=False, indent=1); f.write('\n')
    print('pma/index.json: %d motions, %d with audio, %d encrypted-only' % (len(out), sum('audio' in e for e in out), sum('drm' in e for e in out)))

if __name__ == '__main__': main()
