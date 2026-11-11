#!/usr/bin/env python3
"""Fill the generated tables of docs/PROTOCOL.md (servo ids / ranges, built-in motions) from the simulator data, so the docs cannot drift:
    python tools/make_docs.py [--verify]"""
import json, os, re, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); DOC = os.path.join(ROOT, 'docs', 'PROTOCOL.md')
html = open(os.path.join(ROOT, 'web', 'premaid-sim.html'), encoding='utf-8').read(); i = html.index('window.PREMAID=')
data = json.JSONDecoder().raw_decode(html[i + len('window.PREMAID='):])[0]; pma = {e['joint']: e for e in data['pma']}
K = 270 / 8000
rows = ['| # | joint | name | PMA id | sign | raw range | angle range (deg) | torque (N·m) |', '|--:|---|---|:-:|:-:|---|---|---|']
for n, s in enumerate(data['servos'], 1):
    e = pma[s['joint']]; a, b = sorted((e['sign'] * (e['rawLo'] - 7500) * K, e['sign'] * (e['rawHi'] - 7500) * K))
    rows.append('| %d | `%s` | %s | 0x%02X | %s | %d … %d | %.0f … %.0f | %.2f |' % (n, s['joint'], s['en'], e['pid'], '+' if e['sign'] > 0 else '−', e['rawLo'], e['rawHi'], a, b, s['effort']))
js = open(os.path.join(ROOT, 'web', 'src', 'js', '60-robot-serial.js'), encoding='utf-8').read(); blk = js[js.index('var BUILTIN = ['):]; blk = blk[:blk.index('];')]
items = re.findall(r"\{ id: 0x([0-9A-Fa-f]+), jp: '([^']*)' \}", blk); assert len(items) == 20
bi = ['| id | motion | id | motion | id | motion | id | motion |', '|:-:|---|:-:|---|:-:|---|:-:|---|']
for r in range(5): bi.append('| ' + ' | '.join('0x%s | %s' % (items[r * 4 + c][0].upper(), items[r * 4 + c][1]) for c in range(4)) + ' |')
tables = {'servo-table': '\n'.join(rows), 'builtin-table': '\n'.join(bi)}
text = open(DOC, encoding='utf-8', newline='').read(); new = text
for k, v in tables.items():
    pat = re.compile(r'(<!-- %s:begin -->\n)(?:.*?\n)?(<!-- %s:end -->)' % (k, k), re.S); assert pat.search(new), k; new = pat.sub(lambda m: m.group(1) + v + '\n' + m.group(2), new)
if '--verify' in sys.argv:
    print('OK: docs/PROTOCOL.md tables are up to date' if new == text else 'STALE: docs/PROTOCOL.md (run python tools/make_docs.py and commit)'); sys.exit(0 if new == text else 1)
open(DOC, 'w', encoding='utf-8', newline='\n').write(new); print('updated', DOC)
