#!/usr/bin/env python3
"""Syntax-check the inline JavaScript of the simulator pages.

    python tools/check_js.py

Uses `node --check` on every <script> block without a src attribute.  The
generated page embeds a large JSON asset block, so this catches edits that
break either the source or the generated page.
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PAGES = ('web/premaid-sim.src.html', 'web/premaid-sim.html')
SCRIPT_RE = re.compile(r'<script(?![^>]*\bsrc=)[^>]*>(.*?)</script>', re.I | re.S)


def main():
    node = shutil.which('node') or shutil.which('node.exe')
    if not node:
        print('SKIP: node is not installed')
        return 0
    failed = False
    for rel in PAGES:
        path = os.path.join(PKG, rel)
        if not os.path.isfile(path):
            print('FAILED: missing %s' % rel)
            failed = True
            continue
        with open(path, encoding='utf-8') as f:
            html = f.read()
        scripts = SCRIPT_RE.findall(html)
        if not scripts:
            print('FAILED: %s has no inline scripts' % rel)
            failed = True
            continue
        fd, tmp = tempfile.mkstemp(suffix='.js')
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as f:
                f.write('\n;\n'.join(scripts))
            r = subprocess.run([node, '--check', tmp], capture_output=True, text=True)
            if r.returncode:
                print('FAILED: %s' % rel)
                print(r.stderr.strip())
                failed = True
            else:
                print('OK: %s (%d inline script block(s))' % (rel, len(scripts)))
        finally:
            os.unlink(tmp)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
