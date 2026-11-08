import re, pathlib
src = pathlib.Path("web/premaid-sim.html").read_text(encoding="utf-8")
scripts = re.findall(r"<script(?![^>]*src=)[^>]*>(.*?)</script>", src, re.S|re.I)
print("scripts:", len(scripts), [len(s) for s in scripts])
pathlib.Path("_check.js").write_text("\n;\n".join(scripts), encoding="utf-8")
