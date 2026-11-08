#!/bin/sh
# Run in the repo root.  Removes files nothing uses any more.
git rm -r --quiet web/samples          # 10 .pma, byte-identical to pma/ ; the generator no longer embeds them
git rm --quiet pma/*.wsdcf             # encrypted music (22 MB): browsers cannot play it, nothing reads it
git rm --quiet --ignore-unmatch pma/index.json   # replaced by pma/library.js
git add .gitignore pma/library.js README.md
echo "done - now commit"
