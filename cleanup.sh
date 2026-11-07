#!/bin/sh
# Run in the repo root.  Removes files nothing uses any more.
git rm -r --quiet web/samples          # 10 .pma, byte-identical to pma/ ; the generator no longer embeds them
git rm --quiet pma/*.wsdcf             # encrypted music (22 MB): browsers cannot play it, nothing reads it
git add .gitignore pma/index.json README.md
echo "done - now commit"
