# robot_descriptions.py entry

`robot_descriptions` (https://github.com/robot-descriptions/robot_descriptions.py) loads descriptions from git repositories.
This folder holds what the PR needs:

* `premaidai_description.py`  -> `robot_descriptions/premaidai_description.py`
* `descriptions_entry.txt`    -> add to `DESCRIPTIONS` in `robot_descriptions/_descriptions.py`
* `repositories_entry.txt`    -> add to `REPOSITORIES` in `robot_descriptions/_repositories.py` (replace `@COMMIT@` by a commit/tag of this repo)

Before opening the PR, `python contrib/robot_descriptions/overlay_test.py` loads the robot through robot_descriptions' own MuJoCo and
yourdfpy loaders from a patched copy of the installed package (no network, uses this checkout).
The upstream PR also touches `README.md` (table) and `.github` test lists; follow their CONTRIBUTING.
