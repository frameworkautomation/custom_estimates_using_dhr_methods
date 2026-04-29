# Project Context for Claude

## Repo cloning setup

There is a `cloning_stuff/` folder at the project root that manages external repo dependencies.

### Files

- **`cloning_stuff/repos.txt`** — one SSH URL per line (e.g. `git@github.com:user/repo.git`). Lines starting with `#` are ignored. The user fills this in manually.
- **`cloning_stuff/make_clones.sh`** — reads `repos.txt` and clones each repo into `clones/`. Skips repos already cloned. Prints errors and continues if a clone fails (e.g. SSH key not set up yet).
- **`cloning_stuff/update_clones.sh`** — iterates every repo in `clones/`, pulls from `origin main`, and appends a timestamped log to `cloning_stuff/update_clones.log`. Prints errors per repo and continues.
- **`cloning_stuff/update_clones.log`** — tracked by git so the user can see a history of when external assets were updated.

### Directory layout

```
.
├── .gitignore              # ignores clones/
├── CLAUDE.md               # this file
├── clones/                 # cloned repos land here (git-ignored)
└── cloning_stuff/
    ├── repos.txt
    ├── make_clones.sh
    ├── update_clones.sh
    └── update_clones.log
```

### Notes
- `clones/` is git-ignored — never commit it.
- SSH keys need to be configured on the machine for `make_clones.sh` and `update_clones.sh` to succeed. Until then, both scripts will print the SSH errors and continue rather than exiting immediately.
- Both scripts use `set -uo pipefail` (not `-e`) so a single failure doesn't abort the whole run.
