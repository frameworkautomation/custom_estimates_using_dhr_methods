#!/usr/bin/env bash
set -uo pipefail

# Ensure SSH agent is running
if [ -z "${SSH_AUTH_SOCK:-}" ]; then
  eval "$(ssh-agent -s)" > /dev/null
  ssh-add ~/.ssh/id_rsa 2>/dev/null
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLONES_DIR="$SCRIPT_DIR/../clones"
LOG_FILE="$SCRIPT_DIR/update_clones.log"

timestamp() {
    date '+%Y-%m-%d %H:%M:%S'
}

echo "=== Update run: $(timestamp) ===" | tee -a "$LOG_FILE"

if [[ ! -d "$CLONES_DIR" ]] || [[ -z "$(ls -A "$CLONES_DIR")" ]]; then
    echo "No repos found in clones/" | tee -a "$LOG_FILE"
    exit 0
fi

for repo_dir in "$CLONES_DIR"/*/; do
    repo_name="$(basename "$repo_dir")"

    if [[ ! -d "$repo_dir/.git" ]]; then
        echo "[$repo_name] Skipping — not a git repo" | tee -a "$LOG_FILE"
        continue
    fi

    echo "[$repo_name] Pulling from main..." | tee -a "$LOG_FILE"

    pull_output=$(git -C "$repo_dir" pull origin main 2>&1) && status="OK" || status="FAILED"

    echo "$pull_output" | while IFS= read -r line; do
        echo "[$repo_name]   $line" | tee -a "$LOG_FILE"
    done

    current_commit=$(git -C "$repo_dir" rev-parse HEAD 2>/dev/null || echo "unknown")
    echo "[$repo_name] Status: $status — HEAD: $current_commit" | tee -a "$LOG_FILE"
done

echo "=== Done: $(timestamp) ===" | tee -a "$LOG_FILE"
echo "" >> "$LOG_FILE"
