#!/usr/bin/env bash
set -uo pipefail

# Ensure SSH agent is running
if [ -z "${SSH_AUTH_SOCK:-}" ]; then
  eval "$(ssh-agent -s)" > /dev/null
  ssh-add ~/.ssh/id_rsa 2>/dev/null
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOS_FILE="$SCRIPT_DIR/repos.txt"
CLONES_DIR="$SCRIPT_DIR/../clones"

if [[ ! -f "$REPOS_FILE" ]]; then
    echo "Error: repos.txt not found at $REPOS_FILE"
    exit 1
fi

mkdir -p "$CLONES_DIR"

errors=0

while IFS= read -r line; do
    # Skip empty lines and comments
    [[ -z "$line" || "$line" == \#* ]] && continue

    # Split on | to extract optional extra clone flags
    repo_url="${line%%|*}"
    repo_url="${repo_url%"${repo_url##*[! ]}"}"  # trim trailing whitespace
    extra_flags=""
    if [[ "$line" == *"|"* ]]; then
        extra_flags="${line#*|}"
        extra_flags="${extra_flags#"${extra_flags%%[! ]*}"}"  # trim leading whitespace
        extra_flags="${extra_flags%"${extra_flags##*[! ]}"}"  # trim trailing whitespace
    fi

    repo_name="$(basename "$repo_url" .git)"

    if [[ -d "$CLONES_DIR/$repo_name" ]]; then
        echo "Skipping $repo_name — already exists in clones/"
    else
        echo "Cloning $repo_url into clones/$repo_name ..."
        if git clone $extra_flags "$repo_url" "$CLONES_DIR/$repo_name"; then
            echo "OK: $repo_name cloned."
        else
            echo "ERROR cloning $repo_name"
            errors=$((errors + 1))
        fi
    fi
done < "$REPOS_FILE"

if [[ $errors -gt 0 ]]; then
    echo "Done with $errors error(s)."
else
    echo "Done."
fi
