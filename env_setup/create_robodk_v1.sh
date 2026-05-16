#!/usr/bin/env bash
# Creates the robodk_v1 conda environment for running robodk_code scripts.
# Run from anywhere. Requires miniconda at ~/miniconda3.

set -euo pipefail

CONDA="$HOME/miniconda3/bin/conda"

if [[ ! -x "$CONDA" ]]; then
    echo "ERROR: conda not found at $CONDA"
    echo "Install miniconda first: https://docs.conda.io/en/latest/miniconda.html"
    exit 1
fi

echo "Creating conda environment: robodk_v1 ..."
"$CONDA" create -n robodk_v1 python=3.11 numpy tk pip -y

echo "Installing robodk via pip ..."
"$CONDA" run -n robodk_v1 pip install robodk

echo ""
echo "Done. To activate:"
echo "  source ~/miniconda3/etc/profile.d/conda.sh"
echo "  conda activate robodk_v1"
