#!/usr/bin/env bash
# start_viewer.sh — start the waypoint 3D viewer (FastAPI + Vite dev server)
#
# Usage:
#   bash start_viewer.sh           # default: robodk_v1 conda env
#   CONDA_ENV=my_env bash start_viewer.sh
#
# Opens two background processes and tails their logs.
# Ctrl+C kills both.

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONDA_ENV="${CONDA_ENV:-robodk_v1}"
BACKEND_PORT=8000
FRONTEND_PORT=5173

# ── conda activate ────────────────────────────────────────────────────────────
CONDA_SH="${HOME}/miniconda3/etc/profile.d/conda.sh"
if [[ ! -f "$CONDA_SH" ]]; then
    echo "[ERROR] conda not found at $CONDA_SH"
    exit 1
fi
source "$CONDA_SH"
conda activate "$CONDA_ENV"

# ── log files ─────────────────────────────────────────────────────────────────
LOG_DIR="$REPO_ROOT/.viewer_logs"
mkdir -p "$LOG_DIR"
BACKEND_LOG="$LOG_DIR/backend.log"
FRONTEND_LOG="$LOG_DIR/frontend.log"

# ── cleanup on exit ───────────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "[viewer] Stopping servers..."
    kill "$BACKEND_PID" "$FRONTEND_PID" 2>/dev/null || true
    wait "$BACKEND_PID" "$FRONTEND_PID" 2>/dev/null || true
    echo "[viewer] Done."
}
trap cleanup EXIT INT TERM

# ── start FastAPI ─────────────────────────────────────────────────────────────
echo "[viewer] Starting FastAPI backend on port $BACKEND_PORT..."
uvicorn robodk_code.waypoint_server:app \
    --host 0.0.0.0 \
    --port "$BACKEND_PORT" \
    --reload \
    >"$BACKEND_LOG" 2>&1 &
BACKEND_PID=$!

# ── start Vite ────────────────────────────────────────────────────────────────
echo "[viewer] Starting Vite dev server on port $FRONTEND_PORT..."
cd "$REPO_ROOT/waypoint_viewer"
npm run dev -- --host --port "$FRONTEND_PORT" \
    >"$FRONTEND_LOG" 2>&1 &
FRONTEND_PID=$!
cd "$REPO_ROOT"

# ── wait for backend to be ready ──────────────────────────────────────────────
echo "[viewer] Waiting for backend..."
for i in {1..20}; do
    if curl -s "http://localhost:$BACKEND_PORT/api/waypoints" >/dev/null 2>&1; then
        break
    fi
    sleep 0.5
done

echo ""
echo "  Backend:  http://localhost:$BACKEND_PORT"
echo "  Frontend: http://localhost:$FRONTEND_PORT"
echo ""
echo "  Open http://localhost:$FRONTEND_PORT in your browser."
echo "  Logs: $LOG_DIR/"
echo "  Press Ctrl+C to stop."
echo ""

# ── tail both logs ────────────────────────────────────────────────────────────
tail -f "$BACKEND_LOG" "$FRONTEND_LOG"
