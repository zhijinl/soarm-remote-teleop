#!/usr/bin/env bash
# Reverse SSH tunnel so the remote can reach the local leader stream:
#   remote:127.0.0.1:$STREAM_PORT  ->  local:127.0.0.1:$STREAM_PORT   (where `soarm-local stream` listens)
# Everything rides the SSH connection — no extra exposed ports, encrypted. Works with any
# SSH-reachable remote (not tied to any provider).
#
# Configure via env vars:
#   REMOTE_HOST   (required)  ssh host of the remote machine
#   REMOTE_USER   (default: ubuntu)
#   REMOTE_KEY    (optional)  path to an identity file; omit to use your ssh-agent/config
#   STREAM_PORT   (default: 5599)
#
#   REMOTE_HOST=my.server.example.com REMOTE_KEY=~/.ssh/id_ed25519 ./scripts/tunnel.sh
set -euo pipefail

: "${REMOTE_HOST:?set REMOTE_HOST=<remote ssh host>}"
REMOTE_USER="${REMOTE_USER:-ubuntu}"
STREAM_PORT="${STREAM_PORT:-5599}"

key_opt=()
[ -n "${REMOTE_KEY:-}" ] && key_opt=(-i "${REMOTE_KEY}")

echo "Reverse tunnel: ${REMOTE_USER}@${REMOTE_HOST}  remote:127.0.0.1:${STREAM_PORT} -> local:127.0.0.1:${STREAM_PORT}"
exec ssh "${key_opt[@]}" \
    -o ServerAliveInterval=15 -o ServerAliveCountMax=3 -o ExitOnForwardFailure=yes \
    -N -R "${STREAM_PORT}:localhost:${STREAM_PORT}" \
    "${REMOTE_USER}@${REMOTE_HOST}"
