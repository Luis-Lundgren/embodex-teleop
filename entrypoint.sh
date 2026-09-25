#!/bin/sh
set -e

# Note on database ownership:
# Canonical marketplace database ownership belongs exclusively to `embodex-web`.
# `embodex-teleop` manages local session records in /app/records and communicates
# session data over HTTP/WebSocket APIs without altering database schemas.

# Execute the CMD passed to docker run
exec "$@"
