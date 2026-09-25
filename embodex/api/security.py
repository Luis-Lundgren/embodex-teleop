"""
Embodex Teleoperation Security, Access Control & Short-Lived Ticket Verification.

Defines security modes:
- local: Unauthenticated access permitted strictly for loopback clients (127.0.0.1, ::1).
- simulation: Read-only monitoring endpoints open; hardware-affecting commands require token/ticket.
- hardware: Strict authentication required for all control, trajectory, and WebSocket endpoints.

Authentication Mechanisms:
1. Permanent Service Bearer Token: EMBODEX_API_TOKEN
2. Short-Lived Browser WebSocket Ticket: EMBODEX_WS_TICKET_SECRET
   - Format: compact JWT signed with HMAC-SHA256 (HS256)
   - Lifetime: 30-120 seconds
   - Scope: "teleop"
   - Transport: Sec-WebSocket-Protocol: embodex-ticket.<ticket>
"""

import base64
import hashlib
import hmac
import json
import logging
import os
import time
from typing import Any, Dict, List, Optional, Tuple

from fastapi import HTTPException, Request, Security, status
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer

logger = logging.getLogger(__name__)

SECURITY_MODE_LOCAL = "local"
SECURITY_MODE_SIMULATION = "simulation"
SECURITY_MODE_HARDWARE = "hardware"

VALID_SECURITY_MODES = {SECURITY_MODE_LOCAL, SECURITY_MODE_SIMULATION, SECURITY_MODE_HARDWARE}
LOOPBACK_HOSTS = {"127.0.0.1", "::1", "localhost", "testclient"}

_bearer_security = HTTPBearer(auto_error=False)


def get_security_mode(robot_enabled: bool = False) -> str:
    """Determine current security mode from environment or runtime context."""
    env_mode = os.environ.get("EMBODEX_SECURITY_MODE", "").strip().lower()
    if env_mode in VALID_SECURITY_MODES:
        return env_mode
    if robot_enabled:
        return SECURITY_MODE_HARDWARE
    return SECURITY_MODE_SIMULATION


def get_api_token() -> Optional[str]:
    """Retrieve configured permanent API service token, if any."""
    token = os.environ.get("EMBODEX_API_TOKEN", "").strip()
    return token if token else None


def get_ws_ticket_secret() -> Optional[str]:
    """Retrieve shared secret used to verify short-lived browser WebSocket tickets."""
    secret = os.environ.get("EMBODEX_WS_TICKET_SECRET", "").strip()
    return secret if secret else None


def get_allowed_origins() -> List[str]:
    """
    Parse allowed CORS origins from EMBODEX_ALLOWED_ORIGINS.
    Defaults to localhost development origins if unspecified.
    """
    env_origins = os.environ.get("EMBODEX_ALLOWED_ORIGINS", "").strip()
    if env_origins:
        origins = [orig.strip().rstrip("/") for orig in env_origins.split(",") if orig.strip()]
        return origins

    # Safe local development defaults
    return [
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "http://localhost:8500",
        "http://127.0.0.1:8500",
    ]


def is_loopback_client(client_host: Optional[str]) -> bool:
    """Check if client IP is loopback."""
    if not client_host:
        return False
    return client_host in LOOPBACK_HOSTS


def verify_permanent_token(provided_token: Optional[str]) -> bool:
    """Constant-time comparison against EMBODEX_API_TOKEN."""
    expected_token = get_api_token()
    if not expected_token or not provided_token:
        return False
    return hmac.compare_digest(provided_token.encode("utf-8"), expected_token.encode("utf-8"))


# --- Compact HMAC-SHA256 (HS256) JWT verification without external dependencies ---

def _b64url_decode(s: str) -> bytes:
    """Decode base64url string with proper padding."""
    s += "=" * ((4 - len(s) % 4) % 4)
    return base64.urlsafe_b64decode(s)


def verify_ws_ticket(ticket_str: str) -> Tuple[bool, Optional[Dict[str, Any]], Optional[str]]:
    """
    Verify a short-lived WebSocket ticket signed with HMAC-SHA256.
    Returns (is_valid, payload, error_message).
    """
    secret = get_ws_ticket_secret()
    if not secret:
        return False, None, "EMBODEX_WS_TICKET_SECRET not configured on backend"

    parts = ticket_str.strip().split(".")
    if len(parts) != 3:
        return False, None, "Malformed ticket format (expected 3 parts)"

    header_b64, payload_b64, signature_b64 = parts

    # 1. Verify HMAC signature
    signing_input = f"{header_b64}.{payload_b64}".encode("ascii")
    expected_sig = hmac.new(secret.encode("utf-8"), signing_input, hashlib.sha256).digest()
    expected_sig_b64 = base64.urlsafe_b64encode(expected_sig).decode("ascii").rstrip("=")

    if not hmac.compare_digest(signature_b64.rstrip("="), expected_sig_b64):
        return False, None, "Invalid ticket signature"

    # 2. Decode header and payload
    try:
        header = json.loads(_b64url_decode(header_b64).decode("utf-8"))
        payload = json.loads(_b64url_decode(payload_b64).decode("utf-8"))
    except Exception as e:
        return False, None, f"Failed to parse ticket JSON: {e}"

    if header.get("alg") != "HS256":
        return False, None, f"Unsupported ticket algorithm: {header.get('alg')}"

    # 3. Validate expiration
    exp = payload.get("exp")
    if not isinstance(exp, (int, float)):
        return False, None, "Ticket missing expiration (exp)"

    now = time.time()
    if now > exp:
        return False, None, f"Ticket expired {int(now - exp)}s ago"

    # 4. Validate scope
    scope = payload.get("scope")
    if scope != "teleop":
        return False, None, f"Invalid ticket scope: expected 'teleop', got '{scope}'"

    return True, payload, None


async def require_auth_if_protected(
    request: Request,
    credentials: Optional[HTTPAuthorizationCredentials] = Security(_bearer_security),
    robot_enabled: bool = False,
):
    """
    FastAPI dependency enforcing authentication:
    - local mode: permits unauthenticated access ONLY from loopback (127.0.0.1, ::1).
    - simulation/hardware mode: requires valid Bearer service token.
    """
    mode = get_security_mode(robot_enabled=robot_enabled)
    client_host = request.client.host if request.client else None

    # Local mode: strictly enforce loopback check
    if mode == SECURITY_MODE_LOCAL:
        if is_loopback_client(client_host):
            return True
        logger.warning(
            f"Rejected non-loopback client in local mode: {client_host} -> {request.url.path}"
        )
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Unauthorized: Local mode only allows loopback connections (127.0.0.1).",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # In hardware or simulation mode with configured token:
    token = None
    if credentials and credentials.scheme.lower() == "bearer":
        token = credentials.credentials
    elif "authorization" in request.headers:
        header_val = request.headers["authorization"]
        if header_val.lower().startswith("bearer "):
            token = header_val[7:].strip()
    elif "x-api-key" in request.headers:
        token = request.headers["x-api-key"].strip()

    if not token or not verify_permanent_token(token):
        logger.warning(
            f"Unauthorized HTTP request rejected: {request.method} {request.url.path} "
            f"(mode={mode}, client={client_host or 'unknown'})"
        )
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Unauthorized: Valid EMBODEX_API_TOKEN Bearer credentials required.",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return True


def authenticate_websocket_connection(
    headers: dict,
    query_params: dict,
    client_host: Optional[str] = None,
    robot_enabled: bool = False,
) -> Tuple[bool, Optional[str], Optional[Dict[str, Any]]]:
    """
    Authenticates a WebSocket connection.
    Supports:
    1. Local loopback bypass (if mode == 'local' and host is loopback)
    2. Short-lived ticket in Sec-WebSocket-Protocol: 'embodex-ticket.<ticket>'
    3. Permanent service token in Authorization header: 'Bearer <token>'
    4. Permanent service token in Sec-WebSocket-Protocol: 'bearer.<token>'
    5. Permanent service token or ticket in query string (?token=) (legacy fallback)

    Returns (is_authenticated, accepted_subprotocol, ticket_payload).
    """
    mode = get_security_mode(robot_enabled=robot_enabled)

    # 1. Local mode check
    if mode == SECURITY_MODE_LOCAL:
        if is_loopback_client(client_host):
            return True, None, {"sub": "local-loopback", "scope": "teleop"}
        logger.warning(f"Rejected non-loopback WebSocket in local mode: {client_host}")
        return False, None, None

    # Check Sec-WebSocket-Protocol header (Browser preferred method)
    raw_protocol_header = headers.get("sec-websocket-protocol", "")
    if raw_protocol_header:
        protocols = [p.strip() for p in raw_protocol_header.split(",") if p.strip()]
        for p in protocols:
            # Check short-lived ticket subprotocol: embodex-ticket.<jwt>
            if p.startswith("embodex-ticket."):
                ticket_str = p[len("embodex-ticket."):].strip()
                valid, payload, err = verify_ws_ticket(ticket_str)
                if valid:
                    logger.info(
                        f"Accepted WebSocket connection via short-lived ticket: "
                        f"user={payload.get('sub')} role={payload.get('role')} (client={client_host})"
                    )
                    return True, p, payload
                else:
                    logger.warning(f"Rejected WebSocket ticket: {err} (client={client_host})")
                    return False, None, None

            # Check permanent token subprotocol: bearer.<token>
            if p.startswith("bearer."):
                token_str = p[len("bearer."):].strip()
                if verify_permanent_token(token_str):
                    logger.info(f"Accepted WebSocket via permanent bearer subprotocol (client={client_host})")
                    return True, p, {"sub": "service-token", "scope": "teleop"}

    # Check Authorization header (automation / curl clients)
    auth_header = headers.get("authorization", "")
    if auth_header.lower().startswith("bearer "):
        token = auth_header[7:].strip()
        if verify_permanent_token(token):
            logger.info(f"Accepted WebSocket via Authorization Bearer header (client={client_host})")
            return True, None, {"sub": "service-token", "scope": "teleop"}

    # Query parameter fallback (?token=...)
    param_token = query_params.get("token", "").strip()
    if param_token:
        # Check if query parameter is a signed ticket
        if param_token.count(".") == 2:
            valid, payload, _ = verify_ws_ticket(param_token)
            if valid:
                return True, None, payload
        # Check if query parameter is permanent token
        if verify_permanent_token(param_token):
            return True, None, {"sub": "service-token", "scope": "teleop"}

    # Simulation mode fallback if neither token nor ticket secret configured
    if mode == SECURITY_MODE_SIMULATION and not get_api_token() and not get_ws_ticket_secret():
        return True, None, {"sub": "simulation-public", "scope": "teleop"}

    return False, None, None


def authenticate_websocket(
    headers: dict,
    query_params: dict,
    client_host: Optional[str] = None,
    robot_enabled: bool = False,
) -> bool:
    """Backward-compatible boolean check for websocket authentication."""
    authenticated, _, _ = authenticate_websocket_connection(
        headers, query_params, client_host=client_host, robot_enabled=robot_enabled
    )
    return authenticated

