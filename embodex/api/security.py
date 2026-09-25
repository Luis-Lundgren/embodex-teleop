"""
Embodex Teleoperation Security & Access Control.

Defines security modes:
- local: Unauthenticated local access permitted for rapid development.
- simulation: Read-only endpoints open; control commands optionally token-guarded.
- hardware: Strict authentication required for all hardware-affecting and private endpoints.

Configuration:
- EMBODEX_SECURITY_MODE: 'local' | 'simulation' | 'hardware' (default: 'hardware' if robot enabled, else 'simulation')
- EMBODEX_API_TOKEN: Bearer token secret required for protected endpoints
- EMBODEX_ALLOWED_ORIGINS: Comma-separated list of allowed CORS origins
"""

import hmac
import logging
import os
from typing import List, Optional

from fastapi import HTTPException, Request, Security, status
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer

logger = logging.getLogger(__name__)

# Security configuration from environment
SECURITY_MODE_LOCAL = "local"
SECURITY_MODE_SIMULATION = "simulation"
SECURITY_MODE_HARDWARE = "hardware"

VALID_SECURITY_MODES = {SECURITY_MODE_LOCAL, SECURITY_MODE_SIMULATION, SECURITY_MODE_HARDWARE}

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
    """Retrieve configured API token, if any."""
    token = os.environ.get("EMBODEX_API_TOKEN", "").strip()
    return token if token else None


def get_allowed_origins() -> List[str]:
    """
    Parse allowed CORS origins from EMBODEX_ALLOWED_ORIGINS.
    Defaults to localhost development origins if unspecified.
    """
    env_origins = os.environ.get("EMBODEX_ALLOWED_ORIGINS", "").strip()
    if env_origins:
        # Split by comma and strip whitespace & trailing slashes
        origins = [orig.strip().rstrip("/") for orig in env_origins.split(",") if orig.strip()]
        return origins

    # Safe local development defaults
    return [
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "http://localhost:8500",
        "http://127.0.0.1:8500",
    ]


def verify_token(provided_token: Optional[str]) -> bool:
    """Constant-time token comparison against EMBODEX_API_TOKEN."""
    expected_token = get_api_token()
    if not expected_token:
        # If no token is configured, verification cannot succeed in token-required modes
        return False
    if not provided_token:
        return False
    return hmac.compare_digest(provided_token.encode("utf-8"), expected_token.encode("utf-8"))


async def require_auth_if_protected(
    request: Request,
    credentials: Optional[HTTPAuthorizationCredentials] = Security(_bearer_security),
    robot_enabled: bool = False,
):
    """
    FastAPI dependency enforcing token authentication based on security mode.
    - local: unauthenticated access allowed.
    - simulation / hardware: requires valid Bearer token if token is configured or in hardware mode.
    """
    mode = get_security_mode(robot_enabled=robot_enabled)
    expected_token = get_api_token()

    if mode == SECURITY_MODE_LOCAL:
        return True

    # In hardware mode or when an API token is explicitly set, auth is strictly required
    if mode == SECURITY_MODE_HARDWARE or expected_token is not None:
        token = None
        if credentials and credentials.scheme.lower() == "bearer":
            token = credentials.credentials
        elif "authorization" in request.headers:
            header_val = request.headers["authorization"]
            if header_val.lower().startswith("bearer "):
                token = header_val[7:].strip()
        elif "x-api-key" in request.headers:
            token = request.headers["x-api-key"].strip()

        if not token or not verify_token(token):
            logger.warning(
                f"Unauthorized request rejected: {request.method} {request.url.path} "
                f"(mode={mode}, client={request.client.host if request.client else 'unknown'})"
            )
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Unauthorized: Valid EMBODEX_API_TOKEN Bearer credentials required.",
                headers={"WWW-Authenticate": "Bearer"},
            )

    return True


def authenticate_websocket(
    headers: dict,
    query_params: dict,
    robot_enabled: bool = False,
) -> bool:
    """
    Authenticate WebSocket connections using Bearer header, Sec-WebSocket-Protocol, or query parameter.
    """
    mode = get_security_mode(robot_enabled=robot_enabled)
    expected_token = get_api_token()

    if mode == SECURITY_MODE_LOCAL:
        return True

    # If neither in hardware mode nor is a token set, allow simulation WebSocket
    if mode != SECURITY_MODE_HARDWARE and expected_token is None:
        return True

    provided_token = None

    # 1. Authorization header (e.g. from Python or curl WebSocket client)
    auth_header = headers.get("authorization", "")
    if auth_header.lower().startswith("bearer "):
        provided_token = auth_header[7:].strip()

    # 2. Query param ?token= (browser/WebXR client fallback)
    if not provided_token and "token" in query_params:
        provided_token = query_params.get("token")

    # 3. Sec-WebSocket-Protocol header (supported natively by browser new WebSocket(url, ['bearer', token]))
    if not provided_token and "sec-websocket-protocol" in headers:
        protocols = [p.strip() for p in headers["sec-websocket-protocol"].split(",")]
        for p in protocols:
            if p.startswith("bearer."):
                provided_token = p[7:].strip()
                break

    if provided_token and verify_token(provided_token):
        return True

    return False
