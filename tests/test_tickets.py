"""
Unit and integration tests for Embodex WebSocket ticketing, subprotocol negotiation,
local mode loopback isolation, and session metadata sanitization.
"""

import base64
import hashlib
import hmac
import json
import os
from pathlib import Path
import time
from unittest.mock import MagicMock

from fastapi.testclient import TestClient
import pytest

from embodex.api.security import (
    authenticate_websocket_connection,
    verify_ws_ticket,
    SECURITY_MODE_HARDWARE,
    SECURITY_MODE_LOCAL,
)
from embodex.api.server import create_app, list_recorded_sessions


def _make_ticket(payload: dict, secret: str, header: dict = None) -> str:
    if header is None:
        header = {"alg": "HS256", "typ": "JWT"}
    h_b64 = base64.urlsafe_b64encode(json.dumps(header).encode("utf-8")).decode("ascii").rstrip("=")
    p_b64 = base64.urlsafe_b64encode(json.dumps(payload).encode("utf-8")).decode("ascii").rstrip("=")
    signing_input = f"{h_b64}.{p_b64}".encode("ascii")
    sig = hmac.new(secret.encode("utf-8"), signing_input, hashlib.sha256).digest()
    sig_b64 = base64.urlsafe_b64encode(sig).decode("ascii").rstrip("=")
    return f"{h_b64}.{p_b64}.{sig_b64}"


@pytest.fixture
def mock_system():
    system = MagicMock()
    system.is_running = True
    system.config.enable_robot = True
    system.control_loop = MagicMock()
    system.control_loop.status = {"running": True}
    system.control_loop.robot_interface = None
    system.control_loop.recorder = None
    system.control_loop.task = None
    system.control_loop.record_root = None
    system.web_keyboard_handler = None
    system.vr_server = MagicMock(spec=["clients", "process_controller_data", "handle_grip_release"])
    system.vr_server.clients = set()
    system.command_queue = MagicMock()
    system.control_commands_queue = MagicMock()
    return system


def test_verify_valid_ticket():
    secret = "test-ws-ticket-secret-32-chars-ok"
    os.environ["EMBODEX_WS_TICKET_SECRET"] = secret

    payload = {
        "sub": "user_123",
        "role": "teleoperator",
        "exp": time.time() + 60,
        "scope": "teleop",
        "jti": "random-uuid-1",
    }
    ticket = _make_ticket(payload, secret)
    valid, decoded, err = verify_ws_ticket(ticket)
    assert valid is True
    assert decoded["sub"] == "user_123"
    assert decoded["role"] == "teleoperator"
    assert err is None


def test_verify_expired_ticket():
    secret = "test-ws-ticket-secret-32-chars-ok"
    os.environ["EMBODEX_WS_TICKET_SECRET"] = secret

    payload = {
        "sub": "user_123",
        "role": "teleoperator",
        "exp": time.time() - 10,  # expired 10s ago
        "scope": "teleop",
    }
    ticket = _make_ticket(payload, secret)
    valid, decoded, err = verify_ws_ticket(ticket)
    assert valid is False
    assert "expired" in err.lower()


def test_verify_wrong_signature():
    secret = "test-ws-ticket-secret-32-chars-ok"
    os.environ["EMBODEX_WS_TICKET_SECRET"] = secret

    payload = {
        "sub": "user_123",
        "role": "teleoperator",
        "exp": time.time() + 60,
        "scope": "teleop",
    }
    ticket = _make_ticket(payload, "different-wrong-secret")
    valid, decoded, err = verify_ws_ticket(ticket)
    assert valid is False
    assert "signature" in err.lower()


def test_verify_wrong_scope():
    secret = "test-ws-ticket-secret-32-chars-ok"
    os.environ["EMBODEX_WS_TICKET_SECRET"] = secret

    payload = {
        "sub": "user_123",
        "role": "teleoperator",
        "exp": time.time() + 60,
        "scope": "marketplace",  # wrong scope
    }
    ticket = _make_ticket(payload, secret)
    valid, decoded, err = verify_ws_ticket(ticket)
    assert valid is False
    assert "scope" in err.lower()


def test_websocket_subprotocol_ticket_accepted(mock_system):
    secret = "test-ws-ticket-secret-32-chars-ok"
    os.environ["EMBODEX_WS_TICKET_SECRET"] = secret
    os.environ["EMBODEX_SECURITY_MODE"] = "hardware"
    os.environ["EMBODEX_API_TOKEN"] = "permanent-api-token"

    app = create_app(mock_system)
    client = TestClient(app)

    payload = {
        "sub": "teleoperator_456",
        "role": "teleoperator",
        "exp": time.time() + 60,
        "scope": "teleop",
    }
    ticket = _make_ticket(payload, secret)
    subproto = f"embodex-ticket.{ticket}"

    with client.websocket_connect("/ws", subprotocols=[subproto]) as ws:
        assert getattr(ws, "accepted_subprotocol", None) == subproto


def test_websocket_bearer_protocol_accepted(mock_system):
    token = "permanent-api-token"
    os.environ["EMBODEX_SECURITY_MODE"] = "hardware"
    os.environ["EMBODEX_API_TOKEN"] = token

    app = create_app(mock_system)
    client = TestClient(app)

    subproto = f"bearer.{token}"
    with client.websocket_connect("/ws", subprotocols=[subproto]) as ws:
        assert getattr(ws, "accepted_subprotocol", None) == subproto


def test_websocket_invalid_ticket_rejected(mock_system):
    secret = "test-ws-ticket-secret-32-chars-ok"
    os.environ["EMBODEX_WS_TICKET_SECRET"] = secret
    os.environ["EMBODEX_SECURITY_MODE"] = "hardware"

    app = create_app(mock_system)
    client = TestClient(app)

    subproto = "embodex-ticket.invalid.ticket.format"
    with pytest.raises(Exception):
        with client.websocket_connect("/ws", subprotocols=[subproto]):
            pass


def test_local_mode_loopback_isolation(mock_system):
    os.environ["EMBODEX_SECURITY_MODE"] = "local"
    os.environ.pop("EMBODEX_API_TOKEN", None)

    app = create_app(mock_system)
    client = TestClient(app)

    # 1. Loopback request (127.0.0.1) should succeed without token
    resp_loopback = client.post("/api/robot", json={"action": "connect"})
    assert resp_loopback.status_code == 200

    # 2. Non-loopback client connection directly tested with authenticate_websocket_connection
    auth_ok, _, _ = authenticate_websocket_connection(
        headers={}, query_params={}, client_host="192.168.1.100", robot_enabled=True
    )
    assert auth_ok is False

    auth_loopback, _, _ = authenticate_websocket_connection(
        headers={}, query_params={}, client_host="127.0.0.1", robot_enabled=True
    )
    assert auth_loopback is True


def test_session_listing_does_not_leak_record_dir(tmp_path):
    # Create mock session files
    session_dir = tmp_path / "teleop_20260925_120000"
    session_dir.mkdir()
    (session_dir / "teleop_20260925_120000.json").write_text("{}")
    (session_dir / "meta.json").write_text(json.dumps({"start_time": "2026-09-25T12:00:00Z"}))

    sessions = list_recorded_sessions(tmp_path)
    assert len(sessions) == 1
    session = sessions[0]
    assert session["id"] == "teleop_20260925_120000"
    assert session["createdAt"] == "2026-09-25T12:00:00Z"
    assert "record_dir" not in session


def test_config_endpoint_is_protected(mock_system):
    os.environ["EMBODEX_SECURITY_MODE"] = "hardware"
    os.environ["EMBODEX_API_TOKEN"] = "secret-token"

    app = create_app(mock_system)
    client = TestClient(app)

    # Unauthenticated GET /api/config should return 401
    resp = client.get("/api/config")
    assert resp.status_code == 401

    # Authenticated GET /api/config should succeed
    resp_auth = client.get("/api/config", headers={"Authorization": "Bearer secret-token"})
    assert resp_auth.status_code == 200
