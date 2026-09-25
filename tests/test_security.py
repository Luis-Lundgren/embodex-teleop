"""
Unit tests for Embodex API security, authentication, CORS, and mode isolation.
"""

import os
from unittest.mock import MagicMock

from fastapi.testclient import TestClient
import pytest

from embodex.api.security import (
    authenticate_websocket,
    get_allowed_origins,
    get_security_mode,
    SECURITY_MODE_HARDWARE,
    SECURITY_MODE_LOCAL,
    SECURITY_MODE_SIMULATION,
)
from embodex.api.server import create_app


@pytest.fixture
def mock_system():
    system = MagicMock()
    system.is_running = True
    system.config.enable_robot = False
    system.control_loop = MagicMock()
    system.control_loop.status = {"running": True}
    system.control_loop.robot_interface = None
    system.control_loop.recorder = None
    system.control_loop.task = None
    system.control_loop.record_root = None
    system.web_keyboard_handler = None
    system.vr_server = MagicMock()
    system.vr_server.clients = set()
    system.command_queue = MagicMock()
    system.control_commands_queue = MagicMock()
    return system


def test_security_mode_detection():
    # Explicit env var
    os.environ["EMBODEX_SECURITY_MODE"] = "hardware"
    assert get_security_mode(robot_enabled=False) == SECURITY_MODE_HARDWARE

    os.environ["EMBODEX_SECURITY_MODE"] = "local"
    assert get_security_mode(robot_enabled=True) == SECURITY_MODE_LOCAL

    # Default fallback when no env var set
    os.environ.pop("EMBODEX_SECURITY_MODE", None)
    assert get_security_mode(robot_enabled=True) == SECURITY_MODE_HARDWARE
    assert get_security_mode(robot_enabled=False) == SECURITY_MODE_SIMULATION


def test_cors_origin_parsing():
    os.environ["EMBODEX_ALLOWED_ORIGINS"] = "http://localhost:3000, https://www.embodex.online/"
    origins = get_allowed_origins()
    assert "http://localhost:3000" in origins
    assert "https://www.embodex.online" in origins
    os.environ.pop("EMBODEX_ALLOWED_ORIGINS", None)


def test_unauthenticated_hardware_command_rejected(mock_system):
    mock_system.config.enable_robot = True
    os.environ["EMBODEX_SECURITY_MODE"] = "hardware"
    os.environ["EMBODEX_API_TOKEN"] = "super-secret-token-123"

    try:
        app = create_app(mock_system)
        client = TestClient(app)

        # POST /api/robot without auth
        resp = client.post("/api/robot", json={"action": "connect"})
        assert resp.status_code == 401
        assert "Unauthorized" in resp.json().get("detail", "")

        # POST /api/keypress without auth
        resp = client.post("/api/keypress", json={"key": "w", "action": "press"})
        assert resp.status_code == 401

        # POST /api/config without auth
        resp = client.post("/api/config", json={})
        assert resp.status_code == 401
    finally:
        os.environ.pop("EMBODEX_SECURITY_MODE", None)
        os.environ.pop("EMBODEX_API_TOKEN", None)


def test_authenticated_hardware_command_accepted(mock_system):
    mock_system.config.enable_robot = True
    os.environ["EMBODEX_SECURITY_MODE"] = "hardware"
    os.environ["EMBODEX_API_TOKEN"] = "super-secret-token-123"

    try:
        app = create_app(mock_system)
        client = TestClient(app)

        # POST /api/robot with valid bearer token
        headers = {"Authorization": "Bearer super-secret-token-123"}
        resp = client.post("/api/robot", json={"action": "connect"}, headers=headers)
        assert resp.status_code == 200
        assert resp.json() == {"success": True, "action": "connect"}
    finally:
        os.environ.pop("EMBODEX_SECURITY_MODE", None)
        os.environ.pop("EMBODEX_API_TOKEN", None)


def test_invalid_token_rejected(mock_system):
    mock_system.config.enable_robot = True
    os.environ["EMBODEX_SECURITY_MODE"] = "hardware"
    os.environ["EMBODEX_API_TOKEN"] = "super-secret-token-123"

    try:
        app = create_app(mock_system)
        client = TestClient(app)

        # Wrong token
        headers = {"Authorization": "Bearer completely-wrong-token"}
        resp = client.post("/api/robot", json={"action": "connect"}, headers=headers)
        assert resp.status_code == 401
    finally:
        os.environ.pop("EMBODEX_SECURITY_MODE", None)
        os.environ.pop("EMBODEX_API_TOKEN", None)


def test_cors_allowed_and_rejected_origins(mock_system):
    os.environ["EMBODEX_ALLOWED_ORIGINS"] = "https://app.embodex.online,http://localhost:3000"

    try:
        app = create_app(mock_system)
        client = TestClient(app)

        # Allowed origin
        resp = client.options(
            "/health",
            headers={
                "Origin": "https://app.embodex.online",
                "Access-Control-Request-Method": "GET",
            },
        )
        assert resp.headers.get("access-control-allow-origin") == "https://app.embodex.online"

        # Unknown origin
        resp_unknown = client.options(
            "/health",
            headers={
                "Origin": "https://malicious-site.example.com",
                "Access-Control-Request-Method": "GET",
            },
        )
        assert resp_unknown.headers.get("access-control-allow-origin") != "https://malicious-site.example.com"
    finally:
        os.environ.pop("EMBODEX_ALLOWED_ORIGINS", None)


def test_public_routes_remain_accessible_in_hardware_mode(mock_system):
    mock_system.config.enable_robot = True
    os.environ["EMBODEX_SECURITY_MODE"] = "hardware"
    os.environ["EMBODEX_API_TOKEN"] = "secret-token"

    try:
        app = create_app(mock_system)
        client = TestClient(app)

        # /health is public
        health_resp = client.get("/health")
        assert health_resp.status_code == 200
        assert health_resp.json()["status"] == "ok"
        assert health_resp.json()["security_mode"] == "hardware"

        # /api/status is public
        status_resp = client.get("/api/status")
        assert status_resp.status_code == 200
        assert status_resp.json()["running"] is True
    finally:
        os.environ.pop("EMBODEX_SECURITY_MODE", None)
        os.environ.pop("EMBODEX_API_TOKEN", None)


def test_websocket_authentication_helpers():
    os.environ["EMBODEX_SECURITY_MODE"] = "hardware"
    os.environ["EMBODEX_API_TOKEN"] = "ws-secret-xyz"

    try:
        # No credentials -> rejected
        assert not authenticate_websocket({}, {}, robot_enabled=True)

        # Bearer header -> accepted
        assert authenticate_websocket(
            {"authorization": "Bearer ws-secret-xyz"}, {}, robot_enabled=True
        )

        # Query param ?token= -> accepted
        assert authenticate_websocket(
            {}, {"token": "ws-secret-xyz"}, robot_enabled=True
        )

        # Sec-WebSocket-Protocol -> accepted
        assert authenticate_websocket(
            {"sec-websocket-protocol": "bearer.ws-secret-xyz"}, {}, robot_enabled=True
        )

        # Wrong token -> rejected
        assert not authenticate_websocket(
            {}, {"token": "wrong"}, robot_enabled=True
        )
    finally:
        os.environ.pop("EMBODEX_SECURITY_MODE", None)
        os.environ.pop("EMBODEX_API_TOKEN", None)
