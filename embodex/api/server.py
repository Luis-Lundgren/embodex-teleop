"""
Embodex FastAPI Server.
Exposes REST endpoints, WebXR WebSocket proxy, static files, and health checks with
security mode and origin allowlisting.
"""

import asyncio
import json
import logging
from pathlib import Path
import time
from typing import Optional, TYPE_CHECKING

from fastapi import Depends, FastAPI, HTTPException, Request, Security, WebSocket, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.security import HTTPAuthorizationCredentials
from fastapi.staticfiles import StaticFiles
import uvicorn

from .security import (
    authenticate_websocket,
    authenticate_websocket_connection,
    get_allowed_origins,
    get_security_mode,
    require_auth_if_protected,
    _bearer_security,
)

if TYPE_CHECKING:
    from ..cli import EmbodexTeleopSystem

logger = logging.getLogger(__name__)


def list_recorded_sessions(record_root: Path, active_session_id: Optional[str] = None) -> list:
    """Scan record directories for completed teleop sessions."""
    sessions = []
    seen = set()
    if record_root.exists():
        for json_file in sorted(record_root.glob("*/teleop_*.json"), reverse=True):
            session_id = json_file.stem
            if session_id in seen:
                continue
            seen.add(session_id)
            meta_path = json_file.parent / "meta.json"
            created_at = None
            if meta_path.exists():
                try:
                    with open(meta_path) as f:
                        meta = json.load(f)
                    created_at = meta.get("start_time")
                except Exception:
                    pass
            sessions.append({
                "id": session_id,
                "createdAt": created_at,
                "recording": False,
            })
    if active_session_id and active_session_id not in seen:
        sessions.insert(0, {"id": active_session_id, "recording": True})
    return sessions


def load_session_data(record_root: Path, session_id: str) -> Optional[dict]:
    """Load Motion Exchange JSON for a session by ID."""
    if not record_root.exists():
        return None
    for json_file in record_root.glob(f"*/{session_id}.json"):
        try:
            with open(json_file) as f:
                return json.load(f)
        except Exception as e:
            logger.error(f"Failed to read session {session_id}: {e}")
    return None


async def _handle_fastapi_websocket(
    system: "EmbodexTeleopSystem",
    websocket: WebSocket,
    subprotocol: Optional[str] = None,
):
    """
    Handle WebXR WebSocket connection, forwarding VR controller data to system command queue
    and supporting recording toggle.
    """
    await websocket.accept(subprotocol=subprotocol)
    client_address = f"{websocket.client.host}:{websocket.client.port}" if websocket.client else "unknown"
    logger.info(f"VR client connected: {client_address}")

    vr_server = system.vr_server
    if vr_server:
        vr_server.clients.add(websocket)

    try:
        while True:
            message = await websocket.receive_text()
            try:
                data = json.loads(message)
                action = data.get("action")
                if action == "record_toggle":
                    logger.info("🔴 VR record_toggle requested via WebSocket")
                    if system.control_loop:
                        from telegrip.inputs.base import ControlGoal
                        await system.command_queue.put(
                            ControlGoal(arm="left", metadata={"record_toggle": True})
                        )
                elif action == "auth":
                    # In-band auth confirmation
                    continue
                elif vr_server and hasattr(vr_server, "process_controller_data"):
                    await vr_server.process_controller_data(data)
            except json.JSONDecodeError:
                logger.warning(f"Received non-JSON message: {message}")
            except Exception as e:
                logger.error(f"Error processing VR data: {e}")
    except Exception as e:
        logger.info(f"VR client {client_address} disconnected: {e}")
    finally:
        if vr_server:
            vr_server.clients.discard(websocket)
            if hasattr(vr_server, "handle_grip_release"):
                res_l = vr_server.handle_grip_release("left")
                if asyncio.iscoroutine(res_l):
                    await res_l
                res_r = vr_server.handle_grip_release("right")
                if asyncio.iscoroutine(res_r):
                    await res_r
        logger.info(f"VR client {client_address} cleanup complete")


def _attach_vr_broadcaster_helpers(vr_server):
    """Ensure broadcast helper methods are attached to vr_server for WebXR digital twin."""
    if not hasattr(vr_server, "broadcast_robot_state"):
        async def broadcast_robot_state(
            left_angles=None,
            right_angles=None,
            is_recording=False,
            session_id=None,
            record_dir=None,
            objects=None,
            task=None,
        ):
            if not getattr(vr_server, "clients", None):
                return
            msg = {
                "type": "robot_state",
                "timestamp": int(time.time() * 1000),
                "left_arm": left_angles.tolist() if hasattr(left_angles, "tolist") else (left_angles or []),
                "is_recording": is_recording,
                "session_id": session_id,
                "record_dir": record_dir,
            }
            if right_angles is not None:
                msg["right_arm"] = right_angles.tolist() if hasattr(right_angles, "tolist") else right_angles
            if objects:
                msg["objects"] = objects
            if task:
                msg["task"] = task

            encoded = json.dumps(msg)
            for client in list(vr_server.clients):
                try:
                    if hasattr(client, "send_text"):
                        await client.send_text(encoded)
                    elif hasattr(client, "send"):
                        await client.send(encoded)
                except Exception:
                    pass

        setattr(vr_server, "broadcast_robot_state", broadcast_robot_state)

    if not hasattr(vr_server, "broadcast_recording_stopped"):
        async def broadcast_recording_stopped(session_id, record_dir):
            if not getattr(vr_server, "clients", None):
                return
            msg = {
                "type": "recording_stopped",
                "session_id": session_id,
                "record_dir": record_dir,
            }
            encoded = json.dumps(msg)
            for client in list(vr_server.clients):
                try:
                    if hasattr(client, "send_text"):
                        await client.send_text(encoded)
                    elif hasattr(client, "send"):
                        await client.send(encoded)
                except Exception:
                    pass

        setattr(vr_server, "broadcast_recording_stopped", broadcast_recording_stopped)


def create_app(system: "EmbodexTeleopSystem") -> FastAPI:
    """Create a FastAPI application bound to an EmbodexTeleopSystem instance."""
    app = FastAPI(title="Embodex Teleoperation API", version="0.1.0")

    robot_enabled = getattr(system.config, "enable_robot", False)
    allowed_origins = get_allowed_origins()

    # If wildcard is configured, credentials must not be allowed
    allow_credentials = False if "*" in allowed_origins else True

    app.add_middleware(
        CORSMiddleware,
        allow_origins=allowed_origins,
        allow_credentials=allow_credentials,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Attach VR broadcaster helpers if vr_server is active
    if system.vr_server:
        _attach_vr_broadcaster_helpers(system.vr_server)

    async def auth_dependency(
        request: Request,
        credentials: Optional[HTTPAuthorizationCredentials] = Security(_bearer_security),
    ):
        return await require_auth_if_protected(
            request, credentials=credentials, robot_enabled=robot_enabled
        )

    # ------------------ Public Read-Only Endpoints ------------------

    @app.get("/health")
    async def health():
        """Health check endpoint indicating service, robot, and recording status."""
        loop = system.control_loop
        robot_engaged = False
        if loop and loop.robot_interface:
            robot_engaged = getattr(loop.robot_interface, "is_engaged", False)

        return {
            "status": "ok",
            "system_running": system.is_running,
            "robot_engaged": robot_engaged,
            "recording": loop.recorder.is_running() if (loop and loop.recorder) else False,
            "task_active": (loop.task is not None) if loop else False,
            "security_mode": get_security_mode(robot_enabled=robot_enabled),
        }

    @app.get("/api/status")
    async def get_status():
        try:
            control_status = system.control_loop.status if system.control_loop else {}
            keyboard_enabled = False
            if system.web_keyboard_handler and hasattr(system.web_keyboard_handler, "is_enabled"):
                keyboard_enabled = system.web_keyboard_handler.is_enabled

            robot_engaged = False
            if system.control_loop and system.control_loop.robot_interface:
                robot_engaged = getattr(system.control_loop.robot_interface, "is_engaged", False)

            vr_connected = False
            if system.vr_server and getattr(system.vr_server, "is_running", False):
                vr_connected = len(getattr(system.vr_server, "clients", [])) > 0
            elif system.vr_server and hasattr(system.vr_server, "clients"):
                vr_connected = len(system.vr_server.clients) > 0

            return {
                **control_status,
                "keyboardEnabled": keyboard_enabled,
                "robotEngaged": robot_engaged,
                "vrConnected": vr_connected,
                "securityMode": get_security_mode(robot_enabled=robot_enabled),
            }
        except Exception as e:
            logger.error(f"Error handling status request: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.get("/api/sessions")
    async def get_sessions():
        """List session metadata summaries."""
        try:
            record_root = system.control_loop.record_root if system.control_loop else Path("records")
            active_id = None
            if system.control_loop and system.control_loop.recorder and system.control_loop.recorder.is_running():
                active_id = system.control_loop.recorder.session_id
            return list_recorded_sessions(record_root, active_id)
        except Exception as e:
            logger.error(f"Error listing sessions: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.get("/api/config", dependencies=[Depends(auth_dependency)])
    async def get_config():
        """Read public configuration settings."""
        try:
            from telegrip.config import get_config_data
            return get_config_data()
        except Exception as e:
            logger.error(f"Error getting config: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    # ------------------ Protected Endpoints ------------------

    @app.get("/api/sessions/{session_id}", dependencies=[Depends(auth_dependency)])
    async def get_single_session(session_id: str):
        """Retrieve full trajectory dataset contents for a recorded session (Protected)."""
        try:
            record_root = system.control_loop.record_root if system.control_loop else Path("records")
            data = load_session_data(record_root, session_id)
            if data is None:
                return JSONResponse(status_code=404, content={"error": f"Session {session_id} not found"})
            return data
        except Exception as e:
            logger.error(f"Error fetching session {session_id}: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.post("/api/config", dependencies=[Depends(auth_dependency)])
    async def post_config(request: Request):
        try:
            from telegrip.config import update_config_data
            data = await request.json()
            success = update_config_data(data)
            if success:
                return {"success": True, "message": "Configuration updated"}
            return JSONResponse(status_code=500, content={"error": "Failed to save configuration"})
        except Exception as e:
            logger.error(f"Error updating config: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.post("/api/keyboard", dependencies=[Depends(auth_dependency)])
    async def post_keyboard(request: Request):
        try:
            data = await request.json()
            action = data.get("action")
            if action in ["enable", "disable"]:
                system.add_control_command(f"{action}_keyboard")
                return {"success": True, "action": action}
            return JSONResponse(status_code=400, content={"error": f"Invalid action: {action}"})
        except Exception as e:
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.post("/api/robot", dependencies=[Depends(auth_dependency)])
    async def post_robot(request: Request):
        try:
            data = await request.json()
            action = data.get("action")
            if action in ["connect", "disconnect"]:
                system.add_control_command(f"robot_{action}")
                return {"success": True, "action": action}
            return JSONResponse(status_code=400, content={"error": f"Invalid action: {action}"})
        except Exception as e:
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.post("/api/keypress", dependencies=[Depends(auth_dependency)])
    async def post_keypress(request: Request):
        try:
            data = await request.json()
            key = data.get("key")
            action = data.get("action")
            if key and action in ["press", "release"]:
                command = {"action": "web_keypress", "key": key, "event": action}
                system.add_keypress_command(command)
                return {"success": True, "key": key, "action": action}
            return JSONResponse(status_code=400, content={"error": "Invalid key or action"})
        except Exception as e:
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.post("/api/task", dependencies=[Depends(auth_dependency)])
    async def post_task(request: Request):
        try:
            data = await request.json()
            action = data.get("action")
            if action == "reset":
                system.add_control_command("task_reset")
                return {"success": True, "action": action}
            elif action == "status":
                task = system.control_loop.task if system.control_loop else None
                if task is None:
                    return JSONResponse(status_code=404, content={"error": "No task enabled"})
                return {"success": True, "task": task.get_state(), "objects": task.get_object_states()}
            return JSONResponse(status_code=400, content={"error": f"Invalid action: {action}"})
        except Exception as e:
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.post("/api/restart", dependencies=[Depends(auth_dependency)])
    async def post_restart():
        try:
            system.restart()
            return {"success": True, "message": "Restarting system"}
        except Exception as e:
            return JSONResponse(status_code=500, content={"error": str(e)})

    # ------------------ WebSocket Endpoint (Protected in Hardware/Token Mode) ------------------

    @app.websocket("/ws")
    @app.websocket("/ws/")
    async def websocket_endpoint(websocket: WebSocket):
        headers = dict(websocket.headers)
        query_params = dict(websocket.query_params)
        client_host = websocket.client.host if websocket.client else None

        authenticated, accepted_subprotocol, ticket_payload = authenticate_websocket_connection(
            headers, query_params, client_host=client_host, robot_enabled=robot_enabled
        )

        if not authenticated:
            logger.warning(
                f"Rejected unauthorized WebSocket connection from {client_host or 'unknown'}"
            )
            # 4401: Unauthorized (custom WebSocket close code within 4000-4999 application range)
            await websocket.close(code=4401, reason="Unauthorized: Valid ticket or token required")
            return

        if system.vr_server and hasattr(system.vr_server, "fastapi_handler"):
            await system.vr_server.fastapi_handler(websocket, subprotocol=accepted_subprotocol)
        elif system.vr_server:
            await _handle_fastapi_websocket(system, websocket, subprotocol=accepted_subprotocol)
        else:
            await websocket.close(code=1011)

    # ------------------ Static Files Fallback ------------------
    web_ui_path = Path("web-ui")
    if not web_ui_path.exists():
        vendor_web_ui = Path("vendor/telegrip/web-ui")
        if vendor_web_ui.exists():
            web_ui_path = vendor_web_ui

    if web_ui_path.exists():
        class SafeStaticFiles(StaticFiles):
            async def __call__(self, scope, receive, send):
                if scope["type"] == "websocket":
                    await send({"type": "websocket.close", "code": 1000})
                    return
                await super().__call__(scope, receive, send)

        app.mount("/", SafeStaticFiles(directory=str(web_ui_path), html=True), name="static")

    return app


class UnifiedServer:
    """FastAPI / Uvicorn server runner."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8500, log_level: str = "warning"):
        self.host = host
        self.port = port
        self.log_level = log_level
        self.server: Optional[uvicorn.Server] = None
        self.system_ref: Optional["EmbodexTeleopSystem"] = None

    def set_system_ref(self, system_ref: "EmbodexTeleopSystem"):
        self.system_ref = system_ref

    async def start(self):
        app = create_app(self.system_ref)
        config = uvicorn.Config(
            app,
            host=self.host,
            port=self.port,
            log_level=self.log_level.lower(),
        )
        self.server = uvicorn.Server(config)
        asyncio.create_task(self.server.serve())
        logger.info(f"Embodex server listening on {self.host}:{self.port}")

    async def stop(self):
        if self.server:
            self.server.should_exit = True
            logger.info("Embodex server stopped")
