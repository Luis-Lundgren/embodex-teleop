"""
Embodex FastAPI Server.
Exposes REST endpoints, WebXR WebSocket proxy, static files, and health checks.
"""

import asyncio
import json
import logging
from pathlib import Path
from typing import Optional, TYPE_CHECKING

from fastapi import FastAPI, Request, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

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
                "record_dir": str(json_file.parent),
                "createdAt": created_at,
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


def create_app(system: "EmbodexTeleopSystem") -> FastAPI:
    """Create a FastAPI application bound to an EmbodexTeleopSystem instance."""
    app = FastAPI(title="Embodex Teleoperation API", version="0.1.0")

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

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
                vr_connected = len(system.vr_server.clients) > 0

            return {
                **control_status,
                "keyboardEnabled": keyboard_enabled,
                "robotEngaged": robot_engaged,
                "vrConnected": vr_connected,
            }
        except Exception as e:
            logger.error(f"Error handling status request: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.get("/api/sessions")
    async def get_sessions():
        try:
            record_root = system.control_loop.record_root if system.control_loop else Path("records")
            active_id = None
            if system.control_loop and system.control_loop.recorder and system.control_loop.recorder.is_running():
                active_id = system.control_loop.recorder.session_id
            return list_recorded_sessions(record_root, active_id)
        except Exception as e:
            logger.error(f"Error listing sessions: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.get("/api/sessions/{session_id}")
    async def get_single_session(session_id: str):
        try:
            record_root = system.control_loop.record_root if system.control_loop else Path("records")
            data = load_session_data(record_root, session_id)
            if data is None:
                return JSONResponse(status_code=404, content={"error": f"Session {session_id} not found"})
            return data
        except Exception as e:
            logger.error(f"Error fetching session {session_id}: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.get("/api/config")
    async def get_config():
        try:
            from telegrip.config import get_config_data
            return get_config_data()
        except Exception as e:
            logger.error(f"Error getting config: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.post("/api/config")
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

    @app.post("/api/keyboard")
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

    @app.post("/api/robot")
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

    @app.post("/api/keypress")
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

    @app.post("/api/task")
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

    @app.post("/api/restart")
    async def post_restart():
        try:
            system.restart()
            return {"success": True, "message": "Restarting system"}
        except Exception as e:
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.websocket("/ws")
    @app.websocket("/ws/")
    async def websocket_endpoint(websocket: WebSocket):
        if system.vr_server and hasattr(system.vr_server, "fastapi_handler"):
            await system.vr_server.fastapi_handler(websocket)
        else:
            await websocket.close(code=1011)

    # Static fallback
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
