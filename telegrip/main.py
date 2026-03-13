"""
Main entry point for the unified teleoperation system.
Coordinates HTTPS server, WebSocket server, robot interface, and input providers.
"""

import asyncio
import argparse
import logging
import signal
import sys
import os
import fastapi
from fastapi import FastAPI, WebSocket, Request, BackgroundTasks
from fastapi.responses import JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import socket
import json
import urllib.parse
import time
import contextlib
from typing import Optional
import queue  # Add regular queue for thread-safe communication
import threading
from pathlib import Path
import weakref


def get_local_ip():
    """Get the local IP address of this machine."""
    try:
        # Connect to a remote address to determine the local IP
        # This doesn't actually send any data
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        try:
            # Fallback: get hostname IP
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            # Final fallback
            return "localhost"


@contextlib.contextmanager
def suppress_stdout_stderr():
    """Context manager to suppress stdout and stderr output at the file descriptor level."""
    # Save original file descriptors
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()
    
    # Save original file descriptors
    saved_stdout_fd = os.dup(stdout_fd)
    saved_stderr_fd = os.dup(stderr_fd)
    
    try:
        # Open devnull
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        
        # Redirect stdout and stderr to devnull
        os.dup2(devnull_fd, stdout_fd)
        os.dup2(devnull_fd, stderr_fd)
        
        yield
        
    finally:
        # Restore original file descriptors
        os.dup2(saved_stdout_fd, stdout_fd)
        os.dup2(saved_stderr_fd, stderr_fd)
        
        # Close saved file descriptors
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


# Import telegrip modules after function definition
from .config import TelegripConfig, get_config_data, update_config_data
from .control_loop import ControlLoop
from .inputs.vr_ws_server import VRWebSocketServer
from .inputs.web_keyboard import WebKeyboardHandler
from .inputs.base import ControlGoal
from .ros2_bridge import create_ros2_bridge_if_enabled
from .utils import get_next_session_dir

# Logger will be configured in main() based on command line arguments
logger = logging.getLogger(__name__)


def create_app(system: 'TelegripSystem'):
    """Create a FastAPI application for the teleoperation system."""
    app = FastAPI(title="TeleGrip API")

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    @app.get("/api/status")
    async def get_status():
        try:
            # Get status from control loop
            control_status = system.control_loop.status if system.control_loop else {}
            
            # Get keyboard status
            keyboard_enabled = False
            if system.web_keyboard_handler and hasattr(system.web_keyboard_handler, 'is_enabled'):
                keyboard_enabled = system.web_keyboard_handler.is_enabled
            
            # Get robot engagement status
            robot_engaged = False
            if system.control_loop and system.control_loop.robot_interface:
                robot_engaged = system.control_loop.robot_interface.is_engaged
            
            # Get VR connection status
            vr_connected = False
            if system.vr_server and system.vr_server.is_running:
                vr_connected = len(system.vr_server.clients) > 0
            
            status = {
                **control_status,
                "keyboardEnabled": keyboard_enabled,
                "robotEngaged": robot_engaged,
                "vrConnected": vr_connected
            }
            return status
        except Exception as e:
            logger.error(f"Error handling status request: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.get("/api/config")
    async def get_config():
        try:
            return get_config_data()
        except Exception as e:
            logger.error(f"Error handling config get request: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.post("/api/config")
    async def post_config(request: Request):
        try:
            data = await request.json()
            success = update_config_data(data)
            if success:
                logger.info("Configuration updated successfully")
                return {"success": True, "message": "Configuration updated successfully"}
            else:
                return JSONResponse(status_code=500, content={"error": "Failed to save configuration"})
        except Exception as e:
            logger.error(f"Error handling config post request: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.post("/api/keyboard")
    async def post_keyboard(request: Request):
        try:
            data = await request.json()
            action = data.get('action')
            if action in ['enable', 'disable']:
                command_name = f"{action}_keyboard"
                logger.info(f"🎮 Adding command to queue: {command_name}")
                system.add_control_command(command_name)
                return {"success": True, "action": action}
            else:
                return JSONResponse(status_code=400, content={"error": f"Invalid action: {action}"})
        except Exception as e:
            logger.error(f"Error handling keyboard request: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.post("/api/robot")
    async def post_robot(request: Request):
        try:
            data = await request.json()
            action = data.get('action')
            logger.info(f"🔌 Received robot action: {action}")
            if action in ['connect', 'disconnect']:
                command_name = f"robot_{action}"
                logger.info(f"🔌 Adding command to queue: {command_name}")
                system.add_control_command(command_name)
                return {"success": True, "action": action}
            else:
                return JSONResponse(status_code=400, content={"error": f"Invalid action: {action}"})
        except Exception as e:
            logger.error(f"Error handling robot request: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.post("/api/keypress")
    async def post_keypress(request: Request):
        try:
            data = await request.json()
            key = data.get('key')
            action = data.get('action')
            if key and action in ['press', 'release']:
                command = {
                    "action": "web_keypress",
                    "key": key,
                    "event": action
                }
                logger.info(f"🎮 Adding keypress command to queue: {key}_{action}")
                system.add_keypress_command(command)
                return {"success": True, "key": key, "action": action}
            else:
                return JSONResponse(status_code=400, content={"error": f"Invalid key or action: {key}, {action}"})
        except Exception as e:
            logger.error(f"Error handling keypress request: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.post("/api/restart")
    async def post_restart():
        try:
            logger.info("Restarting teleoperation system...")
            system.restart()
            return {"success": True, "message": "Teleoperation system restarted"}
        except Exception as e:
            logger.error(f"Error handling restart request: {e}")
            return JSONResponse(status_code=500, content={"error": str(e)})

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        if system.vr_server:
            await system.vr_server.fastapi_handler(websocket)
        else:
            await websocket.close(code=1011)

    # Health check
    @app.get("/health")
    async def health():
        return {"status": "ok"}

    # Serve static files from web-ui directory
    from .utils import get_absolute_path
    web_ui_path = get_absolute_path('web-ui')
    if web_ui_path.exists():
        app.mount("/", StaticFiles(directory=str(web_ui_path), html=True), name="static")
    
    return app


class UnifiedServer:
    """FastAPI/Uvicorn server for the teleoperation API and WebSocket."""
    
    def __init__(self, config: TelegripConfig):
        self.config = config
        self.server = None
        self.system_ref = None
    
    def set_system_ref(self, system_ref):
        """Set reference to the main teleoperation system."""
        self.system_ref = system_ref
    
    async def start(self):
        """Start the FastAPI server."""
        try:
            app = create_app(self.system_ref)
            config = uvicorn.Config(
                app, 
                host=self.config.host_ip, 
                port=self.config.port, 
                log_level=self.config.log_level.lower()
            )
            self.server = uvicorn.Server(config)
            
            # Start uvicorn in a separate thread because its run() is blocking
            # or we can use serve() if we are in an async context.
            # However, we want it to run as part of the system's tasks.
            # So we'll return a task that runs the server.
            asyncio.create_task(self.server.serve())
            
            logger.info(f"Server running on {self.config.host_ip}:{self.config.port}")
            
        except Exception as e:
            logger.error(f"Failed to start Unified server: {e}")
            raise
    
    async def stop(self):
        """Stop the server."""
        if self.server:
            self.server.should_exit = True
            logger.info("Unified server stopped")


class TelegripSystem:
    """Main teleoperation system that coordinates all components."""
    
    def __init__(self, config: TelegripConfig):
        self.config = config
        
        # Command queues
        self.command_queue = asyncio.Queue()
        self.control_commands_queue = queue.Queue(maxsize=10)  # Thread-safe queue
        
        # Components
        self.server = UnifiedServer(config)
        self.vr_server = VRWebSocketServer(self.command_queue, config)
        self.web_keyboard_handler = WebKeyboardHandler(self.command_queue, config)
        self.control_loop = ControlLoop(self.command_queue, config, self.control_commands_queue)

        # Set system reference for API calls
        self.server.set_system_ref(self)

        # Set up cross-references
        self.control_loop.web_keyboard_handler = self.web_keyboard_handler
        self.control_loop.vr_server = self.vr_server

        # Optional ROS2 bridge (stream VR/teleop to ROS2 as canonical transform broker)
        self.ros2_bridge = create_ros2_bridge_if_enabled(
            self.config.enable_ros2,
            node_name=self.config.ros2_node_name,
            base_frame=self.config.ros2_base_frame,
            topic_prefix=self.config.ros2_topic_prefix,
        )
        if self.ros2_bridge:
            self.control_loop.ros2_bridge = self.ros2_bridge
            logger.info("ROS2 bridge enabled: VR/teleop streaming to TF2 and topics")

        # Optional recorder (VR raw + robot goal, LeRobot/ROS2 comparable)
        self.recorder = None

        # Set up disconnect callback for ESC key
        self.web_keyboard_handler.disconnect_callback = lambda: self.add_control_command("robot_disconnect")
        
        # Tasks
        self.tasks = []
        self.is_running = False
        self.main_loop = None  # Will be set when the system starts
    
    def add_control_command(self, action: str):
        """Add a control command to the queue for processing."""
        try:
            command = {"action": action}
            logger.info(f"🔌 Queueing control command: {command}")
            self.control_commands_queue.put_nowait(command)
            logger.info(f"🔌 Command queued successfully")
        except queue.Full:
            logger.warning(f"Control commands queue is full, dropping command: {action}")
        except Exception as e:
            logger.error(f"🔌 Error queuing command: {e}")
    
    def add_keypress_command(self, command: dict):
        """Add a keypress command to the queue for processing."""
        try:
            logger.info(f"🎮 Queueing keypress command: {command}")
            self.control_commands_queue.put_nowait(command)
            logger.info(f"🎮 Keypress command queued successfully")
        except queue.Full:
            logger.warning(f"Control commands queue is full, dropping keypress command: {command}")
        except Exception as e:
            logger.error(f"🎮 Error queuing keypress command: {e}")
    
    async def process_control_commands(self):
        """Process control commands from the thread-safe queue."""
        try:
            # Get all available commands from the thread-safe queue
            commands_to_process = []
            while True:
                try:
                    command = self.control_commands_queue.get_nowait()
                    commands_to_process.append(command)
                except queue.Empty:
                    break
            
            # Process each command
            for command in commands_to_process:
                if self.control_loop:
                    await self.control_loop._handle_command(command)
                    
        except Exception as e:
            logger.error(f"Error processing control commands: {e}")
    
    def restart(self):
        """Restart the teleoperation system."""
        def do_restart():
            try:
                logger.info("Initiating system restart...")
                # Use the stored main event loop reference to schedule the soft restart
                if self.main_loop and not self.main_loop.is_closed():
                    future = asyncio.run_coroutine_threadsafe(self._soft_restart_sequence(), self.main_loop)
                    # Wait for restart to complete
                    future.result(timeout=30.0)
                else:
                    logger.error("Main event loop not available for restart")
            except Exception as e:
                logger.error(f"Error during restart: {e}")
        
        # Run restart in a separate thread to avoid blocking the HTTP response
        restart_thread = threading.Thread(target=do_restart, daemon=True)
        restart_thread.start()
    
    async def _soft_restart_sequence(self):
        """Perform a soft restart by reinitializing components without exiting the process."""
        try:
            logger.info("Starting soft restart sequence...")
            
            # Wait a moment to let the HTTP response be sent
            await asyncio.sleep(1)
            
            # Cancel all tasks
            for task in self.tasks:
                task.cancel()
            
            # Wait for tasks to complete with timeout
            if self.tasks:
                try:
                    await asyncio.wait_for(
                        asyncio.gather(*self.tasks, return_exceptions=True), 
                        timeout=5.0
                    )
                except asyncio.TimeoutError:
                    logger.warning("Some tasks did not complete within timeout")
            
            # Stop components in reverse order
            await self.control_loop.stop()
            await self.web_keyboard_handler.stop()
            await self.vr_server.stop()
            # Don't stop Unified server - keep it running for the UI if possible, 
            # though FastAPI might need restart if system state changed significantly.
            # For now, let's keep it running.

            # Wait a moment for cleanup
            await asyncio.sleep(1)

            # Reload configuration from file but preserve command-line overrides
            from .config import get_config_data
            file_config = get_config_data()
            logger.info("Configuration reloaded from file")

            # Keep the existing configuration object to preserve command-line arguments
            # Just update specific values that might have changed in the config file

            # Recreate components with existing configuration
            self.command_queue = asyncio.Queue()
            self.control_commands_queue = queue.Queue(maxsize=10)

            # Create new components
            self.vr_server = VRWebSocketServer(self.command_queue, self.config)
            self.web_keyboard_handler = WebKeyboardHandler(self.command_queue, self.config)
            self.control_loop = ControlLoop(self.command_queue, self.config, self.control_commands_queue)

            # Recreate ROS2 bridge if enabled
            self.ros2_bridge = create_ros2_bridge_if_enabled(
                self.config.enable_ros2,
                node_name=self.config.ros2_node_name,
                base_frame=self.config.ros2_base_frame,
                topic_prefix=self.config.ros2_topic_prefix,
            )
            if self.ros2_bridge:
                self.control_loop.ros2_bridge = self.ros2_bridge
                self.ros2_bridge.start_spin_thread()

            # Set up cross-references
            self.control_loop.web_keyboard_handler = self.web_keyboard_handler

            # Set up disconnect callback for ESC key
            self.web_keyboard_handler.disconnect_callback = lambda: self.add_control_command("robot_disconnect")

            # Clear old tasks
            self.tasks = []

            # Start VR WebSocket server
            await self.vr_server.start()

            # Start web keyboard handler
            await self.web_keyboard_handler.start()

            # Start control loop
            control_task = asyncio.create_task(self.control_loop.start())
            self.tasks.append(control_task)

            # Start control command processor
            command_processor_task = asyncio.create_task(self._run_command_processor())
            self.tasks.append(command_processor_task)

            logger.info("System restart completed successfully")
            
            # Auto-connect to robot if requested (preserve autoconnect behavior after restart)
            if self.config.autoconnect and self.config.enable_robot:
                logger.info("🔌 Auto-connecting to robot motors after restart...")
                await asyncio.sleep(0.5)  # Brief delay to let components settle
                self.add_control_command("robot_connect")
            
        except Exception as e:
            logger.error(f"Error during soft restart sequence: {e}")
            raise
    
    async def start(self):
        """Start all system components."""
        try:
            self.is_running = True
            
            # Store reference to the main event loop for restart functionality
            self.main_loop = asyncio.get_event_loop()
            
            # Start Unified server
            await self.server.start()
            
            # Start VR WebSocket logic
            await self.vr_server.start()

            # Start web keyboard handler
            await self.web_keyboard_handler.start()

            # Start ROS2 bridge spin thread if enabled
            if self.ros2_bridge:
                self.ros2_bridge.start_spin_thread()

            # Start control loop
            control_task = asyncio.create_task(self.control_loop.start())
            self.tasks.append(control_task)

            # Start control command processor
            command_processor_task = asyncio.create_task(self._run_command_processor())
            self.tasks.append(command_processor_task)

            logger.info("All system components started successfully")

            # Start recorder if set (e.g. via --record)
            if self.recorder and self.control_loop and self.control_loop.record_autostart:
                self.recorder.start()

            # Auto-connect to robot if requested
            if self.config.autoconnect and self.config.enable_robot:
                logger.info("🔌 Auto-connecting to robot motors...")
                await asyncio.sleep(0.5)  # Brief delay to let components settle
                self.add_control_command("robot_connect")
            
            # Main loop that handles restarts
            while self.is_running:
                try:
                    # Wait for tasks to complete
                    await asyncio.gather(*self.tasks)
                    # If we get here, all tasks completed normally (shouldn't happen in normal operation)
                    break
                except asyncio.CancelledError:
                    # Tasks were cancelled - check if it's due to restart
                    if self.is_running:
                        # System is restarting, wait for restart to complete
                        await asyncio.sleep(1)
                        # Continue the loop to wait for new tasks
                        continue
                    else:
                        # Normal shutdown
                        break
                except Exception as e:
                    logger.error(f"Error in main task loop: {e}")
                    break
            
        except OSError as e:
            if e.errno == 98:  # Address already in use
                logger.error(f"Error starting teleoperation system: {e}")
                logger.error(f"To find and kill the process using this port, run:")
                logger.error(f"  kill -9 $(lsof -t -i:{self.config.port})")
            else:
                logger.error(f"Error starting teleoperation system: {e}")
            await self.stop()
            raise
        except Exception as e:
            logger.error(f"Error starting teleoperation system: {e}")
            await self.stop()
            raise
    
    async def _run_command_processor(self):
        """Run the control command processor loop."""
        while self.is_running:
            await self.process_control_commands()
            await asyncio.sleep(0.05)  # Check for commands every 50ms
    
    async def stop(self):
        """Stop all system components."""
        logger.info("Shutting down teleoperation system...")
        self.is_running = False

        # Stop VR server first to close websocket connections (unblocks any waiting handlers)
        try:
            await asyncio.wait_for(self.vr_server.stop(), timeout=2.0)
        except asyncio.TimeoutError:
            logger.warning("VR server stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping VR server: {e}")

        # Cancel all tasks
        for task in self.tasks:
            task.cancel()

        # Wait for tasks to complete with timeout
        if self.tasks:
            try:
                await asyncio.wait_for(
                    asyncio.gather(*self.tasks, return_exceptions=True),
                    timeout=2.0
                )
            except asyncio.TimeoutError:
                logger.warning("Some tasks did not complete within timeout")

        # Stop remaining components
        try:
            await asyncio.wait_for(self.control_loop.stop(), timeout=3.0)
        except asyncio.TimeoutError:
            logger.warning("Control loop stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping control loop: {e}")

        try:
            await asyncio.wait_for(self.web_keyboard_handler.stop(), timeout=1.0)
        except asyncio.TimeoutError:
            logger.warning("Web keyboard handler stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping web keyboard handler: {e}")

        # Flush and stop recorder
        if self.recorder:
            try:
                self.recorder.stop()
            except Exception as e:
                logger.warning(f"Error stopping recorder: {e}")

        try:
            await asyncio.wait_for(self.server.stop(), timeout=2.0)
        except asyncio.TimeoutError:
            logger.warning("Unified server stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping Unified server: {e}")

        logger.info("Teleoperation system shutdown complete")


def create_signal_handler(system: 'TelegripSystem', loop: asyncio.AbstractEventLoop):
    """Create a signal handler that properly stops the system."""
    def signal_handler(signum, frame):
        """Handle shutdown signals."""
        logger.info(f"Received signal {signum}")
        system.is_running = False
        # Cancel all tasks from the event loop
        for task in system.tasks:
            loop.call_soon_threadsafe(task.cancel)
        # Raise SystemExit to break out of blocking operations
        raise SystemExit(0)
    return signal_handler


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Unified SO100 Robot Teleoperation System")
    
    # Control flags
    parser.add_argument("--no-robot", action="store_true", help="Disable robot connection (visualization only)")
    parser.add_argument("--no-sim", action="store_true", help="Disable PyBullet simulation and inverse kinematics")
    parser.add_argument("--no-viz", action="store_true", help="Disable PyBullet visualization (headless mode)")
    parser.add_argument("--no-vr", action="store_true", help="Disable VR WebSocket server")
    parser.add_argument("--no-keyboard", action="store_true", help="Disable keyboard input")
    parser.add_argument("--no-server", action="store_true", help="Disable Unified server")
    parser.add_argument("--ros2", action="store_true", help="Stream VR/teleop commands to ROS2 (TF2 + topics) as canonical transform broker")
    parser.add_argument("--autoconnect", action="store_true", help="Automatically connect to robot motors on startup")
    parser.add_argument("--record", action="store_true", help="Record VR inputs and robot goals (LeRobot/ROS2 comparable format)")
    parser.add_argument("--record-dir", default="records", help="Directory for recording output (default: records)")
    parser.add_argument("--record-vr-only", action="store_true", help="Only record VR raw input (no robot goal); use with --ros2 --no-robot")
    parser.add_argument("--digital-twin", action="store_true", help="Enable digital twin visualization in AR")
    parser.add_argument("--log-level", default="warning", 
                       choices=["debug", "info", "warning", "error", "critical"],
                       help="Set logging level (default: warning)")
    
    # Network settings
    parser.add_argument("--port", type=int, default=int(os.environ.get("PORT", 8000)), help="Server port")
    parser.add_argument("--host", default="0.0.0.0", help="Host IP address")
    
    # Paths
    parser.add_argument("--urdf", default="URDF/SO100/so100.urdf", help="Path to robot URDF file")
    parser.add_argument("--webapp", default="webapp", help="Path to webapp directory")
    parser.add_argument("--cert", default="cert.pem", help="Path to SSL certificate")
    parser.add_argument("--key", default="key.pem", help="Path to SSL private key")
    
    # Robot settings
    parser.add_argument("--config", default="config.yaml", help="Path to config file")
    parser.add_argument("--left-port", help="Left arm serial port (overrides config file)")
    parser.add_argument("--right-port", help="Right arm serial port (overrides config file)")
    
    return parser.parse_args()


def create_config_from_args(args) -> TelegripConfig:
    """Create configuration object from command line arguments."""
    # First load the config file
    config_data = get_config_data()
    config = TelegripConfig()
    
    # Apply command line overrides
    config.enable_robot = not args.no_robot
    config.enable_pybullet = not args.no_sim
    config.enable_pybullet_gui = config.enable_pybullet and not args.no_viz
    config.enable_vr = not args.no_vr
    config.enable_keyboard = not args.no_keyboard
    config.enable_ros2 = args.ros2 or config.enable_ros2
    config.autoconnect = args.autoconnect
    config.digital_twin_enabled = args.digital_twin
    config.log_level = args.log_level
    
    config.port = args.port
    config.host_ip = args.host
    
    config.urdf_path = args.urdf
    config.webapp_dir = args.webapp
    config.certfile = args.cert
    config.keyfile = args.key
    
    # Handle port configuration - use command line args if provided, otherwise use config file values
    if args.left_port or args.right_port:
        config.follower_ports = {
            "left": args.left_port if args.left_port else config_data["robot"]["left_arm"]["port"],
            "right": args.right_port if args.right_port else config_data["robot"]["right_arm"]["port"]
        }
    
    return config


async def main():
    """Main entry point."""
    # Parse arguments first to check for log level
    args = parse_arguments()
    
    # Setup logging based on log level
    log_level = getattr(logging, args.log_level.upper())
    
    # Suppress PyBullet's native output when not in verbose mode
    if log_level > logging.INFO:
        os.environ['PYBULLET_SUPPRESS_CONSOLE_OUTPUT'] = '1'
        os.environ['PYBULLET_SUPPRESS_WARNINGS'] = '1'
    
    if log_level <= logging.INFO:
        # Verbose mode - show detailed logging with timestamps
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
    else:
        # Quiet mode - only show warnings and errors with simple format
        logging.basicConfig(
            level=log_level,
            format='%(message)s'
        )


    config = create_config_from_args(args)

    # Ensure SSL certificates exist (generate if needed for first-time startup)
    if not config.ensure_ssl_certificates():
        logger.error("Failed to ensure SSL certificates are available")
        sys.exit(1)

    # Log configuration (only if INFO level or more verbose)
    if log_level <= logging.INFO:
        logger.info("Starting with configuration:")
        logger.info(f"  Robot: {'enabled' if config.enable_robot else 'disabled'}")
        logger.info(f"  PyBullet: {'enabled' if config.enable_pybullet else 'disabled'}")
        logger.info(f"  Headless mode: {'enabled' if not config.enable_pybullet_gui and config.enable_pybullet else 'disabled'}")
        logger.info(f"  VR: {'enabled' if config.enable_vr else 'disabled'}")
        logger.info(f"  Keyboard: {'enabled' if config.enable_keyboard else 'disabled'}")
        logger.info(f"  Auto-connect: {'enabled' if config.autoconnect else 'disabled'}")
        # Show clean startup message
        print(f"🤖 telegrip starting...")
        print(f"📱 Server running on {config.host_ip}:{config.port}")
        print(f"📱 Open the UI in your browser")
        print(f"📱 Then go to the same address on your VR headset browser")
        print(f"💡 Use --log-level info to see detailed output")
        print()
    
    # Create and start teleoperation system
    system = TelegripSystem(config)

    # Setup recording paths and configuration for ControlLoop regardless of --record flag
    system.control_loop.record_root = Path(args.record_dir)
    system.control_loop.record_vr_only = getattr(args, "record_vr_only", False)

    # Optional recording at startup (VR raw + robot goal, comparable to ROS2/LeRobot)
    if args.record:
        from .recorder import TeleopRecorder
        # Auto-increment session directory if configured
        record_dir = get_next_session_dir(system.control_loop.record_root)
        fps = 1.0 / config.send_interval if config.send_interval and config.send_interval > 0 else 20.0
        recorder = TeleopRecorder(
            record_dir=record_dir, 
            fps=fps, 
            vr_only=system.control_loop.record_vr_only,
            left_enabled=config.left_arm_enabled,
            right_enabled=config.right_arm_enabled
        )
        system.recorder = recorder
        system.vr_server.recorder = recorder
        system.control_loop.recorder = recorder
        system.control_loop.record_autostart = False
        system.control_loop.record_single_session = True
        system.control_loop.record_session_dir = record_dir
        if log_level <= logging.INFO:
            logger.info("Recording enabled: %s%s", record_dir, " (VR only)" if system.control_loop.record_vr_only else " (VR raw + robot goal, ROS2/LeRobot comparable)")
        else:
            print(f"📼 Recording to {record_dir}" + (" (VR only)" if system.control_loop.record_vr_only else ""))

    # Setup signal handlers with reference to system and event loop
    loop = asyncio.get_event_loop()
    signal_handler = create_signal_handler(system, loop)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        await system.start()
    except (KeyboardInterrupt, SystemExit):
        if log_level <= logging.INFO:
            logger.info("Received interrupt signal")
        else:
            print("\n🛑 Shutting down...")
    except asyncio.CancelledError:
        # Handle cancelled error (often from restart scenarios)
        if log_level <= logging.INFO:
            logger.info("System tasks cancelled")
    except Exception as e:
        if log_level <= logging.INFO:
            logger.error(f"System error: {e}")
        else:
            print(f"❌ Error: {e}")
    finally:
        try:
            await system.stop()
        except (asyncio.CancelledError, SystemExit):
            # Ignore cancelled/exit errors during shutdown
            pass

        # Suppress SSL transport errors during event loop cleanup
        def ignore_ssl_errors(loop, context):
            # Ignore "Bad file descriptor" and "Event loop is closed" errors during shutdown
            if 'exception' in context:
                exc = context['exception']
                if isinstance(exc, (OSError, RuntimeError)):
                    return
            # Log other errors normally
            loop.default_exception_handler(context)

        loop.set_exception_handler(ignore_ssl_errors)

        if log_level > logging.INFO:
            print("✅ Shutdown complete.")


def main_cli():
    """Console script entry point for pip-installed package."""
    try:
        asyncio.run(main())
    except (KeyboardInterrupt, SystemExit):
        print("\nShutdown complete.")
    except asyncio.CancelledError:
        # Handle cancelled error from restart scenarios
        pass
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main_cli() 
