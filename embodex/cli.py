"""
Embodex Teleoperation Backend CLI.
Initializes and coordinates the WebXR server, robot control loop, trajectory recording,
challenge task simulation, and REST API.
"""

import argparse
import asyncio
import logging
import os
from pathlib import Path
import queue
import signal
import sys
import threading
from typing import Optional

from telegrip.config import TelegripConfig, get_config_data
from telegrip.inputs.vr_ws_server import VRWebSocketServer
from telegrip.inputs.web_keyboard import WebKeyboardHandler

from .adapters.telegrip_adapter import EmbodexControlLoop
from .api.server import UnifiedServer
from .recording.recorder import TeleopRecorder
from .recording.utils import get_next_session_dir
from .ros2.bridge import create_ros2_bridge_if_enabled

logger = logging.getLogger("embodex")


class EmbodexTeleopSystem:
    """Main Embodex teleoperation system coordinating all services."""

    def __init__(self, config: TelegripConfig, record_dir: str = "records", record_vr_only: bool = False):
        self.config = config
        self.record_root = Path(record_dir)
        self.record_vr_only = record_vr_only

        # Command queues
        self.command_queue = asyncio.Queue()
        self.control_commands_queue = queue.Queue(maxsize=10)

        # Components
        port = getattr(config, "port", int(os.environ.get("PORT", 8500)))
        self.server = UnifiedServer(host=config.host_ip, port=port, log_level=config.log_level)
        self.vr_server = VRWebSocketServer(self.command_queue, config)
        self.web_keyboard_handler = WebKeyboardHandler(self.command_queue, config)
        self.control_loop = EmbodexControlLoop(
            command_queue=self.command_queue,
            config=config,
            control_commands_queue=self.control_commands_queue,
            record_dir=self.record_root,
            record_vr_only=self.record_vr_only,
            task_config=getattr(config, "task", {}),
        )

        self.server.set_system_ref(self)
        self.control_loop.web_keyboard_handler = self.web_keyboard_handler
        self.control_loop.vr_server = self.vr_server

        # Optional ROS2 bridge
        self.ros2_bridge = create_ros2_bridge_if_enabled(
            getattr(config, "enable_ros2", False),
            node_name=getattr(config, "ros2_node_name", "embodex_teleop_bridge"),
            base_frame=getattr(config, "ros2_base_frame", "teleop_base"),
            topic_prefix=getattr(config, "ros2_topic_prefix", "embodex"),
        )
        if self.ros2_bridge:
            self.control_loop.ros2_bridge = self.ros2_bridge

        self.web_keyboard_handler.disconnect_callback = lambda: self.add_control_command("robot_disconnect")
        self.tasks = []
        self.is_running = False
        self.main_loop = None

    def add_control_command(self, action: str):
        try:
            self.control_commands_queue.put_nowait({"action": action})
        except queue.Full:
            logger.warning(f"Control command queue full, dropped: {action}")

    def add_keypress_command(self, command: dict):
        try:
            self.control_commands_queue.put_nowait(command)
        except queue.Full:
            logger.warning(f"Keypress command queue full, dropped: {command}")

    async def process_control_commands(self):
        try:
            while True:
                try:
                    command = self.control_commands_queue.get_nowait()
                except queue.Empty:
                    break
                if self.control_loop:
                    await self.control_loop._handle_command(command)
        except Exception as e:
            logger.error(f"Error processing control commands: {e}")

    def restart(self):
        def do_restart():
            try:
                if self.main_loop and not self.main_loop.is_closed():
                    future = asyncio.run_coroutine_threadsafe(self._soft_restart_sequence(), self.main_loop)
                    future.result(timeout=30.0)
            except Exception as e:
                logger.error(f"Error during restart: {e}")

        threading.Thread(target=do_restart, daemon=True).start()

    async def _soft_restart_sequence(self):
        for task in self.tasks:
            task.cancel()
        if self.tasks:
            await asyncio.wait_for(asyncio.gather(*self.tasks, return_exceptions=True), timeout=5.0)
        await self.control_loop.stop()
        if self.ros2_bridge:
            self.ros2_bridge.stop()
        await self.server.stop()
        await self.start()

    async def start(self):
        self.main_loop = asyncio.get_running_loop()
        self.is_running = True
        logger.info("Starting Embodex Teleoperation System...")

        if not self.control_loop.setup():
            if self.config.enable_robot:
                logger.error("Control loop setup failed")
                return

        await self.server.start()

        # Launch background tasks
        control_loop_task = asyncio.create_task(self.control_loop.start())
        command_processor_task = asyncio.create_task(self._command_processor())

        self.tasks = [control_loop_task, command_processor_task]
        await asyncio.gather(*self.tasks)

    async def _command_processor(self):
        while self.is_running:
            await self.process_control_commands()
            await asyncio.sleep(0.01)

    async def stop(self):
        self.is_running = False
        for task in self.tasks:
            task.cancel()
        await self.control_loop.stop()
        if self.ros2_bridge:
            self.ros2_bridge.stop()
        await self.server.stop()
        logger.info("Embodex Teleoperation System stopped")


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Embodex Teleoperation Backend — Open-source XR robot teleop & embodied motion data."
    )
    parser.add_argument("--no-robot", action="store_true", help="Disable physical robot hardware (simulation/digital twin only)")
    parser.add_argument("--no-viz", action="store_true", help="Disable PyBullet GUI (run headless)")
    parser.add_argument("--no-vr", action="store_true", help="Disable VR controller WebSocket input")
    parser.add_argument("--no-keyboard", action="store_true", help="Disable web keyboard control")
    parser.add_argument("--digital-twin", action="store_true", default=True, help="Enable digital twin broadcasting")
    parser.add_argument("--record", action="store_true", help="Automatically start recording session on launch")
    parser.add_argument("--record-dir", type=str, default="records", help="Root directory for dataset session recordings")
    parser.add_argument("--record-vr-only", action="store_true", help="Record only raw VR packets (no robot joints)")
    parser.add_argument("--port", type=int, default=int(os.environ.get("PORT", 8500)), help="Port for REST API and WebSockets")
    parser.add_argument(
        "--host",
        type=str,
        default=None,
        help="Host interface to bind (defaults to 127.0.0.1 in local mode, 0.0.0.0 otherwise)",
    )
    parser.add_argument("--left-port", type=str, help="Serial port for left follower arm")
    parser.add_argument("--right-port", type=str, help="Serial port for right follower arm")
    parser.add_argument("--ros2", action="store_true", help="Enable ROS2 transform broadcaster bridge")
    parser.add_argument("--autoconnect", action="store_true", help="Automatically connect robot arms on launch")
    parser.add_argument("--log-level", choices=["debug", "info", "warning", "error"], default="warning", help="Logging verbosity")
    return parser.parse_args()


def build_config(args) -> TelegripConfig:
    from .api.security import get_security_mode, SECURITY_MODE_LOCAL

    config = TelegripConfig()
    config.enable_robot = not args.no_robot
    config.enable_pybullet_gui = not args.no_viz
    config.enable_vr = not args.no_vr
    config.enable_keyboard = not args.no_keyboard
    config.enable_ros2 = args.ros2 or getattr(config, "enable_ros2", False)
    config.autoconnect = args.autoconnect
    config.digital_twin_enabled = args.digital_twin
    config.log_level = args.log_level
    config.port = args.port

    sec_mode = get_security_mode(robot_enabled=config.enable_robot)
    if args.host:
        config.host_ip = args.host
    elif sec_mode == SECURITY_MODE_LOCAL:
        config.host_ip = "127.0.0.1"
    else:
        config.host_ip = "0.0.0.0"

    config_data = get_config_data()
    robot_cfg = config_data.get("robot", {})
    if args.left_port or args.right_port:
        config.follower_ports = {
            "left": args.left_port or robot_cfg.get("left_arm", {}).get("port", "/dev/ttyACM0"),
            "right": args.right_port or robot_cfg.get("right_arm", {}).get("port", "/dev/ttyACM1"),
        }

    # Pass task configuration if present
    task_cfg = dict(config_data.get("task", {}) or {})
    config.task = task_cfg
    return config


async def async_main():
    args = parse_arguments()
    log_level = getattr(logging, args.log_level.upper())

    if log_level > logging.INFO:
        os.environ["PYBULLET_SUPPRESS_CONSOLE_OUTPUT"] = "1"
        os.environ["PYBULLET_SUPPRESS_WARNINGS"] = "1"

    logging.basicConfig(
        level=log_level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s" if log_level <= logging.INFO else "%(message)s",
    )

    config = build_config(args)

    print("=" * 60)
    print("🤖 Embodex Teleoperation Platform")
    print(f"📡 Backend & WebXR server: http://{config.host_ip}:{config.port}")
    print(f"🦾 Robot hardware: {'Enabled' if config.enable_robot else 'Disabled (Digital Twin / Sim Mode)'}")
    print(f"🥽 WebXR Teleoperation: {'Enabled' if config.enable_vr else 'Disabled'}")
    print(f"📹 Auto-record: {'Active' if args.record else 'Standby'}")
    print("=" * 60)

    system = EmbodexTeleopSystem(
        config=config,
        record_dir=args.record_dir,
        record_vr_only=args.record_vr_only,
    )

    if args.record:
        rec_dir = get_next_session_dir(system.record_root)
        fps = 1.0 / config.send_interval if config.send_interval > 0 else 20.0
        system.control_loop.recorder = TeleopRecorder(
            record_dir=rec_dir,
            fps=fps,
            vr_only=args.record_vr_only,
            left_enabled=getattr(config, "left_arm_enabled", True),
            right_enabled=getattr(config, "right_arm_enabled", True),
        )
        system.control_loop.recorder.start()
        print(f"📼 Recording started -> {rec_dir}")

    loop = asyncio.get_running_loop()

    def handle_signal():
        print("\n🛑 Stopping Embodex Teleop...")
        asyncio.create_task(system.stop())

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, handle_signal)
        except NotImplementedError:
            pass

    try:
        await system.start()
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass


def main():
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
