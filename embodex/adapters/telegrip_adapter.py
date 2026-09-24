"""
Embodex TeleGrip Adapter.
Subclasses and extends TeleGrip's core ControlLoop to integrate Embodex platform capabilities:
- Trajectory & Episode recording (TeleopRecorder)
- Challenge tasks (FiberPlugTask simulation & grasp logic)
- Digital twin broadcasting to WebXR clients
- Workspace table visualization in PyBullet
"""

import asyncio
import logging
import queue
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import pybullet as p

from telegrip.config import (
    GRIPPER_INDEX,
    NUM_JOINTS,
    TelegripConfig,
    WRIST_FLEX_INDEX,
    WRIST_ROLL_INDEX,
    hardware_to_urdf_gripper_deg,
)
from telegrip.control_loop import ArmState, ControlLoop
from telegrip.inputs.base import ControlGoal, ControlMode

from ..recording.recorder import TeleopRecorder, build_robot_goal_snapshot
from ..recording.utils import get_next_session_dir
from ..tasks.fiber_plug import FiberPlugTask

logger = logging.getLogger(__name__)


def create_workspace_table(physics_client, center=(0.2, -0.15, -0.0125), dimensions=(1.2, 1.0, 0.025)):
    """
    Creates a grey workspace table matching the WebXR digital twin (1.2m x 1.0m).
    """
    width, depth, thick = dimensions
    half = [width / 2.0, depth / 2.0, thick / 2.0]
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=physics_client)
    vis = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half,
        rgbaColor=[0.55, 0.55, 0.55, 1.0],
        physicsClientId=physics_client,
    )
    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=[center[0], center[1], -thick / 2.0],
        physicsClientId=physics_client,
    )


class EmbodexControlLoop(ControlLoop):
    """
    Embodex-enhanced teleoperation control loop.
    Wraps TeleGrip's core robot and IK execution with platform features.
    """

    def __init__(
        self,
        command_queue: asyncio.Queue,
        config: TelegripConfig,
        control_commands_queue: Optional[queue.Queue] = None,
        record_dir: Optional[str | Path] = None,
        record_single_session: bool = False,
        record_vr_only: bool = False,
        task_config: Optional[Dict[str, Any]] = None,
    ):
        super().__init__(command_queue, config, control_commands_queue)

        # Recording configuration
        self.record_root = Path(record_dir) if record_dir else Path("records")
        self.record_single_session = record_single_session
        self.record_vr_only = record_vr_only
        self.record_session_dir: Optional[Path] = None
        self._recording_finalized = False
        self.recorder: Optional[TeleopRecorder] = None

        # Challenge task configuration
        self.task_config = task_config or getattr(config, "task", {}) or {}
        self.task: Optional[FiberPlugTask] = None

    def setup(self) -> bool:
        """Initialize core TeleGrip, then attach workspace table and challenge task."""
        success = super().setup()
        if not success and self.config.enable_robot:
            return False

        # Add workspace table to PyBullet world
        if self.visualizer and getattr(self.visualizer, "physics_client", None) is not None:
            try:
                create_workspace_table(self.visualizer.physics_client)
                logger.info("Created Embodex digital twin workspace table in PyBullet")
            except Exception as e:
                logger.warning(f"Could not create workspace table: {e}")

        # Attach challenge task (fiber plug) if enabled
        if self.visualizer and self.task_config.get("enabled", True):
            try:
                left_robot_id = self.visualizer.robot_ids.get("left")
                ee_index = self.visualizer.end_effector_link_indices.get("left")
                if left_robot_id is not None and ee_index is not None:
                    self.task = FiberPlugTask(
                        physics_client=self.visualizer.physics_client,
                        robot_id=left_robot_id,
                        ee_link_index=ee_index,
                        base_position=[0.2, 0, 0],
                        task_config=self.task_config,
                    )
                    if self.task.setup():
                        logger.info(f"Embodex challenge task enabled: {self.task.name}")
                    else:
                        self.task = None
            except Exception as e:
                logger.error(f"Failed to setup Embodex challenge task: {e}")
                self.task = None

        # Setup recorder if needed
        fps = 1.0 / self.config.send_interval if self.config.send_interval > 0 else 20.0
        self.recorder = TeleopRecorder(
            record_dir=self.record_root,
            fps=fps,
            vr_only=self.record_vr_only,
            left_enabled=getattr(self.config, "left_arm_enabled", True),
            right_enabled=getattr(self.config, "right_arm_enabled", True),
        )

        # Connect recorder to VR server callbacks
        if self.vr_server:
            if hasattr(self.vr_server, "add_frame_callback"):
                self.vr_server.add_frame_callback(self.recorder.on_vr_raw)
            setattr(self.vr_server, "recorder", self.recorder)

        return True

    def _task_gripper_angle(self, arm: str, current_gripper: float) -> float:
        """Override gripper angle when challenge task latches jaw."""
        if arm == "left" and self.task and self.task.is_grasped:
            if self.task.cfg.get("latch_gripper_while_grasped", True):
                return float(self.config.gripper_closed_angle)
        return current_gripper

    def _latch_task_gripper(self):
        """Force the left gripper closed while connector is grasped."""
        if not self.task or not self.robot_interface:
            return
        if self.task.is_grasped and self.task.cfg.get("latch_gripper_while_grasped", True):
            self.robot_interface.set_gripper("left", True)

    def _update_task(self):
        """Run challenge task physical simulation and update grasp latch."""
        if not self.task or not self.robot_interface:
            return
        try:
            gripper_deg = float(self.robot_interface.get_arm_angles("left")[GRIPPER_INDEX])
            self.task.update(gripper_deg)
            self._latch_task_gripper()
        except Exception as e:
            logger.debug(f"Task update exception: {e}")

    def _get_display_arm_angles(self, arm: str) -> np.ndarray:
        """Commanded joint angles for display, with task latch applied."""
        angles = self.robot_interface.get_arm_angles(arm) if self.robot_interface else np.zeros(NUM_JOINTS)
        if arm == "left":
            angles = angles.copy()
            angles[GRIPPER_INDEX] = self._task_gripper_angle(arm, angles[GRIPPER_INDEX])
        return angles

    async def _update_digital_twin(self):
        """Broadcast state to connected WebXR clients."""
        if not self.vr_server:
            return

        left_enabled = getattr(self.config, "left_arm_enabled", True)
        right_enabled = getattr(self.config, "right_arm_enabled", True)

        left_angles = self._get_display_arm_angles("left") if left_enabled else np.zeros(NUM_JOINTS)
        right_angles = self._get_display_arm_angles("right") if right_enabled else np.zeros(NUM_JOINTS)
        left_angles = left_angles.copy()
        right_angles = right_angles.copy()

        left_angles[GRIPPER_INDEX] = hardware_to_urdf_gripper_deg(left_angles[GRIPPER_INDEX])
        right_angles[GRIPPER_INDEX] = hardware_to_urdf_gripper_deg(right_angles[GRIPPER_INDEX])

        is_recording = self.recorder.is_running() if self.recorder else False
        session_id = self.recorder.session_id if self.recorder else None
        record_dir = str(self.recorder.record_dir) if self.recorder and self.recorder.record_dir else None

        objects = self.task.get_object_states() if self.task else None
        task_state = self.task.get_state() if self.task else None

        if hasattr(self.vr_server, "broadcast_robot_state"):
            await self.vr_server.broadcast_robot_state(
                left_angles,
                right_angles,
                is_recording=is_recording,
                session_id=session_id,
                record_dir=record_dir,
                objects=objects,
                task=task_state,
            )

    async def _toggle_recording(self):
        """Toggle session recording on/off."""
        if self.recorder and self.recorder.is_running():
            logger.info("Stopping Embodex recording...")
            session_id = self.recorder.stop()
            print(f"📼 Recording STOPPED: {self.recorder.record_dir} (session_id={session_id})")
            if self.vr_server and session_id and hasattr(self.vr_server, "broadcast_recording_stopped"):
                await self.vr_server.broadcast_recording_stopped(session_id, str(self.recorder.record_dir))
            if self.record_single_session:
                self._recording_finalized = True
        else:
            if self.record_single_session and self._recording_finalized:
                logger.info("Recording already finalized for this session.")
                return

            logger.info("Starting Embodex recording...")
            if not self.recorder:
                fps = 1.0 / self.config.send_interval if self.config.send_interval > 0 else 20.0
                self.recorder = TeleopRecorder(
                    record_dir=self.record_root,
                    fps=fps,
                    vr_only=self.record_vr_only,
                    left_enabled=getattr(self.config, "left_arm_enabled", True),
                    right_enabled=getattr(self.config, "right_arm_enabled", True),
                )
                if self.vr_server and hasattr(self.vr_server, "add_frame_callback"):
                    self.vr_server.add_frame_callback(self.recorder.on_vr_raw)

            if self.record_single_session:
                if self.record_session_dir is None:
                    self.record_session_dir = get_next_session_dir(self.record_root)
                new_dir = self.record_session_dir
            else:
                new_dir = get_next_session_dir(self.record_root)

            self.recorder.start(new_dir=new_dir)
            print(f"📼 Recording STARTED: {new_dir} (session_id={self.recorder.session_id})")

    async def _reset_recording(self):
        """Force-stop recording on VR disconnect or session reset."""
        if self.recorder and self.recorder.is_running():
            session_id = self.recorder.stop()
            if self.vr_server and session_id and hasattr(self.vr_server, "broadcast_recording_stopped"):
                await self.vr_server.broadcast_recording_stopped(session_id, str(self.recorder.record_dir))
        if self.record_single_session:
            self._recording_finalized = False
            self.record_session_dir = None

    async def _reset_robot(self):
        """Disengage and home the robot after a session."""
        if self.robot_interface:
            logger.info("Homing and disengaging robot...")
            self.robot_interface.disengage()
            self.left_arm.reset()
            self.right_arm.reset()
            logger.info("Robot reset complete")

    async def _execute_goal(self, goal: ControlGoal):
        """Intercept Embodex custom metadata commands before passing to TeleGrip."""
        if goal.metadata:
            if goal.metadata.get("record_toggle"):
                await self._toggle_recording()
                return
            if goal.metadata.get("task_reset"):
                if self.task:
                    self.task.reset()
                return
            if goal.metadata.get("record_reset"):
                await self._reset_recording()
                return
            if goal.metadata.get("robot_reset"):
                await self._reset_robot()
                return

        await super()._execute_goal(goal)

    async def step(self):
        """Run single control-loop step, recording snapshot and updating task."""
        # 1. Update task simulation
        self._update_task()

        # 2. Run TeleGrip's control and kinematic step
        await super().step()

        # 3. Record snapshot if recording is active
        if self.recorder and self.recorder.is_running():
            try:
                (
                    left_pose,
                    right_pose,
                    left_joints,
                    right_joints,
                    left_gripper,
                    right_gripper,
                ) = build_robot_goal_snapshot(self)

                env_state = None
                task_state = None
                if self.task:
                    task_state = self.task.get_state()
                    objects = self.task.get_object_states()
                    conn = objects.get("connector", {})
                    if "position" in conn and "quaternion" in conn:
                        pos = conn["position"]
                        q = conn["quaternion"]
                        env_state = [pos["x"], pos["y"], pos["z"], q["x"], q["y"], q["z"], q["w"]]

                self.recorder.on_robot_goal(
                    left_pose=left_pose,
                    right_pose=right_pose,
                    left_joints=left_joints,
                    right_joints=right_joints,
                    left_gripper_closed=left_gripper,
                    right_gripper_closed=right_gripper,
                    timestamp=time.time(),
                    env_state=env_state,
                    task_state=task_state,
                )
            except Exception as e:
                logger.debug(f"Recorder step error: {e}")

        # 4. Broadcast digital twin update
        if getattr(self.config, "digital_twin_enabled", True):
            await self._update_digital_twin()

    @property
    def status(self) -> Dict[str, Any]:
        """Extended status including recording and challenge task information."""
        base_status = super().status if hasattr(super(), "status") else {}
        is_rec = self.recorder.is_running() if self.recorder else False
        base_status.update({
            "recording": is_rec,
            "session_id": self.recorder.session_id if (self.recorder and is_rec) else None,
            "record_dir": str(self.recorder.record_dir) if (self.recorder and is_rec) else None,
            "task": self.task.get_state() if self.task else None,
        })
        return base_status
