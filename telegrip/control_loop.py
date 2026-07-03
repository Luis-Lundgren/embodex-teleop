"""
Main control loop for the teleoperation system.
Consumes control goals from the command queue and executes them via the robot interface.
"""

import asyncio
import numpy as np
import logging
import time
import queue  # Add import for thread-safe queue
from pathlib import Path
from typing import Dict, Optional

from .config import TelegripConfig, NUM_JOINTS, WRIST_FLEX_INDEX, WRIST_ROLL_INDEX, GRIPPER_INDEX
from .core.robot_interface import RobotInterface
# PyBulletVisualizer will be imported on demand
from .inputs.base import ControlGoal, ControlMode
# WebKeyboardHandler will be imported on demand to avoid circular imports

logger = logging.getLogger(__name__)


class ArmState:
    """State tracking for a single robot arm."""
    
    def __init__(self, arm_name: str):
        self.arm_name = arm_name
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.goal_position = None  # For visualization
        self.origin_position = None  # Robot position when grip was activated
        self.origin_wrist_roll_angle = 0.0
        self.origin_wrist_flex_angle = 0.0
        self.current_wrist_roll = 0.0
        self.current_wrist_flex = 0.0
        
    def reset(self):
        """Reset arm state to idle."""
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.goal_position = None
        self.origin_position = None
        self.origin_wrist_roll_angle = 0.0
        self.origin_wrist_flex_angle = 0.0


class ControlLoop:
    """Main control loop that processes command queue and controls robot."""
    
    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig, control_commands_queue: Optional[queue.Queue] = None):
        self.command_queue = command_queue
        self.control_commands_queue = control_commands_queue
        self.config = config
        
        # Components
        self.robot_interface = None
        self.visualizer = None
        self.web_keyboard_handler = None  # Reference to web-based keyboard handler
        self.vr_server = None # Reference to VR server for digital twin updates
        
        # Arm states
        self.left_arm = ArmState("left")
        self.right_arm = ArmState("right")
        
        # Control timing
        self.last_log_time = 0
        self.log_interval = 1.0  # Log status every second
        
        # Debug flags
        self._queue_debug_logged = False
        self._process_debug_logged = False

        # Optional ROS2 bridge (canonical transform broker)
        self.ros2_bridge = None
        # Optional recorder for VR raw + robot-goal (LeRobot/ROS2 comparable)
        self.recorder = None
        self.record_root = Path("motion_exchange/datasets")
        self.record_vr_only = False
        self.record_autostart = True
        self.record_single_session = False
        self.record_session_dir: Optional[Path] = None
        self._recording_finalized = False

        self.is_running = False
    
    def setup(self) -> bool:
        """Setup robot interface and visualizer."""
        success = True
        setup_errors = []
        
        # Setup robot interface
        try:
            self.robot_interface = RobotInterface(self.config)
            if not self.robot_interface.connect():
                error_msg = "Robot interface failed to connect"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                if self.config.enable_robot:
                    success = False
        except Exception as e:
            error_msg = f"Robot interface setup failed with exception: {e}"
            logger.error(error_msg)
            setup_errors.append(error_msg)
            if self.config.enable_robot:
                success = False
        
        # Setup PyBullet simulation, IK and visualizer
        if self.config.enable_pybullet:
            try:
                # Import PyBulletVisualizer on demand
                from .core.visualizer import PyBulletVisualizer
                
                self.visualizer = PyBulletVisualizer(
                    self.config.get_absolute_urdf_path(), 
                    use_gui=self.config.enable_pybullet_gui,
                    log_level=self.config.log_level
                )
                if not self.visualizer.setup():
                    error_msg = "PyBullet visualizer setup failed"
                    logger.error(error_msg)
                    setup_errors.append(error_msg)
                    self.visualizer = None
                else:
                    # Connect kinematics to robot interface
                    joint_limits_min, joint_limits_max = self.visualizer.get_joint_limits
                    self.robot_interface.setup_kinematics(
                        self.visualizer.physics_client,
                        self.visualizer.robot_ids,  # Pass both robot instances
                        self.visualizer.joint_indices,  # Pass both joint index mappings
                        self.visualizer.end_effector_link_indices,  # Pass both end effector indices
                        joint_limits_min,
                        joint_limits_max
                    )
            except Exception as e:
                error_msg = f"PyBullet visualizer setup failed with exception: {e}"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                self.visualizer = None
        
        # Report all setup issues
        if setup_errors:
            logger.error("Setup failed with the following errors:")
            for i, error in enumerate(setup_errors, 1):
                logger.error(f"  {i}. {error}")
        
        # Set robot interface on web keyboard handler so it can get current positions
        if self.web_keyboard_handler and self.robot_interface:
            self.web_keyboard_handler.set_robot_interface(self.robot_interface)
            logger.info("Set robot interface on web keyboard handler")

        return success
    
    async def start(self):
        """Start the control loop."""
        if not self.setup():
            logger.error("Control loop setup failed")
            return
        
        self.is_running = True
        logger.info("Control loop started")
        
        # Initialize arm states with current robot positions
        self._initialize_arm_states()
        
        # Main control loop
        while self.is_running:
            try:
                # Process command queue
                await self._process_commands()
                
                # Update robot (with error resilience)
                self._update_robot_safely()
                
                # Update visualization
                if self.visualizer:
                    self._update_visualization()

                # Update AR digital twin
                if self.config.digital_twin_enabled and self.vr_server:
                    await self._update_digital_twin()

                # Stream to ROS2 (TF2 + topics) when enabled
                if self.ros2_bridge:
                    self.ros2_bridge.publish(self)
                # Record robot goal snapshot (ROS2-equivalent) for comparison with rosbag/LeRobot
                if self.recorder and self.recorder.is_running():
                    from .recorder import build_robot_goal_snapshot
                    left_pose, right_pose, left_joints, right_joints, left_gripper, right_gripper = build_robot_goal_snapshot(self)
                    
                    # Also get ACTUAL (measured) joints for observation.state
                    left_actual = self.robot_interface.get_actual_arm_angles("left") if self.config.left_arm_enabled else None
                    right_actual = self.robot_interface.get_actual_arm_angles("right") if self.config.right_arm_enabled else None
                    
                    self.recorder.on_robot_goal(
                        left_pose, right_pose, left_joints, right_joints, left_gripper, right_gripper,
                        left_actual=left_actual, right_actual=right_actual,
                        timestamp=time.time(),
                    )
                
                # Periodic logging
                self._periodic_logging()
                
                # Control rate
                await asyncio.sleep(self.config.send_interval)
                
            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                await asyncio.sleep(0.1)
        
        logger.info("Control loop stopped")
    
    async def stop(self):
        """Stop the control loop."""
        self.is_running = False

        # Cleanup - disengage robot first (returns to home and disables torque)
        if self.robot_interface:
            if self.robot_interface.is_engaged:
                logger.info("🛑 Disengaging robot before shutdown...")
                self.robot_interface.disengage()
            self.robot_interface.disconnect()

        if self.visualizer:
            self.visualizer.disconnect()

        if self.ros2_bridge:
            self.ros2_bridge.stop()
    
    def _initialize_arm_states(self):
        """Initialize arm states with current robot positions."""
        if self.robot_interface:
            # Get current end effector positions
            left_pos = self.robot_interface.get_current_end_effector_position("left")
            right_pos = self.robot_interface.get_current_end_effector_position("right")
            
            # Initialize target positions to current positions (ensure deep copies)
            self.left_arm.target_position = left_pos.copy()
            self.left_arm.goal_position = left_pos.copy()
            self.right_arm.target_position = right_pos.copy()
            self.right_arm.goal_position = right_pos.copy()
            
            # Get current wrist roll angles
            left_angles = self.robot_interface.get_arm_angles("left")
            right_angles = self.robot_interface.get_arm_angles("right")
            
            self.left_arm.current_wrist_roll = left_angles[WRIST_ROLL_INDEX]
            self.right_arm.current_wrist_roll = right_angles[WRIST_ROLL_INDEX]
            
            self.left_arm.current_wrist_flex = left_angles[WRIST_FLEX_INDEX]
            self.right_arm.current_wrist_flex = right_angles[WRIST_FLEX_INDEX]
            
            logger.info(f"Initialized left arm at position: {left_pos.round(3)}")
            logger.info(f"Initialized right arm at position: {right_pos.round(3)}")
    
    async def _process_commands(self):
        """Process commands from the command queue."""
        try:
            # Process regular control goals
            while not self.command_queue.empty():
                goal = self.command_queue.get_nowait()
                await self._execute_goal(goal)
        except Exception as e:
            logger.error(f"Error processing commands: {e}")
            import traceback
            logger.error(f"Traceback: {traceback.format_exc()}")
    
    async def _handle_command(self, command):
        """Handle individual commands."""
        action = command.get('action', '')
        logger.info(f"🔌 Processing control command: {action}")
        
        if action == 'enable_keyboard':
            if self.web_keyboard_handler:
                await self.web_keyboard_handler.start()
                logger.info("🎮 Keyboard control ENABLED via API")
        elif action == 'disable_keyboard':
            if self.web_keyboard_handler:
                await self.web_keyboard_handler.stop()
                logger.info("🎮 Keyboard control DISABLED via API")
        elif action == 'web_keypress':
            # Handle individual keypress events from web interface
            key = command.get('key')
            event = command.get('event')  # 'press' or 'release'

            if self.web_keyboard_handler and self.web_keyboard_handler.is_enabled:
                logger.debug(f"🌐 Processing web keypress: {key}_{event}")
                if event == 'press':
                    self.web_keyboard_handler.on_key_press(key)
                elif event == 'release':
                    self.web_keyboard_handler.on_key_release(key)
            else:
                logger.warning("🎮 Web keyboard handler not enabled")
        elif action == 'robot_connect':
            logger.info("🔌 Processing robot_connect command")
            if self.robot_interface and self.robot_interface.is_connected:
                logger.info(f"🔌 Robot interface available and connected: {self.robot_interface.is_connected}")
                success = self.robot_interface.engage()
                if success:
                    logger.info("🔌 Robot motors ENGAGED via API")
                    # No need to sync keyboard targets - unified system handles this automatically
                else:
                    logger.error("❌ Failed to engage robot motors")
            else:
                logger.warning(f"Cannot engage robot: interface={self.robot_interface is not None}, connected={self.robot_interface.is_connected if self.robot_interface else False}")
        elif action == 'robot_disconnect':
            logger.info("🔌 Processing robot_disconnect command")
            if self.robot_interface:
                logger.info(f"🔌 Robot interface available")
                success = self.robot_interface.disengage()
                if success:
                    logger.info("🔌 Robot motors DISENGAGED via API")
                    # Reset arm states to IDLE when robot is disengaged
                    self.left_arm.reset()
                    self.right_arm.reset()
                    logger.info("🔓 Both arms: Position control DEACTIVATED after robot disconnect")
                    
                    # Hide visualization markers
                    if self.visualizer:
                        for arm in ["left", "right"]:
                            self.visualizer.hide_marker(f"{arm}_goal")
                            self.visualizer.hide_frame(f"{arm}_goal_frame")
                            self.visualizer.hide_marker(f"{arm}_target")
                            self.visualizer.hide_frame(f"{arm}_target_frame")
                else:
                    logger.error("❌ Failed to disengage robot motors")
            else:
                logger.warning("Cannot disengage robot: no robot interface")
        else:
            logger.warning(f"Unknown command: {action}")

    async def _execute_goal(self, goal: ControlGoal):
        """Execute a control goal."""
        arm_state = self.left_arm if goal.arm == "left" else self.right_arm
        
        # Handle special reset signal from keyboard idle timeout
        if (goal.metadata and goal.metadata.get("reset_target_to_current", False)):
            if self.robot_interface and arm_state.mode == ControlMode.POSITION_CONTROL:
                # Reset target position to current robot position
                current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                current_angles = self.robot_interface.get_arm_angles(goal.arm)
                
                arm_state.target_position = current_position.copy()
                arm_state.goal_position = current_position.copy()
                arm_state.origin_position = current_position.copy()
                arm_state.current_wrist_roll = current_angles[WRIST_ROLL_INDEX]
                arm_state.current_wrist_flex = current_angles[WRIST_FLEX_INDEX]
                arm_state.origin_wrist_roll_angle = current_angles[WRIST_ROLL_INDEX]
                arm_state.origin_wrist_flex_angle = current_angles[WRIST_FLEX_INDEX]
                
                logger.info(f"🔄 {goal.arm.upper()} arm: Target position reset to current robot position (idle timeout)")
            return
        
        # Handle recording toggle from VR
        if (goal.metadata and goal.metadata.get("record_toggle")):
            await self._toggle_recording()
            return
        
        # Handle recording reset from VR disconnect
        if (goal.metadata and goal.metadata.get("record_reset")):
            await self._reset_recording()
            return

        # Handle robot reset from VR disconnect
        if (goal.metadata and goal.metadata.get("robot_reset")):
            await self._reset_robot()
            return
        
        # Handle mode changes (only if mode is specified)
        if goal.mode is not None and goal.mode != arm_state.mode:
            if goal.mode == ControlMode.POSITION_CONTROL:
                # Activate position control - always reset target to current position
                arm_state.mode = ControlMode.POSITION_CONTROL
                
                if self.robot_interface:
                    current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                    current_angles = self.robot_interface.get_arm_angles(goal.arm)
                    
                    # Reset everything to current position (like VR grip press)
                    arm_state.target_position = current_position.copy()
                    arm_state.goal_position = current_position.copy()
                    arm_state.origin_position = current_position.copy()
                    arm_state.current_wrist_roll = current_angles[WRIST_ROLL_INDEX]
                    arm_state.current_wrist_flex = current_angles[WRIST_FLEX_INDEX]
                    arm_state.origin_wrist_roll_angle = current_angles[WRIST_ROLL_INDEX]
                    arm_state.origin_wrist_flex_angle = current_angles[WRIST_FLEX_INDEX]
                
                logger.info(f"🔒 {goal.arm.upper()} arm: Position control ACTIVATED (target reset to current position)")
                
            elif goal.mode == ControlMode.IDLE:
                # Deactivate position control
                arm_state.reset()
                
                # Hide visualization markers
                if self.visualizer:
                    self.visualizer.hide_marker(f"{goal.arm}_goal")
                    self.visualizer.hide_frame(f"{goal.arm}_goal_frame")
                
                logger.info(f"🔓 {goal.arm.upper()} arm: Position control DEACTIVATED")
        
        # Handle position control - both VR and keyboard now work the same way (absolute offset from origin)
        if goal.target_position is not None and arm_state.mode == ControlMode.POSITION_CONTROL:
            if goal.metadata and goal.metadata.get("relative_position", False):
                # Both VR and keyboard send absolute offset from robot origin position
                if arm_state.origin_position is not None:
                    arm_state.target_position = arm_state.origin_position + goal.target_position
                    arm_state.goal_position = arm_state.target_position.copy()
                else:
                    # No origin set yet, use current position as base
                    if self.robot_interface:
                        current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                        arm_state.target_position = current_position + goal.target_position
                        arm_state.goal_position = arm_state.target_position.copy()
            else:
                # Absolute position (legacy - should not be used anymore)
                arm_state.target_position = goal.target_position.copy()
                arm_state.goal_position = goal.target_position.copy()
            
            # Handle wrist movements - both VR and keyboard send absolute offsets from origin
            if goal.wrist_roll_deg is not None:
                if goal.metadata and goal.metadata.get("relative_position", False):
                    # Both VR and keyboard send absolute wrist angle relative to origin
                    arm_state.current_wrist_roll = arm_state.origin_wrist_roll_angle + goal.wrist_roll_deg
                else:
                    # Absolute wrist roll (legacy)
                    arm_state.current_wrist_roll = goal.wrist_roll_deg
            
            # Handle wrist flex - both VR and keyboard send absolute offsets from origin
            if goal.wrist_flex_deg is not None:
                if goal.metadata and goal.metadata.get("relative_position", False):
                    # Both VR and keyboard send absolute wrist angle relative to origin
                    arm_state.current_wrist_flex = arm_state.origin_wrist_flex_angle + goal.wrist_flex_deg
                else:
                    # Absolute wrist flex (legacy)
                    arm_state.current_wrist_flex = goal.wrist_flex_deg
        
        # Handle gripper control (independent of mode)
        if goal.gripper_closed is not None and self.robot_interface:
            self.robot_interface.set_gripper(goal.arm, goal.gripper_closed)
    
    def _update_robot_safely(self):
        """Update robot with current control goals (with error handling)."""
        if not self.robot_interface:
            return
        
        try:
            self._update_robot()
        except Exception as e:
            logger.error(f"Error updating robot: {e}")
            # Don't shutdown, just continue - robot interface will handle connection issues
    
    def _update_robot(self):
        """Update robot with current control goals."""
        if not self.robot_interface:
            return
        
        # Update left arm (only if connected)
        if (self.left_arm.mode == ControlMode.POSITION_CONTROL and 
            self.left_arm.target_position is not None and
            self.robot_interface.get_arm_connection_status("left")):
            
            # Solve IK
            ik_solution = self.robot_interface.solve_ik("left", self.left_arm.target_position)
            
            # Update robot angles
            current_gripper = self.robot_interface.get_arm_angles("left")[GRIPPER_INDEX]
            self.robot_interface.update_arm_angles("left", ik_solution, 
                                                 self.left_arm.current_wrist_flex, 
                                                 self.left_arm.current_wrist_roll, 
                                                 current_gripper)

        # Update right arm (only if connected)
        if (self.right_arm.mode == ControlMode.POSITION_CONTROL and 
            self.right_arm.target_position is not None and
            self.robot_interface.get_arm_connection_status("right")):
            
            # Solve IK
            ik_solution = self.robot_interface.solve_ik("right", self.right_arm.target_position)
            
            # Update robot angles
            current_gripper = self.robot_interface.get_arm_angles("right")[GRIPPER_INDEX]
            self.robot_interface.update_arm_angles("right", ik_solution, 
                                                  self.right_arm.current_wrist_flex, 
                                                  self.right_arm.current_wrist_roll, 
                                                  current_gripper)

        # Send commands to robot
        if self.robot_interface.is_connected and self.robot_interface.is_engaged:
            self.robot_interface.send_command()
    
    def _update_visualization(self):
        """Update PyBullet visualization."""
        if not self.visualizer:
            return
        
        # Update robot poses for both arms using ACTUAL angles from robot hardware
        left_angles = self.robot_interface.get_actual_arm_angles("left")
        right_angles = self.robot_interface.get_actual_arm_angles("right")
        
        self.visualizer.update_robot_pose(left_angles, 'left')
        self.visualizer.update_robot_pose(right_angles, 'right')
        
        # Update visualization markers
        if self.left_arm.mode == ControlMode.POSITION_CONTROL:
            if self.left_arm.target_position is not None:
                # Show current end effector position
                current_pos = self.robot_interface.get_current_end_effector_position("left")
                self.visualizer.update_marker_position("left_target", current_pos)
                self.visualizer.update_coordinate_frame("left_target_frame", current_pos)
            
            if self.left_arm.goal_position is not None:
                # Show goal position
                self.visualizer.update_marker_position("left_goal", self.left_arm.goal_position)
                self.visualizer.update_coordinate_frame("left_goal_frame", self.left_arm.goal_position)
        else:
            # Hide markers when not in position control
            self.visualizer.hide_marker("left_target")
            self.visualizer.hide_marker("left_goal")
            self.visualizer.hide_frame("left_target_frame")
            self.visualizer.hide_frame("left_goal_frame")
        
        if self.right_arm.mode == ControlMode.POSITION_CONTROL:
            if self.right_arm.target_position is not None:
                # Show current end effector position
                current_pos = self.robot_interface.get_current_end_effector_position("right")
                self.visualizer.update_marker_position("right_target", current_pos)
                self.visualizer.update_coordinate_frame("right_target_frame", current_pos)
            
            if self.right_arm.goal_position is not None:
                # Show goal position
                self.visualizer.update_marker_position("right_goal", self.right_arm.goal_position)
                self.visualizer.update_coordinate_frame("right_goal_frame", self.right_arm.goal_position)
        else:
            # Hide markers when not in position control
            self.visualizer.hide_marker("right_target")
            self.visualizer.hide_marker("right_goal")
            self.visualizer.hide_frame("right_target_frame")
            self.visualizer.hide_frame("right_goal_frame")
        
        # Step simulation
        self.visualizer.step_simulation()
    
    async def _update_digital_twin(self):
        """Update digital twin visualization via VR server."""
        if not self.vr_server or not self.robot_interface:
            return
            
        # Get current angles only for enabled arms
        left_angles = self.robot_interface.get_actual_arm_angles("left") if self.config.left_arm_enabled else np.zeros(NUM_JOINTS)
        right_angles = self.robot_interface.get_actual_arm_angles("right") if self.config.right_arm_enabled else np.zeros(NUM_JOINTS)
        
        # Include recording status in digital twin update
        is_recording = self.recorder.is_running() if self.recorder else False
        session_id = self.recorder.session_id if self.recorder else None
        record_dir = str(self.recorder.record_dir) if self.recorder and self.recorder.record_dir else None
        await self.vr_server.broadcast_robot_state(
            left_angles, right_angles,
            is_recording=is_recording,
            session_id=session_id,
            record_dir=record_dir,
        )

    async def _toggle_recording(self):
        """Toggle recording state and create recorder if needed."""
        if self.recorder and self.recorder.is_running():
            logger.info("🎬 Stopping recording (VR button)...")
            session_id = self.recorder.stop()
            print(f"📼 Recording STOPPED: {self.recorder.record_dir} (session_id={session_id})")
            if self.vr_server and session_id:
                await self.vr_server.broadcast_recording_stopped(
                    session_id, str(self.recorder.record_dir)
                )
            if self.record_single_session:
                self._recording_finalized = True
        else:
            if self.record_single_session and self._recording_finalized:
                logger.info("🎬 Recording already finalized for this session; restart to record again.")
                return

            logger.info("🎬 Starting recording (VR button)...")
            from .utils import get_next_session_dir
            from .recorder import TeleopRecorder
            
            # Ensure recorder exists
            if not self.recorder:
                fps = 1.0 / self.config.send_interval if self.config.send_interval > 0 else 20.0
                self.recorder = TeleopRecorder(
                    record_dir=self.record_root, # Will be overridden in start()
                    fps=fps,
                    vr_only=self.record_vr_only,
                    left_enabled=self.config.left_arm_enabled,
                    right_enabled=self.config.right_arm_enabled
                )
                # Link to VR server for raw logging
                if self.vr_server:
                    self.vr_server.recorder = self.recorder

            if self.record_single_session:
                if self.record_session_dir is None:
                    self.record_session_dir = get_next_session_dir(self.record_root)
                self.recorder.start(new_dir=self.record_session_dir)
                new_dir = self.record_session_dir
            else:
                # Increment directory and start
                new_dir = get_next_session_dir(self.record_root)
                self.recorder.start(new_dir=new_dir)
            
            # Log to console for visual feedback on host
            print(f"Record single session set to {self.record_single_session}")
            print(f"📼 Recording STARTED: {new_dir} (session_id={self.recorder.session_id})")

    async def _reset_recording(self):
        """Reset recording state to allow a new session if record_single_session is enabled."""
        if self.recorder and self.recorder.is_running():
            logger.info("🎬 Forcing stop of recording (reset/disconnect)...")
            session_id = self.recorder.stop()
            print(f"📼 Recording STOPPED (disconnect): {self.recorder.record_dir} (session_id={session_id})")
            if self.vr_server and session_id:
                await self.vr_server.broadcast_recording_stopped(
                    session_id, str(self.recorder.record_dir)
                )
        
        if self.record_single_session:
            logger.info("🎬 Resetting single session recording state for next connection.")
            self._recording_finalized = False
            self.record_session_dir = None

    async def _reset_robot(self):
        """Disengage and home the robot after a session."""
        if self.robot_interface:
            logger.info("🎬 Homing and disengaging robot (session refresh)...")
            self.robot_interface.disengage()
            self.left_arm.reset()
            self.right_arm.reset()
            logger.info("✅ Robot reset complete")
    
    def _periodic_logging(self):
        """Log status information periodically."""
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time
            
            active_arms = []
            if self.left_arm.mode == ControlMode.POSITION_CONTROL:
                active_arms.append("LEFT")
            if self.right_arm.mode == ControlMode.POSITION_CONTROL:
                active_arms.append("RIGHT")
            
            if active_arms and self.robot_interface:
                left_angles = self.robot_interface.get_arm_angles("left")
                right_angles = self.robot_interface.get_arm_angles("right")
                logger.info(f"🤖 Active control: {', '.join(active_arms)} | Left: {left_angles.round(1)} | Right: {right_angles.round(1)}")
    
    @property
    def status(self) -> Dict:
        """Get current control loop status."""
        is_recording = self.recorder.is_running() if self.recorder else False
        return {
            "running": self.is_running,
            "left_arm_mode": self.left_arm.mode.value,
            "right_arm_mode": self.right_arm.mode.value,
            "robot_connected": self.robot_interface.is_connected if self.robot_interface else False,
            "left_arm_connected": self.robot_interface.get_arm_connection_status("left") if self.robot_interface else False,
            "right_arm_connected": self.robot_interface.get_arm_connection_status("right") if self.robot_interface else False,
            "visualizer_connected": self.visualizer.is_connected if self.visualizer else False,
            "recording": is_recording,
            "session_id": self.recorder.session_id if self.recorder and is_recording else None,
            "record_dir": str(self.recorder.record_dir) if self.recorder and is_recording else None,
        }
