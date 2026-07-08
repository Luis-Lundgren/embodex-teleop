"""
VR WebSocket server for receiving controller data from web browsers.
Adapted from the original vr_robot_teleop.py script.
"""

import asyncio
import json
import ssl
import numpy as np
import math
import logging
from typing import Dict, Optional, Set
from scipy.spatial.transform import Rotation as R

from .base import BaseInputProvider, ControlGoal, ControlMode
from ..config import TelegripConfig
from ..core.kinematics import compute_relative_position

logger = logging.getLogger(__name__)


class VRControllerState:
    """State tracking for a VR controller."""
    
    def __init__(self, hand: str):
        self.hand = hand
        self.grip_active = False
        self.trigger_active = False
        
        # Position tracking for relative movement
        self.origin_position = None
        self.origin_rotation = None
        
        # Quaternion-based rotation tracking (more stable than Euler)
        self.origin_quaternion = None
        self.accumulated_rotation_quat = None  # Accumulated rotation as quaternion
        
        # Rotation tracking for wrist control
        self.z_axis_rotation = 0.0  # For wrist_roll
        self.x_axis_rotation = 0.0  # For wrist_flex (pitch)
        
        # Position tracking
        self.current_position = None
        
        # Rotation tracking
        self.origin_wrist_angle = 0.0
    
    def reset_grip(self):
        """Reset grip state but preserve trigger state."""
        self.grip_active = False
        self.origin_position = None
        self.origin_rotation = None
        self.origin_quaternion = None
        self.accumulated_rotation_quat = None
        self.z_axis_rotation = 0.0
        self.x_axis_rotation = 0.0


class VRWebSocketServer(BaseInputProvider):
    """WebSocket server for VR controller input."""
    
    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig):
        super().__init__(command_queue)
        self.config = config
        self.clients: Set = set()
        self.server = None
        
        # Controller states
        self.left_controller = VRControllerState("left")
        self.right_controller = VRControllerState("right")
        
        # Robot state tracking (for relative position calculation)
        self.left_arm_origin_position = None
        self.right_arm_origin_position = None

        # Optional recorder for VR raw + robot-goal logging (LeRobot/ROS2 comparable)
        self.recorder = None

    def _get_local_ip(self) -> str:
        """Get the local IP address of this machine."""
        import socket
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except Exception:
            try:
                return socket.gethostbyname(socket.gethostname())
            except Exception:
                return "localhost"


    async def start(self):
        """Start the WebSocket server (legacy, now managed by FastAPI)."""
        logger.info("VR WebSocket logic initialized (FastAPI will handle the server)")
        self.is_running = True

    async def stop(self):
        """Stop the WebSocket server."""
        self.is_running = False
        # Close all active client connections
        for client in list(self.clients):
            try:
                # This works for both websockets library and FastAPI WebSocket
                if hasattr(client, 'close'):
                    await client.close()
            except Exception:
                pass
        logger.info("VR WebSocket logic stopped")

    async def fastapi_handler(self, websocket):
        """Handle WebSocket connections from VR controllers via FastAPI."""
        await websocket.accept()
        client_address = f"{websocket.client.host}:{websocket.client.port}"
        logger.info(f"VR client connected: {client_address}")
        self.clients.add(websocket)
        
        try:
            while True:
                message = await websocket.receive_text()
                try:
                    data = json.loads(message)
                    if data.get('action') == 'record_toggle':
                        logger.info("🔴 VR record_toggle requested")
                        # Pass to control loop via command queue
                        await self.command_queue.put(ControlGoal(
                            arm="left",
                            metadata={"record_toggle": True},
                        ))
                    elif data.get('action') == 'task_reset':
                        logger.info("🔌 VR task_reset requested")
                        await self.command_queue.put(ControlGoal(
                            arm="left",
                            metadata={"task_reset": True},
                        ))
                    else:
                        await self.process_controller_data(data)
                except json.JSONDecodeError:
                    logger.warning(f"Received non-JSON message: {message}")
                except Exception as e:
                    logger.error(f"Error processing VR data: {e}")
        
        except Exception as e:
            # FastAPI raises various exceptions on disconnect
            logger.info(f"VR client {client_address} disconnected: {e}")
        finally:
            self.clients.discard(websocket)
            # Handle grip releases when client disconnects
            await self.handle_grip_release('left')
            await self.handle_grip_release('right')
            
            # If no more clients, signal control loop to reset recording state and robot
            if not self.clients:
                logger.info("🎬 Last VR client disconnected, resetting recording state and robot")
                
                # Signal recording reset
                await self.command_queue.put(ControlGoal(
                    arm="left",
                    metadata={"record_reset": True},
                ))
                
                # Signal robot disconnect (which homes and disengages motors)
                # Instead of putting it in command_queue (which is for ControlGoals),
                # we should use the system's add_control_command if we had a reference,
                # but VRWebSocketServer usually only has the command_queue.
                # However, ControlLoop can also handle special metadata in ControlGoal
                # or we can pass a special goal.
                
                await self.command_queue.put(ControlGoal(
                    arm="left",
                    metadata={"robot_reset": True},
                ))
            
            logger.info(f"VR client {client_address} cleanup complete")
    

    async def process_controller_data(self, data: Dict):
        """Process incoming VR controller data."""
        
        # Debug logging (throttled)
        if hasattr(self, '_last_debug_log') and (asyncio.get_event_loop().time() - self._last_debug_log > 1.0):
             # summary of packet
             keys = list(data.keys())
             logger.info(f"VR DATA RECEIVED: Keys={keys}")
             if 'leftController' in data and data['leftController']:
                 lc = data['leftController']
                 if lc:
                     logger.info(f"  Left: pos={lc.get('position')} grip={lc.get('gripActive')} trigger={lc.get('trigger')}")
             self._last_debug_log = asyncio.get_event_loop().time()
        elif not hasattr(self, '_last_debug_log'):
             self._last_debug_log = 0

        # Handle new dual controller format
        if 'leftController' in data and 'rightController' in data:
            # Record raw VR packet for comparison with ROS2 / LeRobot
            if self.recorder and self.recorder.is_running():
                ts = data.get('timestamp')
                if ts is not None and isinstance(ts, (int, float)):
                    ts = ts / 1000.0 if ts > 1e12 else float(ts)
                self.recorder.on_vr_raw(data, timestamp=ts)
            left_data = data['leftController']
            right_data = data['rightController']
            
            # Process left controller
            if left_data and left_data.get('position') and (left_data.get('gripActive', False) or left_data.get('trigger', 0) > 0.5):
                await self.process_single_controller('left', left_data)
            elif left_data and not left_data.get('gripActive', False) and self.left_controller.grip_active:
                await self.handle_grip_release('left')
            
            # Process right controller
            if right_data and right_data.get('position') and (right_data.get('gripActive', False) or right_data.get('trigger', 0) > 0.5):
                await self.process_single_controller('right', right_data)
            elif right_data and not right_data.get('gripActive', False) and self.right_controller.grip_active:
                await self.handle_grip_release('right')
                
            return
        
        # Handle legacy single controller format
        hand = data.get('hand')
        
        # Handle explicit release messages
        if data.get('gripReleased'):
            await self.handle_grip_release(hand)
            return
        
        if data.get('triggerReleased'):
            await self.handle_trigger_release(hand)
            return
            
        # Process single controller data
        if hand and data.get('position') and (data.get('gripActive', False) or data.get('trigger', 0) > 0.5):
            await self.process_single_controller(hand, data)
    
    async def process_single_controller(self, hand: str, data: Dict):
        """Process data for a single controller."""
        position = data.get('position', {})
        rotation = data.get('rotation', {})
        quaternion = data.get('quaternion', {})  # Get quaternion data directly
        grip_active = data.get('gripActive', False)
        trigger = data.get('trigger', 0)
        
        controller = self.left_controller if hand == 'left' else self.right_controller
        
        # Handle trigger for gripper control
        trigger_active = trigger > 0.5
        if trigger_active != controller.trigger_active:
            controller.trigger_active = trigger_active

            # Trigger pressed -> close gripper; released -> open
            # (While a challenge object is grasped, the control loop latches closed.)
            gripper_goal = ControlGoal(
                arm=hand,
                gripper_closed=trigger_active,
                metadata={"source": "vr_trigger"}
            )
            await self.send_goal(gripper_goal)

            logger.info(f"🤏 {hand.upper()} gripper {'CLOSED' if trigger_active else 'OPENED'}")
        
        # Handle grip button for arm movement control
        if grip_active:
            if not controller.grip_active:
                # Grip just activated - set origin and reset target position
                controller.grip_active = True
                controller.origin_position = position.copy()
                
                # Use quaternion data directly if available, otherwise fall back to Euler conversion
                if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                    controller.origin_quaternion = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
                    controller.origin_rotation = controller.origin_quaternion  # Store for compatibility
                else:
                    # Fallback to Euler angle conversion
                    controller.origin_quaternion = self.euler_to_quaternion(rotation) if rotation else None
                    controller.origin_rotation = controller.origin_quaternion
                
                controller.accumulated_rotation_quat = controller.origin_quaternion
                controller.z_axis_rotation = 0.0
                controller.x_axis_rotation = 0.0
                
                # Send reset signal to control loop to reset target position to current robot position
                reset_goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,  # Keep in position control
                    target_position=None,  # Special signal
                    metadata={
                        "source": f"vr_grip_reset_{hand}",
                        "reset_target_to_current": True  # Signal to reset target to current position
                    }
                )
                await self.send_goal(reset_goal)
                
                logger.info(f"🔒 {hand.upper()} grip activated - controlling {hand} arm (target reset to current position)")
            
            # Compute target position
            if controller.origin_position:
                relative_delta = compute_relative_position(
                    position, 
                    controller.origin_position, 
                    self.config.vr_to_robot_scale
                )
                
                # Calculate Z-axis rotation for wrist_roll control
                # Calculate X-axis rotation for wrist_flex control
                if controller.origin_quaternion is not None:
                    # Update quaternion-based rotation tracking
                    if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                        # Use quaternion data directly
                        current_quat = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
                        self.update_quaternion_rotation_direct(controller, current_quat)
                    else:
                        # Fallback to Euler angle conversion
                        self.update_quaternion_rotation(controller, rotation)
                    
                    # Get accumulated rotations from quaternion
                    controller.z_axis_rotation = self.extract_roll_from_quaternion(controller.accumulated_rotation_quat, controller.origin_quaternion)
                    controller.x_axis_rotation = self.extract_pitch_from_quaternion(controller.accumulated_rotation_quat, controller.origin_quaternion)
                
                # Create position control goal
                # Note: We send relative position here, the control loop will handle
                # adding it to the robot's current position
                goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,
                    target_position=relative_delta,  # Relative position delta
                    wrist_roll_deg=-controller.z_axis_rotation,
                    wrist_flex_deg=-controller.x_axis_rotation,
                    metadata={
                        "source": "vr_grip",
                        "relative_position": True,
                        "origin_position": controller.origin_position.copy()
                    }
                )
                await self.send_goal(goal)
    
    async def handle_grip_release(self, hand: str):
        """Handle grip release for a controller."""
        if hand == 'left':
            controller = self.left_controller
        elif hand == 'right':
            controller = self.right_controller
        else:
            return
        
        if controller.grip_active:
            controller.reset_grip()
            
            # Send idle goal to stop arm control
            goal = ControlGoal(
                arm=hand,
                mode=ControlMode.IDLE,
                metadata={"source": "vr_grip_release"}
            )
            await self.send_goal(goal)
            
            logger.info(f"🔓 {hand.upper()} grip released - arm control stopped")
    
    async def handle_trigger_release(self, hand: str):
        """Handle trigger release for a controller."""
        controller = self.left_controller if hand == 'left' else self.right_controller
        
        if controller.trigger_active:
            controller.trigger_active = False
            
            # Open gripper when trigger released (control loop may latch closed if grasped)
            goal = ControlGoal(
                arm=hand,
                gripper_closed=False,
                metadata={"source": "vr_trigger_release"}
            )
            await self.send_goal(goal)
            
            logger.info(f"🤏 {hand.upper()} gripper OPENED (trigger released)")
    
    def euler_to_quaternion(self, euler_deg: Dict[str, float]) -> np.ndarray:
        """Convert Euler angles in degrees to quaternion [x, y, z, w]."""
        euler_rad = [math.radians(euler_deg['x']), math.radians(euler_deg['y']), math.radians(euler_deg['z'])]
        rotation = R.from_euler('xyz', euler_rad)
        return rotation.as_quat()
    
    def update_quaternion_rotation(self, controller: VRControllerState, current_euler: dict):
        """Update quaternion-based rotation tracking."""
        if not current_euler:
            return
        
        # Convert current Euler to quaternion
        current_quat = self.euler_to_quaternion(current_euler)
        
        # Store current quaternion for accumulated rotation calculation
        controller.accumulated_rotation_quat = current_quat
    
    def update_quaternion_rotation_direct(self, controller: VRControllerState, current_quat: np.ndarray):
        """Update quaternion-based rotation tracking using quaternion data directly."""
        if current_quat is None:
            return
        
        # Store current quaternion for accumulated rotation calculation
        controller.accumulated_rotation_quat = current_quat
    
    def extract_roll_from_quaternion(self, current_quat: np.ndarray, origin_quat: np.ndarray) -> float:
        """Extract roll rotation around Z-axis from relative quaternion rotation."""
        if current_quat is None or origin_quat is None:
            return 0.0
        
        try:
            # Calculate relative rotation quaternion (from origin to current)
            origin_rotation = R.from_quat(origin_quat)
            current_rotation = R.from_quat(current_quat)
            relative_rotation = current_rotation * origin_rotation.inv()
            
            # Project the relative rotation onto the Z-axis (roll)
            # Get the rotation vector (axis-angle representation)
            rotvec = relative_rotation.as_rotvec()
            
            # The Z-component of the rotation vector represents rotation around Z-axis (roll)
            z_rotation_rad = rotvec[2]
            z_rotation_deg = -np.degrees(z_rotation_rad)
            
            return z_rotation_deg
        except Exception as e:
            logger.warning(f"Error extracting roll from quaternion: {e}")
            return 0.0
    
    def extract_pitch_from_quaternion(self, current_quat: np.ndarray, origin_quat: np.ndarray) -> float:
        """Extract pitch rotation around X-axis from relative quaternion rotation."""
        if current_quat is None or origin_quat is None:
            return 0.0
        
        try:
            # Calculate relative rotation quaternion (from origin to current)
            origin_rotation = R.from_quat(origin_quat)
            current_rotation = R.from_quat(current_quat)
            relative_rotation = current_rotation * origin_rotation.inv()
            
            # Project the relative rotation onto the X-axis (pitch)
            # Get the rotation vector (axis-angle representation)
            rotvec = relative_rotation.as_rotvec()
            
            # The X-component of the rotation vector represents rotation around X-axis (pitch)
            x_rotation_rad = rotvec[0]
            x_rotation_deg = np.degrees(x_rotation_rad)
            
            return x_rotation_deg
        except Exception as e:
            logger.warning(f"Error extracting pitch from quaternion: {e}")
            return 0.0

    async def broadcast_robot_state(
        self,
        left_angles: np.ndarray,
        right_angles: Optional[np.ndarray] = None,
        is_recording: bool = False,
        session_id: Optional[str] = None,
        record_dir: Optional[str] = None,
        objects: Optional[list] = None,
        task: Optional[dict] = None,
    ):
        """Broadcast robot state to all connected clients."""
        if not self.clients:
            return

        message = {
            "type": "robot_state",
            "timestamp": int(asyncio.get_event_loop().time() * 1000),
            "left_arm": left_angles.tolist() if left_angles is not None else [],
            "recording": is_recording,
        }
        if session_id:
            message["session_id"] = session_id
        if record_dir:
            message["record_dir"] = record_dir
        if objects is not None:
            message["objects"] = objects
        if task is not None:
            message["task"] = task

        if right_angles is not None:
             message["right_arm"] = right_angles.tolist()

        encoded_message = json.dumps(message)

        await asyncio.gather(
            *[client.send_text(encoded_message) for client in self.clients],
            return_exceptions=True
        )

    async def broadcast_recording_stopped(self, session_id: str, record_dir: str):
        """Notify clients that a recording has finished and been flushed."""
        if not self.clients:
            return

        message = json.dumps({
            "type": "recording_stopped",
            "session_id": session_id,
            "record_dir": record_dir,
            "timestamp": int(asyncio.get_event_loop().time() * 1000),
        })

        await asyncio.gather(
            *[client.send_text(message) for client in self.clients],
            return_exceptions=True
        )