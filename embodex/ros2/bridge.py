"""
ROS2 bridge: streams VR/teleop commands as TF2 and topics so a ROS2 backend
acts as the canonical transform broker. Optional; requires rclpy and a sourced
ROS2 environment.
"""

import logging
import math
import threading
from typing import Optional, TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation

if TYPE_CHECKING:
    from .control_loop import ControlLoop  # noqa: F401

logger = logging.getLogger(__name__)

# Lazy ROS2 imports so telegrip runs without ROS2
_rclpy = None
_geometry_msgs = None
_std_msgs = None
_tf2_ros = None


def _import_ros2():
    global _rclpy, _geometry_msgs, _std_msgs, _tf2_ros
    if _rclpy is not None:
        return True
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from std_msgs.msg import Bool
        from tf2_ros import TransformBroadcaster
        _rclpy = rclpy
        _geometry_msgs = type("GeometryMsgs", (), {"PoseStamped": PoseStamped, "TransformStamped": TransformStamped})()
        _std_msgs = type("StdMsgs", (), {"Bool": Bool})()
        _tf2_ros = type("Tf2Ros", (), {"TransformBroadcaster": TransformBroadcaster})()
        return True
    except ImportError as e:
        logger.debug("ROS2 not available: %s", e)
        return False


def _pose_from_position_and_wrist(
    position: np.ndarray,
    wrist_roll_deg: float,
    wrist_flex_deg: float,
) -> tuple:
    """Build (x, y, z, qx, qy, qz, qw) in ROS convention from robot-frame position and wrist angles."""
    # Robot frame: X=forward, Y=left, Z=up. Wrist roll around Z, flex (pitch) around Y.
    roll_rad = math.radians(wrist_roll_deg)
    flex_rad = math.radians(wrist_flex_deg)
    rot = ScipyRotation.from_euler("zy", [roll_rad, flex_rad])
    quat = rot.as_quat()  # scipy: xyzw
    return (
        float(position[0]),
        float(position[1]),
        float(position[2]),
        float(quat[0]),
        float(quat[1]),
        float(quat[2]),
        float(quat[3]),
    )


class ROS2Bridge:
    """
    Publishes telegrip arm goals to ROS2 as TF2 and PoseStamped topics,
    so a ROS2 backend can act as the canonical transform broker.
    """

    def __init__(
        self,
        node_name: str = "embodex_teleop_bridge",
        base_frame: str = "teleop_base",
        topic_prefix: str = "embodex",
    ):
        if not _import_ros2():
            raise RuntimeError(
                "ROS2 bridge requested but rclpy not available. "
                "Source your ROS2 workspace and install rclpy (e.g. ros-humble-rclpy)."
            )
        self._node_name = node_name
        self._base_frame = base_frame
        self._topic_prefix = topic_prefix
        self._node = None
        self._tf_broadcaster = None
        self._pub_left_pose = None
        self._pub_right_pose = None
        self._pub_left_gripper = None
        self._pub_right_gripper = None
        self._thread = None
        self._running = False
        self._init_node()

    def _init_node(self):
        rclpy = _rclpy
        try:
            rclpy.init()
        except Exception:
            pass  # already inited
        self._node = rclpy.create_node(self._node_name)
        self._tf_broadcaster = _tf2_ros.TransformBroadcaster(self._node)
        TransformStamped = _geometry_msgs.TransformStamped
        PoseStamped = _geometry_msgs.PoseStamped
        Bool = _std_msgs.Bool

        self._pub_left_pose = self._node.create_publisher(
            PoseStamped, f"{self._topic_prefix}/left_ee_goal", 10
        )
        self._pub_right_pose = self._node.create_publisher(
            PoseStamped, f"{self._topic_prefix}/right_ee_goal", 10
        )
        self._pub_left_gripper = self._node.create_publisher(
            Bool, f"{self._topic_prefix}/left_gripper_closed", 10
        )
        self._pub_right_gripper = self._node.create_publisher(
            Bool, f"{self._topic_prefix}/right_gripper_closed", 10
        )
        logger.info(
            "ROS2 bridge node '%s': base_frame=%s, topics %s/*",
            self._node_name,
            self._base_frame,
            self._topic_prefix,
        )

    def start_spin_thread(self):
        """Start rclpy spin in a daemon thread so the node processes outgoing messages."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True

        def spin():
            executor = _rclpy.executors.SingleThreadedExecutor()
            executor.add_node(self._node)
            while self._running and _rclpy.ok():
                executor.spin_once(timeout_sec=0.05)

        self._thread = threading.Thread(target=spin, daemon=True)
        self._thread.start()
        logger.info("ROS2 bridge spin thread started")

    def stop(self):
        """Stop the spin thread and destroy the node."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._node is not None and _rclpy is not None:
            self._node.destroy_node()
            self._node = None
        logger.info("ROS2 bridge stopped")

    def publish(self, control_loop: "ControlLoop"):
        """Publish current arm goals from the control loop to ROS2 (TF2 + topics)."""
        if self._node is None or not _rclpy.ok():
            return
        rclpy = _rclpy
        now = self._node.get_clock().now().to_msg()
        TransformStamped = _geometry_msgs.TransformStamped
        PoseStamped = _geometry_msgs.PoseStamped
        Bool = _std_msgs.Bool

        for arm_name, arm_state in [("left", control_loop.left_arm), ("right", control_loop.right_arm)]:
            if arm_state.mode.value != "position" or arm_state.target_position is None:
                continue
            pos = arm_state.target_position
            roll = arm_state.current_wrist_roll
            flex = arm_state.current_wrist_flex
            x, y, z, qx, qy, qz, qw = _pose_from_position_and_wrist(pos, roll, flex)

            child_frame = f"{self._topic_prefix}/{arm_name}_ee_goal"

            # TF2
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self._base_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(t)

            # PoseStamped topic
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now
            pose_msg.header.frame_id = self._base_frame
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw
            if arm_name == "left":
                self._pub_left_pose.publish(pose_msg)
            else:
                self._pub_right_pose.publish(pose_msg)

        # Gripper state: publish from robot interface if available
        if control_loop.robot_interface is not None:
            for arm_name in ("left", "right"):
                try:
                    angles = control_loop.robot_interface.get_arm_angles(arm_name)
                    # Index 5 is gripper; treat closed as angle > threshold (e.g. > 20 deg)
                    closed = float(angles[5]) > 20.0
                    b = Bool()
                    b.data = closed
                    if arm_name == "left":
                        self._pub_left_gripper.publish(b)
                    else:
                        self._pub_right_gripper.publish(b)
                except Exception:
                    pass


def create_ros2_bridge_if_enabled(
    enable_ros2: bool,
  node_name: str = "telegrip_bridge",
  base_frame: str = "teleop_base",
  topic_prefix: str = "telegrip",
) -> Optional[ROS2Bridge]:
    """Create and return a ROS2Bridge if enable_ros2 is True and ROS2 is available; else None."""
    if not enable_ros2:
        return None
    if not _import_ros2():
        logger.warning("ROS2 bridge requested but rclpy not available; skipping.")
        return None
    try:
        bridge = ROS2Bridge(
            node_name=node_name,
            base_frame=base_frame,
            topic_prefix=topic_prefix,
        )
        return bridge
    except Exception as e:
        logger.warning("Failed to create ROS2 bridge: %s", e)
        return None
