"""
Recording module: captures VR raw inputs and ROS2-equivalent robot goals
in a format comparable to rosbag2 / LeRobot datasets.

Output layout:
  <record_dir>/
    vr_raw.jsonl          # Raw VR controller packets (timestamp, leftController, rightController)
    robot_goal.parquet     # Same semantics as ROS2: EE pose (x,y,z,qx,qy,qz,qw) + gripper per arm
    robot_goal.csv         # Same columns for easy comparison with ROS2 tools
    meta.json              # FPS, start/end time, episode boundaries
    lerobot_frames.parquet # LeRobot-style frame table: observation.state, action, timestamp
"""

import json
import logging
import math
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation

logger = logging.getLogger(__name__)


# Default folded pose: matches robot_interface.py initial positions
DEFAULT_FOLDED_POSE = [0.0, -100.0, 100.0, 60.0, 0.0, 0.0]


def _pose_from_position_and_wrist(
    position: np.ndarray,
    wrist_roll_deg: float,
    wrist_flex_deg: float,
) -> tuple:
    """Build (x, y, z, qx, qy, qz, qw) in ROS convention from robot-frame position and wrist angles.
    Mirrors telegrip.ros2_bridge logic so recorded goals match ROS2 output."""
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


class TeleopRecorder:
    """
    Records VR raw inputs and control-loop (ROS2-equivalent) robot goals
    for comparison with ROS2/rosbag2 and LeRobot datasets.
    """

    def __init__(
        self,
        record_dir: str | Path,
        fps: float = 20.0,
        episode_id: Optional[int] = None,
        vr_only: bool = False,
        left_enabled: bool = True,
        right_enabled: bool = False,
    ):
        self.record_dir = Path(record_dir)
        self.fps = fps
        self.episode_id = episode_id if episode_id is not None else 0
        self.vr_only = vr_only  # When True, only record VR raw (no robot goal / lerobot frames)
        self.left_enabled_internal = left_enabled
        self.right_enabled_internal = right_enabled
        self._lock = threading.Lock()
        self._start_time: Optional[float] = None
        self._vr_raw: List[Dict[str, Any]] = []
        self._robot_goal_rows: List[Dict[str, Any]] = []
        self._lerobot_frames: List[Dict[str, Any]] = []
        self._running = False

    def start(self, new_dir: Optional[Path] = None):
        """Start a new recording session."""
        with self._lock:
            if new_dir:
                self.record_dir = Path(new_dir)
            self._start_time = time.time()
            self._vr_raw = []
            self._robot_goal_rows = []
            self._lerobot_frames = []
            self._running = True
        self.record_dir.mkdir(parents=True, exist_ok=True)
        logger.info("Recording started: %s", self.record_dir)

    def stop(self):
        """Stop recording and flush to disk."""
        with self._lock:
            self._running = False
        self._flush()

    def is_running(self) -> bool:
        with self._lock:
            return self._running

    def on_vr_raw(self, data: Dict[str, Any], timestamp: Optional[float] = None):
        """Record one raw VR packet (leftController, rightController)."""
        if timestamp is None:
            timestamp = time.time()
        t0 = self._start_time
        if t0 is None:
            return
        rel_sec = timestamp - t0
        with self._lock:
            if not self._running:
                return
            self._vr_raw.append({
                "timestamp": timestamp,
                "timestamp_rel_sec": rel_sec,
                "episode_index": self.episode_id,
                "leftController": data.get("leftController"),
                "rightController": data.get("rightController"),
            })

    def on_robot_goal(
        self,
        left_pose: Optional[tuple],
        right_pose: Optional[tuple],
        left_joints: Optional[np.ndarray],
        right_joints: Optional[np.ndarray],
        left_gripper_closed: bool,
        right_gripper_closed: bool,
        left_actual: Optional[np.ndarray] = None,
        right_actual: Optional[np.ndarray] = None,
        timestamp: Optional[float] = None,
    ):
        """
        Record one control-loop snapshot (same semantics as ROS2 topics).
        left_pose / right_pose: (x, y, z, qx, qy, qz, qw) or None if arm idle.
        left_joints / right_joints: Joint angle vector or None.
        No-op when vr_only is True.
        """
        if self.vr_only:
            return
        if timestamp is None:
            timestamp = time.time()
        t0 = self._start_time
        if t0 is None:
            return
        rel_sec = timestamp - t0
        row = {
            "timestamp": timestamp,
            "timestamp_rel_sec": rel_sec,
            "episode_index": self.episode_id,
            "frame_index": len(self._robot_goal_rows),
        }
        for arm, pose in [("left", left_pose), ("right", right_pose)]:
            if pose is not None:
                row[f"{arm}_x"] = pose[0]
                row[f"{arm}_y"] = pose[1]
                row[f"{arm}_z"] = pose[2]
                row[f"{arm}_qx"] = pose[3]
                row[f"{arm}_qy"] = pose[4]
                row[f"{arm}_qz"] = pose[5]
                row[f"{arm}_qw"] = pose[6]
            else:
                for k in ["x", "y", "z", "qx", "qy", "qz", "qw"]:
                    row[f"{arm}_{k}"] = float("nan")
        
        # Add joints to row for robot_goal logs
        for arm, joints in [("left", left_joints), ("right", right_joints)]:
            if joints is not None:
                for i, val in enumerate(joints):
                    row[f"{arm}_q{i}"] = float(val)

        row["left_gripper_closed"] = left_gripper_closed
        row["right_gripper_closed"] = right_gripper_closed

        with self._lock:
            if not self._running:
                return
            self._robot_goal_rows.append(row)

            # LeRobot-style: flat observation.state (current) and action (target/next)
            left_enabled = getattr(self, "left_enabled_internal", True)
            right_enabled = getattr(self, "right_enabled_internal", False)

            # Use target joints for action, and current actual joints for state
            # If current (actual) is not provided, fallback to target for both
            # (recorded by build_robot_goal_snapshot)
            action = _build_lerobot_action(left_joints, right_joints, left_enabled, right_enabled)

            actual_l = left_joints if left_actual is None else left_actual
            actual_r = right_joints if right_actual is None else right_actual
            obs_state = _build_lerobot_observation_state(actual_l, actual_r, left_enabled, right_enabled)

            # Only record if we have valid non-zero data (avoid empty leading/trailing space)
            has_motion = any(v != 0.0 for v in action) or left_gripper_closed or right_gripper_closed

            if has_motion:
                self._lerobot_frames.append({
                    "timestamp": timestamp,
                    "episode_index": self.episode_id,
                    "frame_index": len(self._lerobot_frames),
                    "observation.state": obs_state,
                    "action": action,
                })

    def _flush(self):
        """Write all buffers to disk."""
        self.record_dir.mkdir(parents=True, exist_ok=True)
        t0 = self._start_time
        t1 = time.time()
        meta = {
            "fps": self.fps,
            "vr_only": self.vr_only,
            "start_time": t0,
            "end_time": t1,
            "duration_sec": t1 - t0 if t0 else 0,
            "episode_index": self.episode_id,
            "num_vr_raw": len(self._vr_raw),
            "num_robot_goal_frames": len(self._robot_goal_rows),
        }

        # vr_raw: JSONL
        vr_path = self.record_dir / "vr_raw.jsonl"
        with open(vr_path, "w") as f:
            for entry in self._vr_raw:
                f.write(json.dumps(entry, default=_json_default) + "\n")
        logger.info("Wrote %s (%d lines)", vr_path, len(self._vr_raw))

        # robot_goal: CSV and Parquet (ROS2-comparable columns)
        if self._robot_goal_rows:
            csv_path = self.record_dir / "robot_goal.csv"
            _write_robot_goal_csv(csv_path, self._robot_goal_rows)
            logger.info("Wrote %s (%d rows)", csv_path, len(self._robot_goal_rows))
            try:
                import pandas as pd
                pq_path = self.record_dir / "robot_goal.parquet"
                pd.DataFrame(self._robot_goal_rows).to_parquet(pq_path, index=False)
                logger.info("Wrote %s", pq_path)
            except ImportError:
                pass
            # LeRobot-style frames
            if self._lerobot_frames:
                try:
                    import pandas as pd
                    lerobot_path = self.record_dir / "lerobot_frames.parquet"
                    df = pd.DataFrame(self._lerobot_frames)
                    # No longer converting to JSON strings; parquet supports list columns
                    df.to_parquet(lerobot_path, index=False)
                    logger.info("Wrote %s", lerobot_path)

                    # --- Motion Exchange JSON Format ---
                    self._save_motion_exchange_json()
                except Exception as e:
                    logger.debug("Could not write lerobot_frames.parquet: %s", e)

        with open(self.record_dir / "meta.json", "w") as f:
            json.dump(meta, f, indent=2)
        logger.info("Recording saved to %s", self.record_dir)

    def _save_motion_exchange_json(self):
        """Saves the recording in the JSON format expected by Motion Exchange."""
        if not self._lerobot_frames:
            return

        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        episode_name = f"teleop_{ts_str}"
        
        # We assume left arm for now as it's the primary focus of TeleGrip
        joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        
        timestamps = []
        joint_positions = []
        gripper_states = []
        
        t0 = self._start_time or self._lerobot_frames[0]["timestamp"]

        for frame in self._lerobot_frames:
            timestamps.append(frame["timestamp"] - t0)
            # action is [j1, j2, j3, j4, j5, j6] (degrees)
            # Gripper state for SO-100 is joint 6 (index 5)
            full_joints = frame["action"]
            if len(full_joints) >= 6:
                joint_positions.append(full_joints[:5]) # j1-j5
                gripper_states.append(full_joints[5])   # j6 (gripper)
            else:
                joint_positions.append(full_joints)
                gripper_states.append(0.0)

        data = {
            "robot": "so100",
            "episodes": [
                {
                    "id": episode_name,
                    "timestamps": timestamps,
                    "joint_names": joint_names[:5],
                    "joint_positions": joint_positions,
                    "gripper": gripper_states,
                    "source": "teleop"
                }
            ]
        }
        
        json_path = self.record_dir / f"{episode_name}.json"
        with open(json_path, "w") as f:
            json.dump(data, f, indent=2, default=_json_default)
        logger.info("Wrote Motion Exchange JSON: %s", json_path)


def _json_default(obj):
    if isinstance(obj, np.floating):
        return float(obj)
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    raise TypeError(type(obj))


def _write_robot_goal_csv(path: Path, rows: List[Dict[str, Any]]):
    if not rows:
        return
    import csv
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()), extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def _build_lerobot_action(
    left_joints: Optional[np.ndarray],
    right_joints: Optional[np.ndarray],
    left_enabled: bool = True,
    right_enabled: bool = False,
) -> List[float]:
    """Single vector: [q0..q5] for single arm or [left_q0..q5, right_q0..q5] for dual arm."""
    out = []
    if left_enabled:
        if left_joints is not None:
            out.extend([float(x) for x in left_joints])
        else:
            out.extend(DEFAULT_FOLDED_POSE)
    
    if right_enabled:
        if right_joints is not None:
            out.extend([float(x) for x in right_joints])
        else:
            out.extend(DEFAULT_FOLDED_POSE)
    return out


def _build_lerobot_observation_state(
    left_joints: Optional[np.ndarray],
    right_joints: Optional[np.ndarray],
    left_enabled: bool = True,
    right_enabled: bool = False,
) -> List[float]:
    """Observation state: same as action (commanded/current joint positions)."""
    return _build_lerobot_action(left_joints, right_joints, left_enabled, right_enabled)


def build_robot_goal_snapshot(control_loop) -> tuple:
    """
    Build (left_pose, right_pose, left_joints, right_joints, left_gripper_closed, right_gripper_closed)
    from ControlLoop state. Includes joint angles for LeRobot playback.
    """
    left_pose = right_pose = None
    left_joints = right_joints = None
    left_gripper = right_gripper = False
    
    # Only look for joints in enabled arms
    arms_to_check = []
    if control_loop.config.left_arm_enabled:
        arms_to_check.append("left")
    if control_loop.config.right_arm_enabled:
        arms_to_check.append("right")

    # Always get current joint angles if interface is available
    if control_loop.robot_interface is not None:
        for arm_name in arms_to_check:
            try:
                joints = control_loop.robot_interface.get_arm_angles(arm_name)
                # In SO-100, gripper is joint 5. > 20 deg is usually closed.
                closed = float(joints[5]) > 20.0
                if arm_name == "left":
                    left_joints = joints
                    left_gripper = closed
                else:
                    right_joints = joints
                    right_gripper = closed
            except Exception:
                pass

    for arm_name in arms_to_check:
        arm_state = control_loop.left_arm if arm_name == "left" else control_loop.right_arm
        if arm_state.mode.value == "position" and arm_state.target_position is not None:
            pos = arm_state.target_position
            roll = arm_state.current_wrist_roll
            flex = arm_state.current_wrist_flex
            pose = _pose_from_position_and_wrist(pos, roll, flex)
            
            if arm_name == "left":
                left_pose = pose
            else:
                right_pose = pose

    return left_pose, right_pose, left_joints, right_joints, left_gripper, right_gripper

