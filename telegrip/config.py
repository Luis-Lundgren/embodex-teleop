"""
Configuration module for the unified teleoperation system.
Loads configuration from config.yaml file with fallback to default values.
"""

import os
import yaml
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import numpy as np
from pathlib import Path
import logging
from .utils import get_absolute_path, get_project_root

logger = logging.getLogger(__name__)

# Default configuration values (fallback if YAML file doesn't exist)
DEFAULT_CONFIG = {
    "network": {
        "port": int(os.environ.get("PORT", 8000)),
        "host_ip": "0.0.0.0"
    },
    "ssl": {
        "certfile": "cert.pem",
        "keyfile": "key.pem"
    },
    "robot": {
        "left_arm": {
            "name": "Left Arm",
            "port": "/dev/ttyACM0",
            "enabled": True
        },
        "right_arm": {
            "name": "Right Arm",
            "port": "/dev/ttyACM1",
            "enabled": True
        },
        "vr_to_robot_scale": 1.0,
        "send_interval": 0.05,
    },
    "control": {
        "keyboard": {
            "enabled": True,
            "pos_step": 0.01,
            "angle_step": 5.0,
            "gripper_step": 10.0
        },
        "vr": {
            "enabled": True
        },
        "pybullet": {
            "enabled": True
        }
    },
    "paths": {
        "urdf_path": "URDF/SO100/so100.urdf"
    },
    "gripper": {
        "open_angle": 0.0,
        "closed_angle": 45.0
    },
    "ik": {
        "use_reference_poses": True,
        "reference_poses_file": "reference_poses.json",
        "position_error_threshold": 0.001,
        "hysteresis_threshold": 0.01,
        "movement_penalty_weight": 0.01
    },
    "ros2": {
        "enabled": False,
        "base_frame": "teleop_base",
        "node_name": "telegrip_bridge",
        "topic_prefix": "telegrip"
    },
    "task": {
        "enabled": True,
        "name": "fiber_plug_6mm",
        # Poses/offsets in the LEFT robot base frame (PyBullet Z-up, meters)
        # Port fixture: raised (+Z) and farther (-Y) for a more extended arm reach.
        "port_position": [0.0, -0.30, 0.05],
        "port_yaw_deg": 180.0,
        "spawn_center": [-0.08, -0.14],
        "spawn_range": [0.03, 0.03],
        "position_tolerance": 0.015,
        "angle_tolerance_deg": 35.0,
        "seat_depth": 0.012,
        "min_insert_depth": -0.01,
        "snap_distance": 0.03,
        "snap_radius": 0.02,
        "snap_angle_deg": 40.0,
        "grasp_radius": 0.06,
        "gripper_closed_threshold": 22.5,
        "grasp_local_pos": [0.0, 0.01, 0.0],
        "grasp_local_rpy_deg": [0.0, 0.0, 0.0],
        "latch_gripper_while_grasped": True,
        "guide_radius": 0.02,
        "guide_angle_deg": 40.0,
    }
}

def load_config(config_path: str = "config.yaml") -> dict:
    """Load configuration from YAML file with fallback to defaults."""
    config = DEFAULT_CONFIG.copy()
    
    # Try to load from project root first (package installation directory)
    package_config_path = get_absolute_path(config_path)
    
    # Check if config exists in package directory
    if package_config_path.exists():
        config_file_to_use = package_config_path
        logger.info(f"Loading config from package directory: {config_file_to_use}")
    # Fallback to current working directory (for user-provided configs)
    elif os.path.exists(config_path):
        config_file_to_use = Path(config_path)
        logger.info(f"Loading config from current directory: {config_file_to_use}")
    else:
        logger.info(f"Config file {config_path} not found in package directory ({package_config_path}) or current directory, using defaults")
        return config
    
    try:
        with open(config_file_to_use, 'r') as f:
            yaml_config = yaml.safe_load(f)
            if yaml_config:
                # Deep merge yaml config into default config
                _deep_merge(config, yaml_config)
    except Exception as e:
        logger.warning(f"Could not load config from {config_file_to_use}: {e}")
        logger.info("Using default configuration")
    
    return config

def save_config(config: dict, config_path: str = "config.yaml"):
    """Save configuration to YAML file in project root."""
    # Always save to project root directory
    abs_config_path = get_absolute_path(config_path)
    try:
        with open(abs_config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, indent=2)
        return True
    except Exception as e:
        logger.error(f"Error saving config to {abs_config_path}: {e}")
        return False

def _deep_merge(base: dict, update: dict):
    """Deep merge update dict into base dict."""
    for key, value in update.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value

# Load configuration
_config_data = load_config()

# Extract values for backward compatibility
PORT = _config_data["network"].get("port", int(os.environ.get("PORT", 8000)))
HOST_IP = _config_data["network"]["host_ip"]

CERTFILE = _config_data["ssl"]["certfile"]
KEYFILE = _config_data["ssl"]["keyfile"]

VR_TO_ROBOT_SCALE = _config_data["robot"]["vr_to_robot_scale"]
SEND_INTERVAL = _config_data["robot"]["send_interval"]

POS_STEP = _config_data["control"]["keyboard"]["pos_step"]
ANGLE_STEP = _config_data["control"]["keyboard"]["angle_step"]
GRIPPER_STEP = _config_data["control"]["keyboard"]["gripper_step"]

URDF_PATH = _config_data["paths"]["urdf_path"]

GRIPPER_OPEN_ANGLE = _config_data["gripper"]["open_angle"]
GRIPPER_CLOSED_ANGLE = _config_data["gripper"]["closed_angle"]

# URDF jaw joint (joint "6") limits from so100.urdf: -0.2 .. 2.0 rad.
# In the URDF/GLB frame 0 deg = jaw closed and positive angles open it,
# which is the reverse of the hardware command frame (0=open, 45=closed).
GRIPPER_URDF_MIN_DEG = -11.5
GRIPPER_URDF_MAX_DEG = 114.6


def hardware_to_urdf_gripper_deg(hardware_deg: float) -> float:
    """Map hardware gripper degrees (0=open, closed_angle=closed) to URDF/GLB
    jaw joint degrees (0=closed, positive=open) used by PyBullet and the
    web/VR digital twins."""
    urdf_deg = GRIPPER_CLOSED_ANGLE - hardware_deg
    return max(GRIPPER_URDF_MIN_DEG, min(GRIPPER_URDF_MAX_DEG, urdf_deg))

# IK Configuration
USE_REFERENCE_POSES = _config_data["ik"]["use_reference_poses"]
REFERENCE_POSES_FILE = _config_data["ik"]["reference_poses_file"]
IK_POSITION_ERROR_THRESHOLD = _config_data["ik"]["position_error_threshold"]
IK_HYSTERESIS_THRESHOLD = _config_data["ik"]["hysteresis_threshold"]
IK_MOVEMENT_PENALTY_WEIGHT = _config_data["ik"]["movement_penalty_weight"]

# ROS2 bridge (optional)
_ros2_config = _config_data.get("ros2", {})
ENABLE_ROS2 = _ros2_config.get("enabled", False)
ROS2_BASE_FRAME = _ros2_config.get("base_frame", "teleop_base")
ROS2_NODE_NAME = _ros2_config.get("node_name", "telegrip_bridge")
ROS2_TOPIC_PREFIX = _ros2_config.get("topic_prefix", "telegrip")

# --- Joint Configuration ---
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
NUM_JOINTS = len(JOINT_NAMES)
NUM_IK_JOINTS = 3  # Use only first 3 joints for IK (Rotation, Pitch, Elbow)
WRIST_FLEX_INDEX = 3
WRIST_ROLL_INDEX = 4
GRIPPER_INDEX = 5

# Motor configuration for SO100
COMMON_MOTORS = {
    "shoulder_pan": [1, "sts3215"],
    "shoulder_lift": [2, "sts3215"], 
    "elbow_flex": [3, "sts3215"],
    "wrist_flex": [4, "sts3215"],
    "wrist_roll": [5, "sts3215"],
    "gripper": [6, "sts3215"],
}

# URDF joint name mapping
URDF_TO_INTERNAL_NAME_MAP = {
    "1": "shoulder_pan",
    "2": "shoulder_lift",
    "3": "elbow_flex",
    "4": "wrist_flex",
    "5": "wrist_roll",
    "6": "gripper",
}

# --- PyBullet Configuration ---
END_EFFECTOR_LINK_NAME = "Fixed_Jaw_tip"

# --- Keyboard Control ---
POS_STEP = 0.01  # meters
ANGLE_STEP = 5.0 # degrees
GRIPPER_STEP = 10.0 # degrees

# --- Device Ports ---
DEFAULT_FOLLOWER_PORTS = {
    "left": _config_data["robot"]["left_arm"]["port"],
    "right": _config_data["robot"]["right_arm"]["port"]
}

@dataclass
class TelegripConfig:
    """Main configuration class for the teleoperation system."""
    
    # Network settings
    port: int = PORT
    host_ip: str = HOST_IP
    
    # SSL settings
    certfile: str = CERTFILE
    keyfile: str = KEYFILE
    
    # Robot settings
    vr_to_robot_scale: float = VR_TO_ROBOT_SCALE
    send_interval: float = SEND_INTERVAL
    
    # Device ports
    follower_ports: Dict[str, str] = None
    
    # Control flags
    enable_pybullet: bool = True
    enable_pybullet_gui: bool = True
    enable_robot: bool = True
    enable_vr: bool = True
    enable_keyboard: bool = True
    enable_ros2: bool = ENABLE_ROS2
    autoconnect: bool = False
    digital_twin_enabled: bool = False
    log_level: str = "warning"
    
    # Arm settings
    left_arm_enabled: bool = True
    right_arm_enabled: bool = True
    
    # Paths
    urdf_path: str = URDF_PATH
    webapp_dir: str = "webapp"
    
    # IK settings
    use_reference_poses: bool = USE_REFERENCE_POSES
    reference_poses_file: str = REFERENCE_POSES_FILE
    ik_position_error_threshold: float = IK_POSITION_ERROR_THRESHOLD
    ik_hysteresis_threshold: float = IK_HYSTERESIS_THRESHOLD
    ik_movement_penalty_weight: float = IK_MOVEMENT_PENALTY_WEIGHT
    
    # Gripper settings
    gripper_open_angle: float = GRIPPER_OPEN_ANGLE
    gripper_closed_angle: float = GRIPPER_CLOSED_ANGLE

    # ROS2 bridge (canonical transform broker)
    ros2_base_frame: str = ROS2_BASE_FRAME
    ros2_node_name: str = ROS2_NODE_NAME
    ros2_topic_prefix: str = ROS2_TOPIC_PREFIX
    
    # Keyboard control
    pos_step: float = POS_STEP
    angle_step: float = ANGLE_STEP
    gripper_step: float = GRIPPER_STEP

    # Challenge task (fiber plug) settings, populated from config file
    task: Dict = None

    def __post_init__(self):
        # Initialize follower_ports if not set
        if self.follower_ports is None:
            self.follower_ports = {
                "left": _config_data["robot"]["left_arm"]["port"],
                "right": _config_data["robot"]["right_arm"]["port"]
            }
        
        # Ensure ports are not None
        if self.follower_ports["left"] is None:
            self.follower_ports["left"] = "/dev/ttyACM0"
        if self.follower_ports["right"] is None:
            self.follower_ports["right"] = "/dev/ttyACM1"
            
        # Initialize arm enabled flags from config data
        self.left_arm_enabled = _config_data["robot"]["left_arm"].get("enabled", True)
        self.right_arm_enabled = _config_data["robot"]["right_arm"].get("enabled", True)

        # ROS2 bridge from config
        ros2 = _config_data.get("ros2", {})
        self.enable_ros2 = ros2.get("enabled", ENABLE_ROS2)
        self.ros2_base_frame = ros2.get("base_frame", ROS2_BASE_FRAME)
        self.ros2_node_name = ros2.get("node_name", ROS2_NODE_NAME)
        self.ros2_topic_prefix = ros2.get("topic_prefix", ROS2_TOPIC_PREFIX)

        # Challenge task settings (merged file config over defaults)
        if self.task is None:
            task_cfg = dict(DEFAULT_CONFIG["task"])
            task_cfg.update(_config_data.get("task", {}) or {})
            self.task = task_cfg
    
    @property
    def ssl_files_exist(self) -> bool:
        """Check if SSL certificate files exist."""
        cert_path = get_absolute_path(self.certfile)
        key_path = get_absolute_path(self.keyfile)
        return cert_path.exists() and key_path.exists()
    
    def ensure_ssl_certificates(self) -> bool:
        """Ensure SSL certificates exist, generating them if necessary."""
        from .utils import ensure_ssl_certificates
        return ensure_ssl_certificates(self.certfile, self.keyfile)
    
    @property
    def urdf_exists(self) -> bool:
        """Check if URDF file exists."""
        urdf_path = get_absolute_path(self.urdf_path)
        return urdf_path.exists()
    
    @property
    def webapp_exists(self) -> bool:
        """Check if webapp directory exists."""
        webapp_path = get_absolute_path(self.webapp_dir)
        return webapp_path.exists()
    
    def get_absolute_urdf_path(self) -> str:
        """Get absolute path to URDF file."""
        return str(get_absolute_path(self.urdf_path))
    
    def get_absolute_reference_poses_path(self) -> str:
        """Get absolute path to reference poses file."""
        return str(get_absolute_path(self.reference_poses_file))
    
    def get_absolute_ssl_paths(self) -> tuple:
        """Get absolute paths to SSL certificate files."""
        cert_path = str(get_absolute_path(self.certfile))
        key_path = str(get_absolute_path(self.keyfile))
        return cert_path, key_path

def get_config_data():
    """Get the current configuration data."""
    return _config_data.copy()

def update_config_data(new_config: dict):
    """Update the global configuration data."""
    global _config_data
    _config_data = new_config
    
    # Save to file
    save_config(_config_data)

# Global configuration instance
config = TelegripConfig() 
