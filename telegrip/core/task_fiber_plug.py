"""
Fiber-plug challenge task simulation.

Mimics the Intrinsic "AI for Industry" challenge: plug a fiber cable connector
into a 6mm port. A connector (free rigid body) and a port fixture (static body)
are spawned in the PyBullet world next to the left robot. Grasping is
proximity-based (the robot is kinematically driven, so contact grasping is not
possible): closing the gripper near the connector attaches it to the
end-effector link with a fixed constraint; opening releases it. Insertion is
detected geometrically from the ferrule tip pose relative to the socket.

All poses in the public API (object states, config) are expressed in the LEFT
robot base frame (PyBullet Z-up, base at identity orientation).
"""

import logging
import random
import time
from typing import Dict, List, Optional

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R

logger = logging.getLogger(__name__)

# --- Geometry constants. Keep in sync with scripts/generate_challenge_assets.py ---
CONN_BODY = (0.022, 0.055, 0.018)   # connector body full extents; +Y is insertion axis
FERRULE_LENGTH = 0.028
FERRULE_TIP_Y = CONN_BODY[1] / 2 + FERRULE_LENGTH  # ferrule tip in connector local frame

PLATE = (0.12, 0.08, 0.008)
PANEL = (0.10, 0.020, 0.10)
SOCKET_HEIGHT = 0.06
PANEL_FRONT_Y = 0.012 - PANEL[1] / 2  # front face y in fixture local frame

DEFAULT_TASK_CONFIG = {
    "enabled": True,
    "name": "fiber_plug_6mm",
    # Port fixture pose in left-robot base frame (on the table, facing the robot).
    # The arm's workspace is mostly at negative Y, so the port sits at -Y and is
    # yawed 180 deg so its front face points back toward the robot.
    "port_position": [0.0, -0.24, 0.0],
    "port_yaw_deg": 180.0,
    # Connector spawn: center [x, y] and random range [+-dx, +-dy]
    "spawn_center": [-0.10, -0.16],
    "spawn_range": [0.03, 0.03],
    # Success tolerances
    "position_tolerance": 0.008,      # meters, ferrule tip to socket entry
    "angle_tolerance_deg": 20.0,      # axis misalignment
    "seat_depth": 0.012,              # how far the ferrule seats into the socket
    # Grasp
    "grasp_radius": 0.06,             # meters, EE tip to connector center
    "gripper_closed_threshold": 22.5, # degrees on the gripper joint
}


class FiberPlugTask:
    """PyBullet-side simulation of the fiber-plug challenge."""

    CONNECTOR_MASS = 0.03

    def __init__(self, physics_client, robot_id: int, ee_link_index: int,
                 base_position, task_config: Optional[dict] = None):
        self.physics_client = physics_client
        self.robot_id = robot_id
        self.ee_link_index = ee_link_index
        self.base_position = np.asarray(base_position, dtype=float)

        cfg = dict(DEFAULT_TASK_CONFIG)
        if task_config:
            cfg.update({k: v for k, v in task_config.items() if v is not None})
        self.cfg = cfg
        self.name = cfg.get("name", "fiber_plug_6mm")

        # PyBullet bodies
        self.connector_id: Optional[int] = None
        self.fixture_id: Optional[int] = None
        self.grasp_constraint: Optional[int] = None

        # Task state
        self.success = False
        self.success_time: Optional[float] = None
        self.attempts = 1
        self.start_time = time.time()
        self.initial_connector_pose: Optional[List[float]] = None
        self._prev_gripper_closed = False
        self.is_grasped = False

    # ------------------------------------------------------------------ setup

    def setup(self) -> bool:
        """Create the port fixture and connector bodies."""
        try:
            self._create_fixture()
            self._create_connector()
            self._spawn_connector()
            logger.info("FiberPlugTask ready (port at %s, base frame)", self.cfg["port_position"])
            return True
        except Exception as e:
            logger.error(f"FiberPlugTask setup failed: {e}")
            return False

    def _port_world_pose(self):
        pos = self.base_position + np.asarray(self.cfg["port_position"], dtype=float)
        yaw = np.deg2rad(float(self.cfg.get("port_yaw_deg", 0.0)))
        orn = p.getQuaternionFromEuler([0, 0, yaw])
        return pos, orn

    def _create_fixture(self):
        pos, orn = self._port_world_pose()
        # Compound static body: base plate + upright panel (visual-only here,
        # rendered by the frontend GLB; collisions with the connector are
        # disabled so the geometric insertion check governs the interaction).
        plate_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[PLATE[0] / 2, PLATE[1] / 2, PLATE[2] / 2])
        panel_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[PANEL[0] / 2, PANEL[1] / 2, PANEL[2] / 2])
        self.fixture_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=plate_col,
            basePosition=(pos + np.array([0, 0, PLATE[2] / 2])).tolist(),
            baseOrientation=orn,
            linkMasses=[0],
            linkCollisionShapeIndices=[panel_col],
            linkVisualShapeIndices=[-1],
            linkPositions=[[0, 0.012, PLATE[2] / 2 + PANEL[2] / 2]],
            linkOrientations=[[0, 0, 0, 1]],
            linkInertialFramePositions=[[0, 0, 0]],
            linkInertialFrameOrientations=[[0, 0, 0, 1]],
            linkParentIndices=[0],
            linkJointTypes=[p.JOINT_FIXED],
            linkJointAxis=[[0, 0, 0]],
        )

    def _create_connector(self):
        body_col = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[CONN_BODY[0] / 2, CONN_BODY[1] / 2, CONN_BODY[2] / 2]
        )
        self.connector_id = p.createMultiBody(
            baseMass=self.CONNECTOR_MASS,
            baseCollisionShapeIndex=body_col,
            basePosition=[0, 0, 1.0],  # placed properly by _spawn_connector
        )
        p.changeDynamics(self.connector_id, -1, lateralFriction=0.8, spinningFriction=0.05)

        # The connector only collides with the ground plane: disable collisions
        # with the robots and the fixture so the kinematically-driven arm and
        # the proximity grasp never send it flying.
        self._disable_collisions_with(self.fixture_id)
        for body_idx in range(p.getNumBodies()):
            body_id = p.getBodyUniqueId(body_idx)
            if body_id in (self.connector_id, self.fixture_id):
                continue
            info = p.getBodyInfo(body_id)
            body_name = info[1].decode("utf-8") if info and info[1] else ""
            if "plane" in body_name.lower():
                continue
            self._disable_collisions_with(body_id)

    def _disable_collisions_with(self, other_id: int):
        if other_id is None or other_id == self.connector_id:
            return
        num_links = p.getNumJoints(other_id)
        for link in range(-1, num_links):
            p.setCollisionFilterPair(self.connector_id, other_id, -1, link, enableCollision=0)

    def _spawn_connector(self):
        cx, cy = self.cfg["spawn_center"]
        dx, dy = self.cfg["spawn_range"]
        x = cx + random.uniform(-dx, dx)
        y = cy + random.uniform(-dy, dy)
        yaw = random.uniform(-0.6, 0.6)  # roughly facing the port
        pos = self.base_position + np.array([x, y, CONN_BODY[2] / 2 + 0.001])
        orn = p.getQuaternionFromEuler([0, 0, yaw])
        p.resetBasePositionAndOrientation(self.connector_id, pos.tolist(), orn)
        p.resetBaseVelocity(self.connector_id, [0, 0, 0], [0, 0, 0])
        self.initial_connector_pose = self._base_frame_pose(pos, orn)

    # ------------------------------------------------------------------ reset

    def reset(self):
        """Re-randomize the connector, clear success, count a new attempt."""
        if self.grasp_constraint is not None:
            self._remove_constraint("grasp_constraint")
        # Restore dynamics in case the connector was frozen after a success
        p.changeDynamics(self.connector_id, -1, mass=self.CONNECTOR_MASS)
        self.is_grasped = False
        self.success = False
        self.success_time = None
        self.attempts += 1
        self.start_time = time.time()
        self._spawn_connector()
        logger.info(f"FiberPlugTask reset (attempt {self.attempts})")

    def _remove_constraint(self, attr: str):
        cid = getattr(self, attr)
        if cid is not None:
            try:
                p.removeConstraint(cid)
            except Exception:
                pass
            setattr(self, attr, None)

    # ------------------------------------------------------------------ update

    def update(self, gripper_angle_deg: float):
        """Per-tick update: grasp/release edges and insertion detection.

        Call after the robot joints have been reset to their actual angles.
        """
        if self.connector_id is None:
            return

        gripper_closed = gripper_angle_deg > float(self.cfg["gripper_closed_threshold"])

        if not self.success:
            # Grasp on close edge
            if gripper_closed and not self._prev_gripper_closed and self.grasp_constraint is None:
                self._try_grasp()
            # Release on open edge
            elif not gripper_closed and self._prev_gripper_closed and self.grasp_constraint is not None:
                self._release()

            self._check_insertion()

        self._prev_gripper_closed = gripper_closed

    def _ee_tip_world(self):
        state = p.getLinkState(self.robot_id, self.ee_link_index)
        return np.array(state[0]), np.array(state[1])

    def _try_grasp(self):
        ee_pos, ee_orn = self._ee_tip_world()
        conn_pos, conn_orn = p.getBasePositionAndOrientation(self.connector_id)
        dist = np.linalg.norm(np.array(conn_pos) - ee_pos)
        if dist > float(self.cfg["grasp_radius"]):
            logger.debug(f"Grasp attempt missed: connector {dist * 100:.1f}cm from gripper")
            return

        # Attach with the current relative transform so the connector doesn't snap
        inv_pos, inv_orn = p.invertTransform(ee_pos.tolist(), ee_orn.tolist())
        rel_pos, rel_orn = p.multiplyTransforms(inv_pos, inv_orn, conn_pos, conn_orn)
        self.grasp_constraint = p.createConstraint(
            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=self.ee_link_index,
            childBodyUniqueId=self.connector_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=rel_pos,
            parentFrameOrientation=rel_orn,
            childFramePosition=[0, 0, 0],
            childFrameOrientation=[0, 0, 0, 1],
        )
        p.changeConstraint(self.grasp_constraint, maxForce=100)
        self.is_grasped = True
        logger.info(f"🔗 Connector GRASPED ({dist * 100:.1f}cm from gripper)")

    def _release(self):
        self._remove_constraint("grasp_constraint")
        self.is_grasped = False
        logger.info("🔗 Connector RELEASED")

    def _socket_entry_world(self):
        """World position of the socket entry point and insertion axis (+Y of fixture)."""
        pos, orn = self._port_world_pose()
        rot = R.from_quat(orn)
        entry_local = np.array([0.0, PANEL_FRONT_Y, SOCKET_HEIGHT])
        entry_world = pos + rot.apply(entry_local)
        axis_world = rot.apply(np.array([0.0, 1.0, 0.0]))
        return entry_world, axis_world

    def _check_insertion(self):
        conn_pos, conn_orn = p.getBasePositionAndOrientation(self.connector_id)
        conn_rot = R.from_quat(conn_orn)

        tip_world = np.array(conn_pos) + conn_rot.apply(np.array([0.0, FERRULE_TIP_Y, 0.0]))
        conn_axis = conn_rot.apply(np.array([0.0, 1.0, 0.0]))

        entry_world, insert_axis = self._socket_entry_world()

        pos_error = np.linalg.norm(tip_world - entry_world)
        cos_angle = float(np.clip(np.dot(conn_axis, insert_axis), -1.0, 1.0))
        angle_error_deg = np.degrees(np.arccos(cos_angle))

        if pos_error < float(self.cfg["position_tolerance"]) and \
           angle_error_deg < float(self.cfg["angle_tolerance_deg"]):
            self._latch_success(entry_world, insert_axis)

    def _latch_success(self, entry_world: np.ndarray, insert_axis: np.ndarray):
        self.success = True
        self.success_time = time.time()
        self._remove_constraint("grasp_constraint")
        self.is_grasped = False

        # Snap the connector into the seated pose (ferrule seated in the socket)
        _, port_orn = self._port_world_pose()
        seat_depth = float(self.cfg["seat_depth"])
        seated_pos = entry_world + insert_axis * seat_depth - R.from_quat(port_orn).apply(
            np.array([0.0, FERRULE_TIP_Y, 0.0])
        )
        p.resetBasePositionAndOrientation(self.connector_id, seated_pos.tolist(), port_orn)
        p.resetBaseVelocity(self.connector_id, [0, 0, 0], [0, 0, 0])

        # Freeze it in place (static body) so it stays seated without drift
        p.changeDynamics(self.connector_id, -1, mass=0)

        elapsed = self.success_time - self.start_time
        logger.info(f"✅ FIBER PLUGGED IN! ({elapsed:.1f}s, attempt {self.attempts})")
        print(f"✅ Task SUCCESS: fiber connector seated ({elapsed:.1f}s, attempt {self.attempts})")

    # ------------------------------------------------------------------ state

    def _base_frame_pose(self, world_pos, world_orn) -> List[float]:
        rel = np.array(world_pos) - self.base_position
        return [float(v) for v in rel] + [float(v) for v in world_orn]

    def get_object_states(self) -> List[Dict]:
        """Object poses in the left-robot base frame, for streaming/recording."""
        objects = []
        if self.connector_id is not None:
            pos, orn = p.getBasePositionAndOrientation(self.connector_id)
            pose = self._base_frame_pose(pos, orn)
            objects.append({
                "id": "fiber_connector",
                "position": pose[:3],
                "quaternion": pose[3:],
                "grasped": self.is_grasped,
            })
        if self.fixture_id is not None:
            pos, orn = self._port_world_pose()
            pose = self._base_frame_pose(pos, orn)
            objects.append({
                "id": "port_panel",
                "position": pose[:3],
                "quaternion": pose[3:],
                "static": True,
            })
        return objects

    def get_state(self) -> Dict:
        """Task status for streaming/recording."""
        if self.success and self.success_time is not None:
            elapsed = self.success_time - self.start_time
        else:
            elapsed = time.time() - self.start_time
        return {
            "name": self.name,
            "success": self.success,
            "elapsed_s": round(elapsed, 2),
            "attempts": self.attempts,
            "grasped": self.is_grasped,
        }
