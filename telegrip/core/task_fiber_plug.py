"""
Fiber-plug challenge task simulation.

Mimics the Intrinsic "AI for Industry" challenge: plug a fiber cable connector
into a 6mm port. A connector (free rigid body) and a port fixture (static body)
are spawned in the PyBullet world next to the left robot. Grasping is
proximity-based (the robot is kinematically driven, so contact grasping is not
possible): closing the gripper near the connector snaps it into a canonical
jaw pose and attaches it to the end-effector with a fixed constraint; opening
releases it. While grasped, geometric panel guidance blocks face penetration
except through the socket. Insertion is latched when the ferrule seats.

All poses in the public API (object states, config) are expressed in the LEFT
robot base frame (PyBullet Z-up, base at identity orientation).
"""

import logging
import random
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R

logger = logging.getLogger(__name__)

# --- Geometry constants. Keep in sync with scripts/generate_challenge_assets.py ---
CONN_BODY = (0.022, 0.055, 0.018)   # connector body full extents; +Y is insertion axis
FERRULE_RADIUS = 0.006
FERRULE_LENGTH = 0.028
FERRULE_TIP_Y = CONN_BODY[1] / 2 + FERRULE_LENGTH  # ferrule tip in connector local frame

PLATE = (0.12, 0.08, 0.008)
PANEL = (0.10, 0.020, 0.10)
SOCKET_HEIGHT = 0.06
SOCKET_RADIUS = 0.0085
PANEL_FRONT_Y = 0.012 - PANEL[1] / 2  # front face y in fixture local frame
# Hole center in panel-link local frame (panel link origin at panel geometric center)
PANEL_HOLE_Z = SOCKET_HEIGHT - (PLATE[2] / 2 + PANEL[2] / 2)

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
    "position_tolerance": 0.015,      # meters, ferrule tip to socket axis (lateral)
    "angle_tolerance_deg": 35.0,      # axis misalignment
    "seat_depth": 0.012,              # how far the ferrule seats into the socket
    "min_insert_depth": -0.01,        # allow latch while still approaching the face
    # Proximity snap: when the tip is this close to the hole (and roughly
    # aligned), auto-seat and mark the challenge complete.
    "snap_distance": 0.03,            # meters, tip-to-entry Euclidean distance
    "snap_radius": 0.02,              # meters, max lateral offset for snap
    "snap_angle_deg": 40.0,           # max axis misalignment for snap
    # Grasp: canonical pose of connector origin in Fixed_Jaw_tip frame.
    # Connector +Y (ferrule) aligns with tip +Y so the white tip faces out of
    # the jaws toward the panel when the gripper approaches the port.
    "grasp_radius": 0.06,             # meters, EE tip to connector center
    "gripper_closed_threshold": 22.5, # degrees on the gripper joint
    "grasp_local_pos": [0.0, 0.01, 0.0],
    "grasp_local_rpy_deg": [0.0, 0.0, 0.0],
    # Once grasped, ignore gripper-open until success/reset (XR trigger release
    # must not drop the connector).
    "latch_gripper_while_grasped": True,
    # Guided insertion (while grasped)
    "guide_radius": 0.02,             # lateral acceptance for socket guidance
    "guide_angle_deg": 40.0,          # max axis misalignment to enter guide mode
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
        self._fixture_collision_enabled = True

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
        """Static plate + upright panel with a socket aperture (compound boxes)."""
        pos, orn = self._port_world_pose()
        plate_col = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[PLATE[0] / 2, PLATE[1] / 2, PLATE[2] / 2]
        )

        # Panel pieces in panel-link local frame (origin at panel center).
        # Leave a square gap of side ~2*SOCKET_RADIUS around the hole so the
        # ferrule can enter; guidance handles circular acceptance while grasped.
        hx, hy, hz = PANEL[0] / 2, PANEL[1] / 2, PANEL[2] / 2
        gap = SOCKET_RADIUS + 0.001
        hole_z = PANEL_HOLE_Z

        z_top = hole_z + gap
        z_bot = hole_z - gap
        top_half = (hz - z_top) / 2
        bot_half = (z_bot - (-hz)) / 2
        side_half_x = (hx - gap) / 2
        mid_half_z = gap  # half-height of left/right pieces spanning the hole band

        shape_types = []
        half_extents = []
        frame_positions = []
        frame_orients = []

        def add_box(half, center):
            shape_types.append(p.GEOM_BOX)
            half_extents.append(list(half))
            frame_positions.append(list(center))
            frame_orients.append([0, 0, 0, 1])

        # Top / bottom full-width slabs
        if top_half > 1e-4:
            add_box([hx, hy, top_half], [0.0, 0.0, z_top + top_half])
        if bot_half > 1e-4:
            add_box([hx, hy, bot_half], [0.0, 0.0, -hz + bot_half])
        # Left / right beside the hole (only across the hole's vertical band)
        if side_half_x > 1e-4 and mid_half_z > 1e-4:
            add_box([side_half_x, hy, mid_half_z], [-(gap + side_half_x), 0.0, hole_z])
            add_box([side_half_x, hy, mid_half_z], [+(gap + side_half_x), 0.0, hole_z])

        panel_col = p.createCollisionShapeArray(
            shapeTypes=shape_types,
            halfExtents=half_extents,
            collisionFramePositions=frame_positions,
            collisionFrameOrientations=frame_orients,
        )

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
        """Dynamic connector: body box + ferrule box along +Y (fits the socket gap)."""
        half_y = CONN_BODY[1] / 2
        # Approximate the ferrule with a thin box so createCollisionShapeArray
        # works across PyBullet builds (cylinder kwargs are not portable).
        ferrule_half = [FERRULE_RADIUS, FERRULE_LENGTH / 2, FERRULE_RADIUS]
        body_col = p.createCollisionShapeArray(
            shapeTypes=[p.GEOM_BOX, p.GEOM_BOX],
            halfExtents=[
                [CONN_BODY[0] / 2, CONN_BODY[1] / 2, CONN_BODY[2] / 2],
                ferrule_half,
            ],
            collisionFramePositions=[
                [0, 0, 0],
                [0, half_y + FERRULE_LENGTH / 2, 0],
            ],
            collisionFrameOrientations=[
                [0, 0, 0, 1],
                [0, 0, 0, 1],
            ],
        )
        self.connector_id = p.createMultiBody(
            baseMass=self.CONNECTOR_MASS,
            baseCollisionShapeIndex=body_col,
            basePosition=[0, 0, 1.0],  # placed properly by _spawn_connector
        )
        p.changeDynamics(self.connector_id, -1, lateralFriction=0.8, spinningFriction=0.05)

        # Collide with the fixture (aperture) and ground; never with the kinematic robot.
        self._fixture_collision_enabled = True
        for body_idx in range(p.getNumBodies()):
            body_id = p.getBodyUniqueId(body_idx)
            if body_id in (self.connector_id, self.fixture_id):
                continue
            info = p.getBodyInfo(body_id)
            body_name = info[1].decode("utf-8") if info and info[1] else ""
            if "plane" in body_name.lower():
                continue
            self._set_collisions_with(body_id, enable=False)

    def _set_collisions_with(self, other_id: int, enable: bool):
        if other_id is None or other_id == self.connector_id:
            return
        flag = 1 if enable else 0
        num_links = p.getNumJoints(other_id)
        for link in range(-1, num_links):
            p.setCollisionFilterPair(self.connector_id, other_id, -1, link, enableCollision=flag)

    def _set_fixture_collision(self, enable: bool):
        if self.fixture_id is None or enable == self._fixture_collision_enabled:
            return
        self._set_collisions_with(self.fixture_id, enable=enable)
        self._fixture_collision_enabled = enable

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
        self._set_fixture_collision(True)
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
        """Per-tick update: grasp/release edges, panel guidance, insertion.

        Call after the robot joints have been reset to their actual angles.
        """
        if self.connector_id is None:
            return

        gripper_closed = gripper_angle_deg > float(self.cfg["gripper_closed_threshold"])

        if not self.success:
            if self.grasp_constraint is None:
                # Snap only while the XR trigger is held (gripper closed) and in range.
                if gripper_closed:
                    self._try_grasp()
            # Release on open edge — skipped while latching so XR trigger release
            # keeps the connector attached until success or task reset.
            elif (not gripper_closed and self._prev_gripper_closed
                  and self.grasp_constraint is not None
                  and not self.cfg.get("latch_gripper_while_grasped", True)):
                self._release()

            if self.is_grasped:
                self._apply_panel_guidance()

            if not self.success:
                self._check_insertion()

        self._prev_gripper_closed = gripper_closed

    def _ee_tip_world(self):
        state = p.getLinkState(self.robot_id, self.ee_link_index)
        return np.array(state[0]), np.array(state[1])

    def _canonical_grasp_frames(self) -> Tuple[List[float], List[float]]:
        """Parent-frame pose of the connector origin in Fixed_Jaw_tip."""
        local_pos = [float(v) for v in self.cfg["grasp_local_pos"]]
        rpy = np.deg2rad(np.asarray(self.cfg["grasp_local_rpy_deg"], dtype=float))
        local_orn = list(p.getQuaternionFromEuler(rpy.tolist()))
        return local_pos, local_orn

    def _try_grasp(self):
        ee_pos, ee_orn = self._ee_tip_world()
        conn_pos, _ = p.getBasePositionAndOrientation(self.connector_id)
        dist = np.linalg.norm(np.array(conn_pos) - ee_pos)
        if dist > float(self.cfg["grasp_radius"]):
            logger.debug(f"Grasp attempt missed: connector {dist * 100:.1f}cm from gripper")
            return

        # Snap into the canonical jaw pose, then lock with a fixed constraint.
        parent_pos, parent_orn = self._canonical_grasp_frames()
        world_pos, world_orn = p.multiplyTransforms(
            ee_pos.tolist(), ee_orn.tolist(), parent_pos, parent_orn
        )
        p.resetBasePositionAndOrientation(self.connector_id, world_pos, world_orn)
        p.resetBaseVelocity(self.connector_id, [0, 0, 0], [0, 0, 0])

        self.grasp_constraint = p.createConstraint(
            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=self.ee_link_index,
            childBodyUniqueId=self.connector_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=parent_pos,
            parentFrameOrientation=parent_orn,
            childFramePosition=[0, 0, 0],
            childFrameOrientation=[0, 0, 0, 1],
        )
        p.changeConstraint(self.grasp_constraint, maxForce=100)
        self.is_grasped = True
        if self.cfg.get("latch_gripper_while_grasped", True):
            # Keep task state aligned with the forced closed jaw in the control loop.
            self._prev_gripper_closed = True
        # Geometric guidance owns panel interaction while grasped.
        self._set_fixture_collision(False)
        logger.info(f"🔗 Connector GRASPED (canonical jaw snap, was {dist * 100:.1f}cm away)")

    def _release(self):
        self._remove_constraint("grasp_constraint")
        self.is_grasped = False
        self._set_fixture_collision(True)
        logger.info("🔗 Connector RELEASED")

    def _attach_at_world_pose(self, world_pos, world_orn):
        """Recreate the grasp constraint so the connector stays at world_pos/orn."""
        ee_pos, ee_orn = self._ee_tip_world()
        inv_pos, inv_orn = p.invertTransform(ee_pos.tolist(), ee_orn.tolist())
        rel_pos, rel_orn = p.multiplyTransforms(inv_pos, inv_orn, world_pos, world_orn)
        self._remove_constraint("grasp_constraint")
        p.resetBasePositionAndOrientation(self.connector_id, world_pos, world_orn)
        p.resetBaseVelocity(self.connector_id, [0, 0, 0], [0, 0, 0])
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

    def _socket_entry_world(self):
        """World position of the socket entry point and insertion axis (+Y of fixture)."""
        pos, orn = self._port_world_pose()
        rot = R.from_quat(orn)
        entry_local = np.array([0.0, PANEL_FRONT_Y, SOCKET_HEIGHT])
        entry_world = pos + rot.apply(entry_local)
        axis_world = rot.apply(np.array([0.0, 1.0, 0.0]))
        return entry_world, axis_world

    def _unconstrained_grasp_world(self):
        """Where the connector would be from the EE + canonical grasp frames."""
        ee_pos, ee_orn = self._ee_tip_world()
        parent_pos, parent_orn = self._canonical_grasp_frames()
        world_pos, world_orn = p.multiplyTransforms(
            ee_pos.tolist(), ee_orn.tolist(), parent_pos, parent_orn
        )
        return np.array(world_pos), np.array(world_orn)

    def _tip_from_pose(self, conn_pos, conn_orn):
        conn_rot = R.from_quat(conn_orn)
        tip_world = np.array(conn_pos) + conn_rot.apply(np.array([0.0, FERRULE_TIP_Y, 0.0]))
        conn_axis = conn_rot.apply(np.array([0.0, 1.0, 0.0]))
        return tip_world, conn_axis

    def _tip_and_axis_world(self):
        conn_pos, conn_orn = p.getBasePositionAndOrientation(self.connector_id)
        tip_world, conn_axis = self._tip_from_pose(conn_pos, conn_orn)
        return tip_world, conn_axis, np.array(conn_pos), np.array(conn_orn)

    def _apply_panel_guidance(self):
        """Block panel-face penetration; guide through the socket when aligned.

        Uses the unconstrained EE-driven grasp pose so guidance tracks the arm
        even before the physics constraint has stepped.
        """
        conn_pos, conn_orn = self._unconstrained_grasp_world()
        tip_world, conn_axis = self._tip_from_pose(conn_pos, conn_orn)
        entry_world, insert_axis = self._socket_entry_world()

        delta = tip_world - entry_world
        depth = float(np.dot(delta, insert_axis))  # >0 means past the entry plane
        lateral_vec = delta - depth * insert_axis
        lateral = float(np.linalg.norm(lateral_vec))

        cos_angle = float(np.clip(np.dot(conn_axis, insert_axis), -1.0, 1.0))
        angle_error_deg = float(np.degrees(np.arccos(cos_angle)))

        guide_radius = float(self.cfg["guide_radius"])
        guide_angle = float(self.cfg["guide_angle_deg"])
        in_guide = lateral <= guide_radius and angle_error_deg <= guide_angle

        tip_dist = float(np.linalg.norm(delta))
        snap_distance = float(self.cfg.get("snap_distance", 0.03))

        if in_guide:
            # Close enough to the hole while guided → seat and complete.
            if tip_dist <= snap_distance:
                self._latch_success(entry_world, insert_axis)
                return
            tip_target = entry_world + depth * insert_axis
            _, port_orn = self._port_world_pose()
            new_orn = list(port_orn)
            new_pos = tip_target - R.from_quat(new_orn).apply(np.array([0.0, FERRULE_TIP_Y, 0.0]))
            self._attach_at_world_pose(new_pos.tolist(), new_orn)
            return

        # Outside the socket: do not allow the tip past the panel front plane.
        if depth > 0.0:
            tip_target = tip_world - depth * insert_axis
            shift = tip_target - tip_world
            new_pos = (conn_pos + shift).tolist()
            self._attach_at_world_pose(new_pos, conn_orn.tolist())
        # else: free motion in front of the panel — leave the canonical grasp constraint alone.

    def _check_insertion(self):
        tip_world, conn_axis, _, _ = self._tip_and_axis_world()
        entry_world, insert_axis = self._socket_entry_world()

        delta = tip_world - entry_world
        depth = float(np.dot(delta, insert_axis))
        lateral_vec = delta - depth * insert_axis
        lateral = float(np.linalg.norm(lateral_vec))
        tip_dist = float(np.linalg.norm(delta))

        cos_angle = float(np.clip(np.dot(conn_axis, insert_axis), -1.0, 1.0))
        angle_error_deg = float(np.degrees(np.arccos(cos_angle)))

        snap_distance = float(self.cfg.get("snap_distance", 0.03))
        snap_radius = float(self.cfg.get("snap_radius", self.cfg["position_tolerance"]))
        snap_angle = float(self.cfg.get("snap_angle_deg", self.cfg["angle_tolerance_deg"]))

        # Proximity snap: tip near the hole + roughly aimed → seat and complete.
        if (tip_dist <= snap_distance and
                lateral <= snap_radius and
                angle_error_deg <= snap_angle):
            self._latch_success(entry_world, insert_axis)
            return

        min_depth = float(self.cfg["min_insert_depth"])
        if (lateral < float(self.cfg["position_tolerance"]) and
                angle_error_deg < float(self.cfg["angle_tolerance_deg"]) and
                depth >= min_depth):
            self._latch_success(entry_world, insert_axis)

    def _latch_success(self, entry_world: np.ndarray, insert_axis: np.ndarray):
        self.success = True
        self.success_time = time.time()
        self._remove_constraint("grasp_constraint")
        self.is_grasped = False
        self._set_fixture_collision(True)

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
