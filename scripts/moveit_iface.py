"""
move_group だけを相手にする軽いノードと、MoveIt の衝突物体（床・障害物・お皿・運搬物）。

教師データの生成は Gazebo を使わず、move_group のサービス（/apply_planning_scene、/check_state_validity、
/compute_fk、/plan_kinematic_path）だけで行う。並列に作るときは、ROS_DOMAIN_ID を分けて move_group を
いくつも起動し、それぞれにこのノードを 1 つずつつなぐ（scripts/generate_parallel.py）。

形の作り方は v2（pai_obstacle_v2 の obstacle_shapes.py・floor_scene.py・carried_object.py）と同じ：
  - 障害物は余裕 C だけ外側に広げる。三角錐は各頂点に半径 C の球（の外接多面体）を置いた凸包
  - お皿は、薄い底板と、薄い箱を円く並べた縁
  - 持っている運搬物は Attached Collision Object（MoveIt の判定のためだけ。物理的な固定ではない）
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints, JointConstraint,
                             MoveItErrorCodes, PlanningScene, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionFK, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive

import spec

GROUP = "arm"
BASE = "base_link"
PAYLOAD_ID = "payload_held"
PAYLOAD_WORLD_ID = "payload_world"
PLATE_ID = "plate"
TOUCH_LINKS = ["gripper_frame_link", "gripper_link", "moving_jaw_so101_v1_link", "left_jaw_link"]
# 運搬物が床・お皿に載っている接触は許す（MoveIt の判定で、ワールドの運搬物と床・お皿が触れているだけの場合）
ALLOWED_PAIRS = [(PAYLOAD_WORLD_ID, "floor"), (PAYLOAD_WORLD_ID, PLATE_ID), (PAYLOAD_ID, "floor"), (PAYLOAD_ID, PLATE_ID)]


def quat_z(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def make_pose(x, y, z, yaw=0.0):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
    _, _, pose.orientation.z, pose.orientation.w = quat_z(yaw)
    return pose


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# ------------------------------------------------------------------ 形

def pyramid_vertices():
    """三角錐の頂点（底面の重心が原点、底面は z = 0、頂点の 1 つは +x 向き）。"""
    s = spec.SHAPES["pyramid"]
    r = s["side"] / math.sqrt(3.0)
    base = [(r * math.cos(a), r * math.sin(a), 0.0) for a in (0.0, 2 * math.pi / 3, 4 * math.pi / 3)]
    return np.array(base + [(0.0, 0.0, s["height"])])


PYRAMID_FACES = [(0, 2, 1), (0, 1, 3), (1, 2, 3), (2, 0, 3)]
_INFLATED = {}


def _sphere_directions():
    t = (1 + math.sqrt(5)) / 2
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
         (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    v = [np.array(p) / np.linalg.norm(p) for p in v]
    points = list(v)
    for i in range(12):
        for j in range(i + 1, 12):
            if np.linalg.norm(v[i] - v[j]) < 1.1:
                m = v[i] + v[j]
                points.append(m / np.linalg.norm(m))
    points = np.array(points)
    from scipy.spatial import ConvexHull
    hull = ConvexHull(points)
    return points, min(-eq[3] for eq in hull.equations)


def inflated_pyramid(clearance):
    """余裕 clearance を含む三角錐の凸包（頂点、三角形）。"""
    key = round(clearance, 6)
    if key in _INFLATED:
        return _INFLATED[key]
    from scipy.spatial import ConvexHull
    base = pyramid_vertices()
    if clearance <= 0.0:
        result = (base, np.array(PYRAMID_FACES))
    else:
        directions, inner = _sphere_directions()
        points = np.array([v + d * clearance / inner for v in base for d in directions])
        hull = ConvexHull(points)
        index = {old: new for new, old in enumerate(hull.vertices)}
        faces = []
        for simplex, eq in zip(hull.simplices, hull.equations):
            a, b, c = (points[i] for i in simplex)
            if np.dot(np.cross(b - a, c - a), eq[:3]) < 0:
                simplex = simplex[[0, 2, 1]]
            faces.append([index[i] for i in simplex])
        result = (points[hull.vertices], np.array(faces))
    _INFLATED[key] = result
    return result


def floor_object(add=True):
    obj = CollisionObject(id="floor")
    obj.header.frame_id = BASE
    if not add:
        obj.operation = CollisionObject.REMOVE
        return obj
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2.0, 2.0, 0.02])]
    obj.primitive_poses = [make_pose(0.0, 0.0, -0.011)]
    obj.operation = CollisionObject.ADD
    return obj


def obstacle_object(object_id, shape, pan, yaw, clearance, add=True, radius=spec.ARC):
    """障害物（shoulder_pan = pan・回転軸から radius の位置、z 軸まわりの向き yaw、余裕 clearance を含む形）。"""
    obj = CollisionObject(id=object_id)
    obj.header.frame_id = BASE
    if not add:
        obj.operation = CollisionObject.REMOVE
        return obj
    x, y = spec.on_arc(pan, radius)
    s = spec.SHAPES[shape]
    if shape == "cylinder":
        h = s["height"] + 2 * clearance
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[h, s["radius"] + clearance])]
        obj.primitive_poses = [make_pose(x, y, s["height"] / 2.0, yaw)]
    elif shape == "box":
        sx, sy, sz = s["size"]
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                         dimensions=[sx + 2 * clearance, sy + 2 * clearance, sz + 2 * clearance])]
        obj.primitive_poses = [make_pose(x, y, sz / 2.0, yaw)]
    else:
        vertices, faces = inflated_pyramid(clearance)
        mesh = Mesh()
        mesh.vertices = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in vertices]
        mesh.triangles = [MeshTriangle(vertex_indices=[int(i) for i in f]) for f in faces]
        obj.meshes = [mesh]
        obj.mesh_poses = [make_pose(x, y, 0.0, yaw)]
    obj.operation = CollisionObject.ADD
    return obj


def plate_parts(pan):
    """お皿の部品（寸法、相対位置、z 回りの角度）。底板は運搬物と同じ向き（-pan）。"""
    inner = spec.PLATE_DIAMETER / 2.0 - spec.PLATE_RIM
    floor_side = 2.0 * inner / math.sqrt(2.0) - 0.001   # 内側の円に収まる正方形
    parts = [((floor_side, floor_side, spec.PLATE_FLOOR), (0.0, 0.0, spec.PLATE_FLOOR / 2.0), -pan)]
    radius = spec.PLATE_DIAMETER / 2.0 - spec.PLATE_RIM / 2.0
    n = spec.PLATE_RIM_SEGMENTS
    length = 2.0 * radius * math.tan(math.pi / n) + 0.001
    for k in range(n):
        a = 2.0 * math.pi * k / n
        parts.append(((spec.PLATE_RIM, length, spec.PLATE_DEPTH),
                      (radius * math.cos(a), radius * math.sin(a), spec.PLATE_DEPTH / 2.0), a))
    return parts


def plate_object(pan, add=True, radius=spec.ARC):
    obj = CollisionObject(id=PLATE_ID)
    obj.header.frame_id = BASE
    if not add:
        obj.operation = CollisionObject.REMOVE
        return obj
    cx, cy = spec.on_arc(pan, radius)
    for size, (x, y, z), yaw in plate_parts(pan):
        obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size]))
        obj.primitive_poses.append(make_pose(cx + x, cy + y, z, yaw))
    obj.operation = CollisionObject.ADD
    return obj


def payload_world_object(pan, on_plate=False, add=True, radius=spec.ARC):
    """床（またはお皿）の上に立っている運搬物。面はパッドの向きにそろえる（z 軸まわりに -pan）。"""
    obj = CollisionObject(id=PAYLOAD_WORLD_ID)
    obj.header.frame_id = BASE
    if not add:
        obj.operation = CollisionObject.REMOVE
        return obj
    x, y = spec.on_arc(pan, radius)
    z = spec.PAYLOAD_SIZE[2] / 2.0 + (spec.PLATE_FLOOR if on_plate else 0.0)
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(spec.PAYLOAD_SIZE))]
    obj.primitive_poses = [make_pose(x, y, z, -pan)]
    obj.operation = CollisionObject.ADD
    return obj


def held_payload():
    """持っている運搬物（gripper_link に付ける。パッドの中心より GRASP_ABOVE だけ指先側に中心がある）。"""
    att = AttachedCollisionObject(link_name="gripper_link", touch_links=list(TOUCH_LINKS))
    att.object.id = PAYLOAD_ID
    att.object.header.frame_id = "gripper_link"
    px, py, pz = spec.PAD_CENTER_IN_GRIPPER_LINK
    # gripper_link の -z が指先の向き。運搬物の高さ（4 cm）を指に沿わせる
    att.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(spec.PAYLOAD_SIZE))]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = px, py, pz - spec.GRASP_ABOVE
    pose.orientation.w = 1.0
    att.object.primitive_poses = [pose]
    att.object.operation = CollisionObject.ADD
    return att


# ------------------------------------------------------------------ ノード

class MoveItClient(Node):
    def __init__(self, name="pai_v3_moveit_client"):
        super().__init__(name)
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.validity_client = self.create_client(GetStateValidity, "/check_state_validity")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        for client in (self.scene_client, self.validity_client, self.fk_client, self.plan_client):
            if not client.wait_for_service(timeout_sec=30.0):
                raise RuntimeError(f"{client.srv_name} が利用できません")
        # 判定の条件：holding = 運搬物を持っている（指は grasp_angle）、gripper_angle = 持たないときの指の角度
        self.holding = False
        self.grasp_angle = 0.10
        self.gripper_angle = None
        # True のときだけ、運搬物が床・お皿に載っている接触を許す（つかむ・置く区間）。運ぶ区間では許さない
        self.allow_support = False

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def apply(self, objects):
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = list(objects)
        response = self._call(self.scene_client, ApplyPlanningScene.Request(scene=scene))
        return response is not None and response.success

    def robot_state(self, q, gripper=None):
        state = RobotState()
        names, positions = list(spec.ARM_JOINTS), [float(v) for v in q]
        angle = self.grasp_angle if self.holding else (gripper if gripper is not None else self.gripper_angle)
        if angle is not None:
            names += spec.GRIPPER_JOINTS
            positions += [float(angle)] * len(spec.GRIPPER_JOINTS)
        state.joint_state = JointState(name=names, position=positions)
        if self.holding:
            state.attached_collision_objects = [held_payload()]
        return state

    def check(self, q, gripper=None):
        """(SAFE か, 許されていない接触の一覧)。運搬物が床・お皿に載っているだけの接触は許す。"""
        request = GetStateValidity.Request(group_name=GROUP)
        request.robot_state = self.robot_state(q, gripper)
        response = self._call(self.validity_client, request)
        if response is None:
            return False, [("no response", "", 0.0)]
        if response.valid:
            return True, []
        bad = []
        for c in response.contacts:
            a, b = c.contact_body_1, c.contact_body_2
            if self.allow_support and any((x in a and y in b) or (x in b and y in a) for x, y in ALLOWED_PAIRS):
                continue
            bad.append((a, b, float(c.depth)))
        return not bad, bad

    def valid(self, q):
        return self.check(q)[0]

    def fk(self, q, link="gripper_link"):
        request = GetPositionFK.Request()
        request.header.frame_id = BASE
        request.fk_link_names = [link]
        request.robot_state.joint_state = JointState(name=list(spec.ARM_JOINTS), position=[float(v) for v in q])
        pose = self._call(self.fk_client, request).pose_stamped[0].pose
        return np.array([pose.position.x, pose.position.y, pose.position.z]), rotation(pose.orientation)

    def payload_center(self, q):
        """持っている運搬物の中心（base_link）と、gripper_link の回転。"""
        p, r = self.fk(q)
        px, py, pz = spec.PAD_CENTER_IN_GRIPPER_LINK
        return p + r @ np.array([px, py, pz - spec.GRASP_ABOVE]), r

    def plan(self, start, goal, planning_time=5.0, attempts=10):
        """start → goal の OMPL の経路（関節の列）。手首は回さない（経路の条件）。見つからなければ None。"""
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = GROUP
        motion.start_state = self.robot_state(start)
        goal_c = Constraints()
        for name, value in zip(spec.ARM_JOINTS, goal):
            goal_c.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(value), tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        motion.goal_constraints = [goal_c]
        motion.path_constraints = Constraints(joint_constraints=[JointConstraint(
            joint_name="wrist_roll", position=spec.WRIST_ROLL, tolerance_above=0.005, tolerance_below=0.005,
            weight=1.0)])
        motion.allowed_planning_time = planning_time
        motion.num_planning_attempts = attempts
        motion.pipeline_id = "ompl"
        response = self._call(self.plan_client, request)
        if response is None or response.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        traj = response.motion_plan_response.trajectory.joint_trajectory
        index = [traj.joint_names.index(j) for j in spec.ARM_JOINTS]
        raw = [[float(p.positions[i]) for i in index] for p in traj.points]
        raw[0], raw[-1] = list(start), list(goal)
        roll = spec.ARM_JOINTS.index("wrist_roll")
        for q in raw:
            q[roll] = spec.WRIST_ROLL
        return raw
