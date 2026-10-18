#!/usr/bin/env python3
"""
最終デモ（issue #5・#6）の、低い位置のミッションの把持台・お皿・運搬物を Gazebo と MoveIt に置く。

形は SHAPES で 1 か所だけに定義し、Gazebo（SDF）と MoveIt（CollisionObject）の両方を同じ値から作る。

- 把持台（pick_stand）：4 × 4 cm、高さ 4 cm の箱。上面に運搬物を立てて置く
- お皿（place_plate）：直径 6 cm、深さ 1 cm。内側の底は床から 4 cm。
  台は円柱、縁は薄い箱を円く並べて作る（Gazebo・MoveIt とも単純形状だけ）。
  お皿の底（物体が載る面）は薄い箱にする。Gazebo（bullet）の円柱の上面は縁が丸められて完全な平面にならず、
  載せた物体がゆっくり傾いて倒れたため。円柱の台は底の箱より 1 mm 低くする
- 運搬物（carried_object）：grasp_scene.py と同じ（3 × 3 × 4 cm、36 g、摩擦だけで持つ）

位置は data/mission_poses.json の pick_object・place_object から決める。

    python3 scripts/mission_scene.py reset      # 台とお皿を置き（なければ）、運搬物を把持台に置き直す
    python3 scripts/mission_scene.py remove     # 把持台・お皿・運搬物を片付ける（Gazebo と MoveIt）
    python3 scripts/mission_scene.py check      # 台とお皿を置いた MoveIt で、つかむ・置く姿勢を確かめる
"""

import json
import math
import sys
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_DIR / "scripts"))
import grasp_scene  # noqa: E402

POSES_FILE = REPO_DIR / "data" / "mission_poses.json"
POSES = json.loads(POSES_FILE.read_text())

PICK_STAND_NAME = "pick_stand"
PLATE_NAME = "place_plate"
OBJECT_NAME = grasp_scene.OBJECT_NAME

SURFACE = POSES["surface_height"]     # 把持台の上面・お皿の内側の底の高さ [m]
STAND_SIZE_XY = 0.04
PLATE_DIAMETER = 0.06                 # お皿の外径 [m]
PLATE_DEPTH = 0.01                    # お皿の深さ（底から縁の上まで） [m]
RIM_THICKNESS = 0.003                 # 縁の厚さ [m]
RIM_SEGMENTS = 16                     # 縁を作る箱の数
FLOOR_SIZE = 0.038                    # お皿の底の箱の一辺（内径 5.4 cm の円に収まる正方形） [m]
FLOOR_THICKNESS = 0.004


def shapes():
    """各モデルの形。{モデル名: (中心の xy, [(種類, 寸法, 相対位置 xyz, z 回りの角度, 色)])}"""
    px, py, _ = POSES["pick_object"]
    qx, qy, _ = POSES["place_object"]
    stand = ("box", (STAND_SIZE_XY, STAND_SIZE_XY, SURFACE), (0.0, 0.0, SURFACE / 2.0), 0.0, "gray")
    # お皿：台（円柱）の上面が底。縁は底の上に PLATE_DEPTH の高さ
    pedestal_height = SURFACE - 0.001
    pedestal = ("cylinder", (PLATE_DIAMETER / 2.0, pedestal_height), (0.0, 0.0, pedestal_height / 2.0), 0.0, "white")
    # 底の箱は、置く物体と同じ向き（目標側の shoulder_pan）にそろえる
    floor = ("box", (FLOOR_SIZE, FLOOR_SIZE, FLOOR_THICKNESS), (0.0, 0.0, SURFACE - FLOOR_THICKNESS / 2.0),
             POSES["goal_pan"], "white")
    rim = []
    radius = PLATE_DIAMETER / 2.0 - RIM_THICKNESS / 2.0
    length = 2.0 * radius * math.tan(math.pi / RIM_SEGMENTS) + 0.001   # 隣とわずかに重ねて隙間をなくす
    for k in range(RIM_SEGMENTS):
        a = 2.0 * math.pi * k / RIM_SEGMENTS
        rim.append(("box", (RIM_THICKNESS, length, PLATE_DEPTH),
                    (radius * math.cos(a), radius * math.sin(a), SURFACE + PLATE_DEPTH / 2.0), a, "white"))
    return {
        PICK_STAND_NAME: ((px, py), [stand]),
        PLATE_NAME: ((qx, qy), [pedestal, floor] + rim),
    }


SCENE_CONTACT_TOPIC = "/pai/scene_contacts"   # 把持台・お皿に触れたもの（運搬物以外が触れたら不許可）

COLORS = {"gray": "0.55 0.55 0.6 1", "white": "0.92 0.92 0.88 1"}


def model_sdf(name, parts):
    body = []
    for i, (kind, size, (x, y, z), yaw, color) in enumerate(parts):
        if kind == "box":
            geometry = f"<box><size>{size[0]} {size[1]} {size[2]}</size></box>"
        else:
            geometry = f"<cylinder><radius>{size[0]}</radius><length>{size[1]}</length></cylinder>"
        friction = grasp_scene.OBJECT_FRICTION
        body.append(f"""
      <collision name="collision_{i}">
        <pose>{x} {y} {z} 0 0 {yaw}</pose>
        <geometry>{geometry}</geometry>
        <surface><friction><ode><mu>{friction}</mu><mu2>{friction}</mu2></ode></friction></surface>
      </collision>
      <visual name="visual_{i}">
        <pose>{x} {y} {z} 0 0 {yaw}</pose>
        <geometry>{geometry}</geometry>
        <material><ambient>{COLORS[color]}</ambient><diffuse>{COLORS[color]}</diffuse></material>
      </visual>""")
    collisions = "".join(f"<collision>collision_{i}</collision>" for i in range(len(parts)))
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">{''.join(body)}
      <sensor name="contact" type="contact">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <contact>{collisions}<topic>{SCENE_CONTACT_TOPIC}</topic></contact>
      </sensor>
    </link>
  </model>
</sdf>
"""


def moveit_objects(add=True):
    """MoveIt の CollisionObject（台とお皿）。"""
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive

    objects = []
    for name, ((cx, cy), parts) in shapes().items():
        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = "base_link"
        if not add:
            obj.operation = CollisionObject.REMOVE
            objects.append(obj)
            continue
        for kind, size, (x, y, z), yaw, _ in parts:
            if kind == "box":
                primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size])
            else:
                # MoveIt の円柱の寸法は（高さ、半径）の順
                primitive = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[float(size[1]), float(size[0])])
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = cx + x, cy + y, z
            pose.orientation.z, pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
            obj.primitives.append(primitive)
            obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        objects.append(obj)
    return objects


def object_on(where, add=True):
    """台やお皿の上にある運搬物を、MoveIt のワールドの物体として表す（where：pick / place）。"""
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive

    obj = CollisionObject()
    obj.id = "object_on_stand"
    obj.header.frame_id = "base_link"
    if not add:
        obj.operation = CollisionObject.REMOVE
        return obj
    sx, sy, sz = grasp_scene.OBJECT_SIZE
    x, y, z = POSES[f"{where}_object"]
    # 運搬物は指に沿った向き（腕の向き）に立っている
    yaw = 0.0 if where == "pick" else POSES["goal_pan"]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[sx, sy, sz])]
    obj.primitive_poses = [pose]
    obj.operation = CollisionObject.ADD
    return obj


def apply(node, objects):
    import rclpy
    from moveit_msgs.msg import PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene

    scene = PlanningScene(is_diff=True)
    scene.world.collision_objects = list(objects)
    future = node.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, future)


def gazebo_reset():
    names = grasp_scene.model_names()
    for name, ((cx, cy), parts) in shapes().items():
        if name not in names:
            grasp_scene.spawn(name, model_sdf(name, parts), cx, cy, 0.0)
    grasp_scene.remove(OBJECT_NAME)
    import time
    time.sleep(0.3)
    x, y, z = POSES["pick_object"]
    # 台の上面から 2 mm 上に置いて、自然に落として載せる
    grasp_scene.spawn(OBJECT_NAME, grasp_scene.object_sdf(), x, y, z + 0.002)
    time.sleep(1.0)
    print(f"把持台（上面 {SURFACE:.3f} m）とお皿を置き、運搬物を把持台に置きました：中心 ({x:.3f}, {y:.3f}, {z:.3f})")


# 規則で許されている接触：運搬物と、把持台・お皿（台の上に載っている状態）
ALLOWED_CONTACTS = [("carried_object", PICK_STAND_NAME), ("carried_object", PLATE_NAME)]


def state_check(node, joints):
    """MoveIt で状態を確かめる。許されている接触だけなら SAFE とし、その旨を返す。"""
    import rclpy
    from moveit_msgs.srv import GetStateValidity
    from run_mlp_gazebo_demo import GROUP_NAME

    request = GetStateValidity.Request(group_name=GROUP_NAME)
    request.robot_state = node.make_robot_state(dict(zip(POSES["arm_joints"], joints)))
    future = node.validity_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if result.valid:
        return True, ""
    pairs = [(c.contact_body_1, c.contact_body_2, c.depth) for c in result.contacts]

    def allowed(a, b):
        return any((x in a and y in b) or (x in b and y in a) for x, y in ALLOWED_CONTACTS)

    others = [(a, b, d) for a, b, d in pairs if not allowed(a, b)]
    if others:
        return False, "　" + "、".join(f"{a} と {b}（{d * 1000:.2f} mm）" for a, b, d in others[:3])
    return True, "　（運搬物が台・お皿に載っている接触だけ）"


def check():
    """台とお皿を置いた MoveIt で、つかむ・置く動作の各姿勢を確かめる。"""
    import rclpy
    from compute_mission_poses import set_floor
    from run_mlp_gazebo_demo import MLPDemoNode

    rclpy.init()
    node = MLPDemoNode("mission_scene_check")
    set_floor(node, True)
    apply(node, moveit_objects(True))
    open_angle, release_angle = POSES["open_angle"], POSES["release_angle"]
    # （姿勢、物を持つか、グリッパの角度（持たないとき）、運搬物のある場所、説明）
    cases = [
        ("carry_start", False, open_angle, "pick", "構える（指を開く）"),
        ("pick", False, open_angle, "pick", "下ろす（指を開いたまま）"),
        ("pick", True, None, None, "挟む"),
        ("carry_start", True, None, None, "持ち上げる"),
        ("carry_goal", True, None, None, "お皿の上（運ぶ高さ）"),
        ("place", True, None, None, "お皿に下ろす"),
        ("place", False, release_angle, "place", "指を開いて離す"),
    ] + [(f"post_place_path[{k}]", False, release_angle, "place", "真上に上げる（経由点）")
         for k in range(len(POSES.get("post_place_path", [])))] + [
        ("post_place", False, release_angle, "place", "真上に上げる"),
    ]
    all_safe = True
    for pose_name, holding, angle, where, label in cases:
        node.carried_object = holding
        node.gripper_angle = angle
        apply(node, [object_on(where, True) if where else object_on("pick", False)])
        if pose_name.startswith("post_place_path["):
            joints = POSES["post_place_path"][int(pose_name.split("[")[1].rstrip("]"))]
        else:
            joints = POSES[pose_name]
        safe, note = state_check(node, joints)
        all_safe &= safe
        print(f"  {label:20s}（{pose_name}）: {'SAFE' if safe else 'UNSAFE'}{note}", flush=True)
    # MoveIt を元に戻す（台・お皿・運搬物・床を片付ける）
    apply(node, moveit_objects(False) + [object_on("pick", False)])
    set_floor(node, False)
    print(f"つかむ・置く姿勢：{'すべて SAFE' if all_safe else 'UNSAFE あり'}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if all_safe else 1


def main():
    command = sys.argv[1] if len(sys.argv) > 1 else "reset"
    if command == "remove":
        for name in (OBJECT_NAME, PICK_STAND_NAME, PLATE_NAME):
            grasp_scene.remove(name)
        import rclpy
        from run_mlp_gazebo_demo import MLPDemoNode
        rclpy.init()
        node = MLPDemoNode("mission_scene_remove")
        apply(node, moveit_objects(False) + [object_on("pick", False)])
        node.destroy_node()
        rclpy.shutdown()
        print("把持台・お皿・運搬物を片付けました（Gazebo と MoveIt）。")
        return 0
    if command == "check":
        return check()
    gazebo_reset()
    return 0


if __name__ == "__main__":
    sys.exit(main())
