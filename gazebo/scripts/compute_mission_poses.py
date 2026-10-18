#!/usr/bin/env python3
"""
最終デモ（issue #5・#6）の、低い位置で運ぶミッションの関節の角度を計算して data/mission_poses.json に保存する。

- 物体の中心を床から CARRY_HEIGHT（下端は 7 cm）に保ち、指を水平にして運ぶ
- 手首（wrist_roll）は最初から最後まで約 90° のまま（左右から挟む向き）。エピソードの中で手首は回さない。
  ちょうど 90°（1.5708 rad）では、グリッパの開閉の軸が鉛直から約 2.8° 傾き、パッドの面も傾いて
  挟んだ物体が傾く（置いたときに一辺で立って倒れる）。そこで、開閉の軸が鉛直になる角度を計算して使う
- 把持台は開始側（shoulder_pan = 0）、お皿は目標側（shoulder_pan = GOAL_PAN）。どちらも土台から RADIUS の距離

MoveIt の IK は 5 関節の SO-101 では低い位置で解が見つからないので、腕の面の中で考え、
手首（wrist_flex）は「指が水平になる」角度にし、肩（shoulder_lift）と肘（elbow_flex）を FK を使ったニュートン法で解く。

左右の指は水平に並んでいるので、物体の真上から下ろせば、物体は左右のパッドの間に入る。
（物体の手前から近づく姿勢は、低い位置では手首の角度が可動範囲を超えるため使わない）

  carry_start  把持台の物体の LIFT 上。エピソードの始めは、ここで指を開いて構える。挟んだ後に戻る（MLP の開始姿勢）
  pick         carry_start から真下に LIFT だけ下ろした姿勢（ここで挟む）
  carry_goal   MLP の目標姿勢（carry_start と shoulder_pan だけが違う）。お皿の真上
  place        お皿の底から RELEASE だけ上（ここで指を開いて離す）
  post_place   place から真上に上げ、パッドの下端が物体の上端より上に抜けた姿勢（エピソードの終わり）
               関節を直線に補間すると指先が前後に約 6 mm ずれるので、途中の経由点（post_place_path）も解いておく
  transit      高さ TRANSIT_HEIGHT の高い姿勢（条件の間に、障害物の上を通って戻るのに使う。動画には入れない）

    python3 scripts/compute_mission_poses.py
"""

import json
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

sys.path.insert(0, str(Path(__file__).resolve().parent))
from carried_object import GRASP_GRIPPER_ANGLE, OBJECT_IN_GRIPPER_LINK  # noqa: E402
from compute_grasp_poses import ARM_JOINTS, Calculator, rotation  # noqa: E402
from run_mlp_gazebo_demo import MLPDemoNode  # noqa: E402

OUTPUT = Path(__file__).resolve().parent.parent / "data" / "mission_poses.json"

RADIUS = 0.30            # 土台から物体の中心までの距離 [m]
CARRY_HEIGHT = 0.09      # 運ぶ高さ（物体の中心） [m]
LIFT = 0.03              # 台から持ち上げる高さ [m]
RELEASE = 0.007          # 置くときの、お皿の底からの浮き [m]（指の下端がお皿の縁より上になる高さ。物体は 7 mm 落ちて載る）
POST_PLACE_HEIGHT = 0.13 # 離した後に上げる高さ（物体の中心の位置で）。パッドの下端が物体の上端より 3.5 cm 上 [m]
TRANSIT_HEIGHT = 0.30    # 条件の間に戻るときの高さ（物体の中心の位置で） [m]
GOAL_PAN = 1.60          # 目標側の shoulder_pan [rad]
WRIST_ROLL = 1.5708      # 手首（左右から挟む向き。ずっとこのまま）。main() で開閉の軸が鉛直になる角度に直す
OPEN_ANGLE = 0.30        # 構えるときのグリッパの角度（0.45 では右の指が手首に当たる）
RELEASE_ANGLE = 0.30     # 置いて離すときのグリッパの角度（RELEASE が 7 mm なら、指はお皿の縁に当たらない）
OBJECT_HEIGHT = 0.04
SURFACE_HEIGHT = CARRY_HEIGHT - LIFT - OBJECT_HEIGHT / 2.0   # 把持台の上面とお皿の内側の底の高さ（0.04 m）

OBJ = np.array(OBJECT_IN_GRIPPER_LINK)


class Solver:
    def __init__(self, calc):
        self.calc = calc

    def state(self, joints):
        pose = self.calc.fk(joints)
        r = rotation(pose.orientation)
        center = np.array([pose.position.x, pose.position.y, pose.position.z]) + r @ OBJ
        finger = -r[:, 2]
        # 指の向きを、前（+x、shoulder_pan = 0 のとき土台から外向き）からの上下の角度で測る。
        # 水平でも後ろ向き（土台の方向）の解に飛ばないよう、前後を区別する
        return center, math.atan2(finger[2], finger[0])

    def level(self, lift, elbow, flex=0.0):
        """指が水平になる wrist_flex。"""
        for _ in range(3):
            _, pitch = self.state([0.0, lift, elbow, flex, WRIST_ROLL])
            _, pitch2 = self.state([0.0, lift, elbow, flex + 0.01, WRIST_ROLL])
            flex -= pitch / ((pitch2 - pitch) / 0.01)
        return flex

    def solve(self, radius, height, seed):
        """shoulder_pan = 0 で、物体の中心を (radius, 0, height) に置く関節の角度。"""
        lift, elbow = seed
        for _ in range(60):
            flex = self.level(lift, elbow)
            center, _ = self.state([0.0, lift, elbow, flex, WRIST_ROLL])
            error = np.array([center[0] - radius, center[2] - height])
            if np.linalg.norm(error) < 1e-4:
                return [0.0, lift, elbow, flex, WRIST_ROLL]
            jacobian = np.zeros((2, 2))
            for k, (dl, de) in enumerate([(0.01, 0.0), (0.0, 0.01)]):
                f2 = self.level(lift + dl, elbow + de, flex)
                c2, _ = self.state([0.0, lift + dl, elbow + de, f2, WRIST_ROLL])
                jacobian[:, k] = (np.array([c2[0], c2[2]]) - np.array([center[0], center[2]])) / 0.01
            dl, de = np.linalg.solve(jacobian, -error)
            # 1 回の修正幅を制限して、遠くへ飛ばないようにする
            dl, de = float(np.clip(dl, -0.2, 0.2)), float(np.clip(de, -0.2, 0.2))
            lift, elbow = lift + dl, elbow + de
        raise RuntimeError(f"半径 {radius:.3f} m、高さ {height:.3f} m の姿勢が見つかりません")


def jaw_tilt(calc, joints):
    """グリッパの開閉の軸（指のリンクの z 軸）の、鉛直からの横方向の傾き [rad]。"""
    request = GetPositionFK.Request()
    request.header.frame_id = "base_link"
    request.fk_link_names = ["left_jaw_link"]
    request.robot_state.joint_state = JointState(name=ARM_JOINTS, position=[float(v) for v in joints])
    future = calc.fk_client.call_async(request)
    rclpy.spin_until_future_complete(calc, future)
    r = rotation(future.result().pose_stamped[0].pose.orientation)
    pan = joints[0]
    lateral = np.array([-math.sin(pan), math.cos(pan), 0.0])   # 指の向きに直角な水平の向き
    return math.asin(float(np.dot(r[:, 2], lateral)))


def level_roll(calc, solver, seed):
    """運ぶ姿勢で、開閉の軸が鉛直になる wrist_roll（割線法）。"""
    global WRIST_ROLL
    a, b = 1.5708, 1.5708 + 0.05
    fa = jaw_tilt(calc, solver.solve(RADIUS, CARRY_HEIGHT, seed)[:4] + [a])
    for _ in range(10):
        WRIST_ROLL = b
        fb = jaw_tilt(calc, solver.solve(RADIUS, CARRY_HEIGHT, seed))
        if abs(fb) < 1e-5:
            break
        a, b, fa = b, b - fb * (b - a) / (fb - fa), fb
    return WRIST_ROLL


def set_floor(node, add=True):
    """MoveIt に床（上面が z = 0 の板）を置く／片付ける。"""
    obj = CollisionObject()
    obj.id = "floor"
    obj.header.frame_id = "base_link"
    if add:
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2.0, 2.0, 0.02])]
        pose = Pose()
        pose.position.z = -0.011
        pose.orientation.w = 1.0
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
    else:
        obj.operation = CollisionObject.REMOVE
    scene = PlanningScene(is_diff=True)
    scene.world.collision_objects = [obj]
    future = node.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, future)


def main():
    rclpy.init()
    calc = Calculator()
    checker = MLPDemoNode("compute_mission_poses")
    solver = Solver(calc)

    # 低い姿勢（物体の中心 9 cm、半径 0.30 m）の解を出発点にして、近い姿勢を順に解く
    seed = (0.198, 1.182)
    roll = level_roll(calc, solver, seed)
    print(f"開閉の軸が鉛直になる手首の角度：{roll:.4f} rad（{math.degrees(roll):.2f}°、90° との差 {math.degrees(roll) - 90:+.2f}°）")
    targets = [
        ("carry_start", RADIUS, CARRY_HEIGHT),
        ("pick", RADIUS, CARRY_HEIGHT - LIFT),
        ("place", RADIUS, CARRY_HEIGHT - LIFT + RELEASE),
        ("post_place", RADIUS, POST_PLACE_HEIGHT),
        ("transit", RADIUS, TRANSIT_HEIGHT),
    ]
    planar = {}
    solved = []   # (半径, 高さ, 関節)
    for name, radius, height in targets:
        # すでに解けた姿勢のうち、目標に最も近いものを出発点にする
        if solved:
            _, _, near = min(solved, key=lambda s: math.hypot(s[0] - radius, s[1] - height))
            start = (near[1], near[2])
        else:
            start = seed
        joints = None
        if name == "transit":
            # 高さを少しずつ上げながら解く
            for h in np.linspace(CARRY_HEIGHT, height, 8)[1:]:
                joints = solver.solve(radius, h, start)
                start = (joints[1], joints[2])
        else:
            joints = solver.solve(radius, height, start)
        planar[name] = joints
        solved.append((radius, height, joints))

    # place から post_place までの経由点（1 cm ごと、両端を除く）
    place_height = CARRY_HEIGHT - LIFT + RELEASE
    path = []
    start = (planar["place"][1], planar["place"][2])
    for h in np.arange(place_height + 0.01, POST_PLACE_HEIGHT - 0.005, 0.01):
        joints = solver.solve(RADIUS, float(h), start)
        start = (joints[1], joints[2])
        path.append([GOAL_PAN] + list(joints[1:]))

    goal_side = {"carry_goal": "carry_start", "place": "place", "post_place": "post_place"}
    poses = {}
    for name in ("carry_start", "pick", "transit"):
        poses[name] = list(planar[name])
    for name, base in goal_side.items():
        poses[name] = [GOAL_PAN] + list(planar[base][1:])
    poses["transit_start_side"] = list(planar["transit"])
    poses["transit_goal_side"] = [GOAL_PAN] + list(planar["transit"][1:])
    del poses["transit"]

    def center_of(joints):
        pose = calc.fk(joints)
        return np.array([pose.position.x, pose.position.y, pose.position.z]) + rotation(pose.orientation) @ OBJ

    # 関節の可動範囲（URDF の limit）に収まっているか
    limits = {"shoulder_pan": 1.91986, "shoulder_lift": 1.74533, "elbow_flex": 1.69,
              "wrist_flex": 1.65806, "wrist_roll": 2.74385}
    within = True
    for name, joints in poses.items():
        for joint, value in zip(ARM_JOINTS, joints):
            if abs(value) > limits[joint]:
                print(f"  {name}：{joint} = {value:.3f} rad が可動範囲（±{limits[joint]:.3f}）を超えています")
                within = False

    # 床を置いて、各姿勢が床・自分自身と当たらないことを確かめる
    set_floor(checker, True)
    holding = {"pick", "carry_start", "carry_goal", "place"}
    print("各姿勢の、開閉の軸の傾き：" + "、".join(
        f"{name} {math.degrees(jaw_tilt(calc, joints)):+.2f}°" for name, joints in poses.items()))
    print("姿勢（物体の中心）と、床を置いた MoveIt での確認：")
    all_safe = True
    for name, joints in poses.items():
        checker.carried_object = name in holding
        checker.gripper_angle = None if name in holding else OPEN_ANGLE
        safe = checker.state_is_valid(dict(zip(ARM_JOINTS, joints)))
        all_safe &= safe
        c = center_of(joints)
        print(f"  {name:18s}: 関節 {np.round(joints, 3).tolist()}  中心 ({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f})  "
              f"{'物を持つ' if name in holding else '指を開く'} {'SAFE' if safe else 'UNSAFE'}")
    set_floor(checker, False)

    result = {
        "arm_joints": ARM_JOINTS,
        "radius": RADIUS,
        "carry_height": CARRY_HEIGHT,
        "lift": LIFT,
        "release": RELEASE,
        "post_place_height": POST_PLACE_HEIGHT,
        "goal_pan": GOAL_PAN,
        "wrist_roll": WRIST_ROLL,
        "open_angle": OPEN_ANGLE,
        "release_angle": RELEASE_ANGLE,
        "grasp_angle": GRASP_GRIPPER_ANGLE,
        "object_in_gripper": OBJ.tolist(),
        "object_height": OBJECT_HEIGHT,
        "surface_height": SURFACE_HEIGHT,
        "pick_object": [round(float(v), 4) for v in center_of(poses["pick"]) - np.array([0, 0, 0])],
        "place_object": [round(float(v), 4) for v in center_of(poses["place"]) - np.array([0, 0, RELEASE])],
        **{name: [float(v) for v in joints] for name, joints in poses.items()},
        "post_place_path": [[float(v) for v in joints] for joints in path],
    }
    OUTPUT.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"離した後に上げる経由点：{len(path)} 点")
    print(f"把持台の上面・お皿の底の高さ：{SURFACE_HEIGHT:.3f} m")
    print(f"物体の置き場所：把持台 {result['pick_object']}、お皿 {result['place_object']}")
    print(f"保存しました：{OUTPUT}")
    checker.destroy_node()
    calc.destroy_node()
    rclpy.shutdown()
    print(f"関節の可動範囲：{'すべて範囲内' if within else '範囲外あり'}")
    return 0 if all_safe and within else 1


if __name__ == "__main__":
    sys.exit(main())
