#!/usr/bin/env python3
"""
物体をつかむ動作の関節の角度を、MoveIt の FK / IK で計算して data/grasp_poses.json に保存する。

開始姿勢では指が水平に伸び、上下から挟む向きになっているので、台の上の物体はそのままではつかめない。
そこで手首を 90° 回して横から挟む。物体の中心は手首の回転軸の上にあるので、回しても動かない。

  start_roll90  開始姿勢から手首だけ 90° 回した姿勢（持ち上げた位置）
  pick          start_roll90 から LIFT_HEIGHT だけ下げた姿勢（台の上の物体を挟む位置）
  pre_pick      pick から指の方向に APPROACH_DISTANCE だけ引いた姿勢（グリッパを開いて構える）
  goal          MLP の目標姿勢（教師データ v5 の目標）
  place_above   配置台の 3 cm 上で、手首を 90° 回して横から挟んだ向きの姿勢（goal から移る）
  place         配置台の RELEASE_HEIGHT 上（ここで指を開いて置く）
  post_place    place から指の方向に APPROACH_DISTANCE だけ引いた姿勢
  release       pick から RELEASE_HEIGHT だけ上げた姿勢（ここで指を開き、物体を台にそっと落として置く。
                台の高さぴったりまで下ろすと、台が物体を押し上げ、物体が指の中で傾いて指の箱に当たるため）

    python3 scripts/compute_grasp_poses.py
"""

import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState

REPO_DIR = Path(__file__).resolve().parent.parent
OUTPUT = REPO_DIR / "data" / "grasp_poses.json"

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
START = [0.0, 0.0, 0.0, 0.0, 0.0]
GOAL = [1.60, -0.60, 0.70, -0.30, 0.40]   # 教師データ v5 の目標姿勢（obstacle_dataset_generator_v5.py）
WRIST_ROLL_SIDE = 1.5708     # 横から挟むための手首の角度 [rad]
LIFT_HEIGHT = 0.03           # 物体を台から持ち上げる高さ [m]
APPROACH_DISTANCE = 0.06     # 物体の手前で構える距離 [m]
RELEASE_HEIGHT = 0.004       # 置くときに指を開く高さ（台の上面からの物体の浮き） [m]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from carried_object import GRASP_GRIPPER_ANGLE, OBJECT_IN_GRIPPER_LINK  # noqa: E402

OBJECT_IN_GRIPPER = np.array(OBJECT_IN_GRIPPER_LINK)   # 物体の中心（gripper_link 座標）
OBJECT_HEIGHT = 0.04         # 運搬物の高さ [m]（台に立てて置く）
OPEN_ANGLE = 0.45            # 構えるときのグリッパの角度 [rad]（パッドのすき間 約 6.5 cm）
GRASP_ANGLE = GRASP_GRIPPER_ANGLE   # 挟むときのグリッパの角度 [rad]


class Calculator(Node):
    def __init__(self):
        super().__init__("compute_grasp_poses")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.fk_client.wait_for_service(timeout_sec=10.0)
        self.ik_client.wait_for_service(timeout_sec=10.0)

    def fk(self, joints):
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.fk_link_names = ["gripper_link"]
        request.robot_state.joint_state = JointState(name=ARM_JOINTS, position=[float(v) for v in joints])
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().pose_stamped[0].pose

    def ik(self, pose, seed):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.ik_link_name = "gripper_link"
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.sec = 1
        request.ik_request.robot_state.joint_state = JointState(name=ARM_JOINTS, position=[float(v) for v in seed])
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose = pose
        request.ik_request.pose_stamped = target
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.error_code.val != 1:
            raise RuntimeError(f"IK に失敗しました（error_code {response.error_code.val}）")
        positions = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        return [positions[name] for name in ARM_JOINTS]


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def shifted(pose, delta):
    result = type(pose)()
    result.orientation = pose.orientation
    result.position.x = pose.position.x + float(delta[0])
    result.position.y = pose.position.y + float(delta[1])
    result.position.z = pose.position.z + float(delta[2])
    return result


def main():
    rclpy.init()
    node = Calculator()

    start_roll90 = START[:4] + [WRIST_ROLL_SIDE]
    pose_lifted = node.fk(start_roll90)
    r = rotation(pose_lifted.orientation)
    finger_direction = -r[:, 2]

    pose_pick = shifted(pose_lifted, (0.0, 0.0, -LIFT_HEIGHT))
    pick = node.ik(pose_pick, start_roll90)
    pose_pre = shifted(pose_pick, -finger_direction * APPROACH_DISTANCE)
    pre_pick = node.ik(pose_pre, pick)
    release = node.ik(shifted(pose_pick, (0.0, 0.0, RELEASE_HEIGHT)), pick)

    # IK の結果を FK で確かめる
    def object_center(joints):
        p = node.fk(joints)
        return np.array([p.position.x, p.position.y, p.position.z]) + rotation(p.orientation) @ OBJECT_IN_GRIPPER

    # 置く側：目標の方向（shoulder_pan = GOAL[0]）で、つかむときと同じ「横から挟む」向きにする。
    # 配置台は、目標姿勢で物体があるところの真下。物体の高さは把持台と同じにする
    pick_center = object_center(pick)
    goal_center = object_center(GOAL)
    place_orientation = node.fk([GOAL[0], 0.0, 0.0, 0.0, WRIST_ROLL_SIDE]).orientation
    r_place = rotation(place_orientation)
    place_finger_direction = -r_place[:, 2]

    def gripper_pose_for(center):
        pose = type(pose_pick)()
        pose.orientation = place_orientation
        position = np.array(center) - r_place @ OBJECT_IN_GRIPPER
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
        return pose

    place_center = np.array([goal_center[0], goal_center[1], pick_center[2]])
    seed = [GOAL[0], GOAL[1], GOAL[2], GOAL[3], WRIST_ROLL_SIDE]
    place_above = node.ik(gripper_pose_for(place_center + np.array([0.0, 0.0, LIFT_HEIGHT])), seed)
    place = node.ik(gripper_pose_for(place_center + np.array([0.0, 0.0, RELEASE_HEIGHT])), place_above)
    post_place = node.ik(shifted(gripper_pose_for(place_center + np.array([0.0, 0.0, RELEASE_HEIGHT])),
                                 -place_finger_direction * APPROACH_DISTANCE), place)

    result = {
        "arm_joints": ARM_JOINTS,
        "start": START,
        "start_roll90": start_roll90,
        "pick": pick,
        "pre_pick": pre_pick,
        "release": release,
        "release_height": RELEASE_HEIGHT,
        "open_angle": OPEN_ANGLE,
        "grasp_angle": GRASP_ANGLE,
        "lift_height": LIFT_HEIGHT,
        "object_in_gripper": OBJECT_IN_GRIPPER.tolist(),
        "object_height": OBJECT_HEIGHT,
        "approach_distance": APPROACH_DISTANCE,
        "goal": GOAL,
        "place_above": place_above,
        "place": place,
        "post_place": post_place,
        "pick_object": [round(float(v), 4) for v in object_center(pick)],
        "place_object": [round(float(v), 4) for v in place_center],
        "lifted_object": [round(float(v), 4) for v in object_center(start_roll90)],
    }
    OUTPUT.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    for name in ["start_roll90", "pick", "pre_pick", "release", "goal", "place_above", "place", "post_place"]:
        c = object_center(result[name])
        print(f"{name:13s}: 関節 {np.round(result[name], 3).tolist()}  物体の中心 ({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f})")
    print(f"指が伸びる向き：{np.round(finger_direction, 3).tolist()}、指が閉じる向き：{np.round(r[:, 0], 3).tolist()}")
    print(f"保存しました：{OUTPUT}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
