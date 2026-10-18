#!/usr/bin/env python3
"""
障害物なしで、物体をつかんで持ち上げ、保持し、置き戻す試験（Phase 2）。

  1. グリッパを開き、物体の手前に構える（pre_pick）
  2. 把持台と運搬物を置く（scripts/grasp_scene.py）
  3. 前に進んで物体を左右の指の間に入れ（pick）、グリッパを閉じて挟む
  4. 持ち上げて（start_roll90）、HOLD_SECONDS 秒保持する
  5. 手首を戻して開始姿勢（start）にし、さらに保持する
  6. 台の少し上まで下ろして、グリッパを開き（物体は台にそっと落ちる）、手前に引く（置き戻す）

各段階で「グリッパから見た物体の位置」を測り、挟んだ直後からのずれ（滑り）を記録する。
物体は動的な剛体で、保持は左右のパッドの摩擦だけ。固定ジョイントや追従は使わない。

    scripts/run_episode_recorded.sh --script scripts/grasp_test.py --view close
    scripts/run_episode_recorded.sh --script scripts/grasp_test.py -- --place   # 運んで配置台に置く

--place のときは、6 で置き戻す代わりに、目標姿勢（障害物なしで関節を直線に補間）へ運び、
配置台の上で手首を 90° 回して下ろし、指を開いて置く。物体が配置台の上に立った向きで置かれたかを確かめる。

成功の条件（PLAN.md §3 の目安）：物体が台から 2.5 cm 以上持ち上がり、保持中のずれが 10 mm 未満で、
置き戻した後に物体が台の上に載っていること。
"""

import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionFK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

REPO_DIR = Path(__file__).resolve().parent.parent
POSES = json.loads((REPO_DIR / "data" / "grasp_poses.json").read_text())
ARM_JOINTS = POSES["arm_joints"]
GRIPPER_JOINTS = ["gripper", "gripper_left"]
OBJECT_IN_GRIPPER = np.array(POSES["object_in_gripper"])
HOLD_SECONDS = 5.0
MAX_SLIP = 0.010
MIN_LIFT = 0.025


def duration(seconds):
    return Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def object_pose():
    """運搬物の位置と、上向きの軸が鉛直からどれだけ傾いているか（度）。"""
    text = subprocess.run(["gz", "topic", "-e", "-t", "/world/default/pose/info", "-n", "1"],
                          capture_output=True, text=True).stdout
    start = text.find('name: "carried_object"')
    if start < 0:
        return None, None
    block = text[start:start + 500]
    values = {}
    for section in ("position", "orientation"):
        part = block[block.find(section):]
        for line in part.splitlines()[1:6]:
            line = line.strip()
            if line == "}":
                break
            key, value = line.split(":")
            values[f"{section}.{key}"] = float(value)
    p = np.array([values.get(f"position.{k}", 0.0) for k in "xyz"])
    qx, qy, qz = (values.get(f"orientation.{k}", 0.0) for k in "xyz")
    qw = values.get("orientation.w", 1.0)
    up_z = 1 - 2 * (qx * qx + qy * qy)   # 物体の z 軸の、世界の z 成分
    tilt = float(np.degrees(np.arccos(np.clip(abs(up_z), -1.0, 1.0))))
    return p, tilt


def object_position():
    text = subprocess.run(["gz", "topic", "-e", "-t", "/world/default/pose/info", "-n", "1"],
                          capture_output=True, text=True).stdout
    start = text.find('name: "carried_object"')
    if start < 0:
        return None
    block = text[start:start + 400]
    block = block[block.find("position"):]
    values = {"x": 0.0, "y": 0.0, "z": 0.0}
    for line in block.splitlines()[1:5]:
        line = line.strip()
        for key in values:
            if line.startswith(f"{key}:"):
                values[key] = float(line.split()[1])
    return np.array([values["x"], values["y"], values["z"]])


class GraspTest(Node):
    def __init__(self):
        super().__init__("grasp_test")
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.gripper = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.joint_state = None
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.arm.wait_for_server(timeout_sec=10.0)
        self.gripper.wait_for_server(timeout_sec=10.0)
        self.fk_client.wait_for_service(timeout_sec=10.0)

    def on_joint_state(self, msg):
        self.joint_state = msg

    def move(self, client, joints, positions, seconds, label):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(joint_names=joints)
        goal.trajectory.points = [JointTrajectoryPoint(positions=[float(p) for p in positions],
                                                       time_from_start=duration(seconds))]
        self.get_logger().info(f"{label} を送信: {seconds:.1f} s")
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        ok = result_future.result().status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f"{label}: {'成功' if ok else '失敗'}")
        return ok

    def arm_to(self, name, seconds):
        return self.move(self.arm, ARM_JOINTS, POSES[name], seconds, f"arm→{name}")

    def gripper_to(self, angle, seconds, label):
        return self.move(self.gripper, GRIPPER_JOINTS, [angle, angle], seconds, label)

    def wait(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def object_in_gripper(self):
        """グリッパ（gripper_link）から見た物体の中心の位置。"""
        self.joint_state = None
        while self.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.fk_link_names = ["gripper_link"]
        request.robot_state.joint_state = self.joint_state
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        pose = future.result().pose_stamped[0].pose
        p = np.array([pose.position.x, pose.position.y, pose.position.z])
        world = object_position()
        if world is None:
            return None, None
        return rotation(pose.orientation).T @ (world - p), world


def main():
    place_mode = "--place" in sys.argv[1:]
    rclpy.init()
    node = GraspTest()
    records = []

    def record(stage):
        local, world = node.object_in_gripper()
        records.append((stage, local, world))
        if local is None:
            print(f"[{stage}] 物体が見つかりません", flush=True)
        else:
            print(f"[{stage}] グリッパから見た物体 ({local[0] * 1000:+.1f}, {local[1] * 1000:+.1f}, {local[2] * 1000:+.1f}) mm、"
                  f"世界での高さ {world[2]:.3f} m", flush=True)
        return local, world

    ok = True
    ok &= node.gripper_to(POSES["open_angle"], 2.0, "グリッパを開く")
    ok &= node.arm_to("pre_pick", 4.0)
    print("把持台と運搬物を置きます。", flush=True)
    subprocess.run([sys.executable, str(REPO_DIR / "scripts" / "grasp_scene.py"), "reset"], check=True)
    node.wait(1.0)
    _, initial_world = record("置いた直後")

    ok &= node.arm_to("pick", 3.0)
    ok &= node.gripper_to(POSES["grasp_angle"], 2.0, "グリッパを閉じて挟む")
    node.wait(1.0)
    grasped, _ = record("挟んだ直後")

    ok &= node.arm_to("start_roll90", 3.0)
    lifted, lifted_world = record("持ち上げた直後")
    node.wait(HOLD_SECONDS)
    held, _ = record(f"{HOLD_SECONDS:.0f} 秒保持した後")

    ok &= node.arm_to("start", 3.0)
    node.wait(2.0)
    rolled, _ = record("手首を戻して開始姿勢")

    if place_mode:
        # 目標姿勢へ運び、配置台の上で手首を 90° 回して下ろし、指を開いて置く
        ok &= node.arm_to("goal", 6.0)
        carried, _ = record("目標姿勢")
        ok &= node.arm_to("place_above", 3.0)
        ok &= node.arm_to("place", 3.0)
        ok &= node.gripper_to(POSES["open_angle"], 2.0, "グリッパを開いて離す")
        ok &= node.arm_to("post_place", 3.0)
        node.wait(1.0)
        _, final_world = record("配置台に置いた後")
        final_world, tilt = object_pose()
        target = np.array(POSES["place_object"])
    else:
        ok &= node.arm_to("start_roll90", 3.0)
        # 台の高さぴったりではなく、少し上（release）で指を開いて、物体を台にそっと落として置く
        ok &= node.arm_to("release", 3.0)
        ok &= node.gripper_to(POSES["open_angle"], 2.0, "グリッパを開いて離す")
        ok &= node.arm_to("pre_pick", 3.0)
        node.wait(1.0)
        _, final_world = record("置き戻した後")
        target = initial_world

    node.destroy_node()
    rclpy.shutdown()

    print()
    print("===== 把持の試験の結果 =====")
    checks = []
    if grasped is None or lifted_world is None or final_world is None:
        print("物体の位置を測れませんでした。")
        return 1
    lift = lifted_world[2] - initial_world[2]
    checks.append((f"持ち上げた高さ {lift * 1000:.1f} mm（{MIN_LIFT * 1000:.0f} mm 以上）", lift >= MIN_LIFT))
    stages = [("持ち上げた直後", lifted), ("保持した後", held), ("開始姿勢", rolled)]
    if place_mode:
        stages.append(("目標姿勢", carried))
    for stage, local in stages:
        slip = np.linalg.norm(local - grasped) if local is not None else float("inf")
        checks.append((f"{stage}のずれ {slip * 1000:.1f} mm（{MAX_SLIP * 1000:.0f} mm 未満）", slip < MAX_SLIP))
    placed = abs(final_world[2] - target[2]) < 0.005 and np.linalg.norm(final_world[:2] - target[:2]) < 0.01
    where = "配置台" if place_mode else "把持台"
    checks.append((f"{where}に置いた位置のずれ {np.linalg.norm(final_world - target) * 1000:.1f} mm（台の上、水平のずれ 10 mm 未満）", placed))
    if place_mode:
        checks.append((f"置いた物体の傾き {tilt:.1f}°（5° 未満で立っている）", tilt < 5.0))
    checks.append(("コントローラの動作", bool(ok)))
    for label, passed in checks:
        print(f"  [{'OK' if passed else 'NG'}] {label}")
    success = all(passed for _, passed in checks)
    print("判定：" + ("成功" if success else "失敗"))
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
