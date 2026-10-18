#!/usr/bin/env python3
"""
最終デモ（L3、issue #5）：物体をつかみ、MLP の回避軌道で障害物を避けて運び、お皿に置く。
左・中央・右の 3 条件を、1 回の実行で順番に自動で動かす。

障害物は円柱・直方体・三角錐の 3 種類で、左・中央・右にランダムに割り当てる（各形を 1 回ずつ）。
障害物の位置は各範囲の中心（shoulder_pan 0.50 / 0.80 / 1.10 rad）。MLP の軌道は、教師データ v6 で学習した
MLP が test の本番用エピソード（is_demo = 1）について推論したもの（data/v6/mlp_test_predictions.csv）。
オンラインの OMPL 再計画はしない。

1 条件（エピソード）の流れ：

  準備（動画に入れない）
    1. 高い姿勢（transit）を通って、把持台の真上へ戻る（前の条件の障害物は表示したまま、指は開く）
    2. 障害物をこの条件の形と位置に置き換え、運搬物を把持台に置き直す
    3. 全区間を MoveIt で確かめる（障害物は余裕 C だけ広げる。床・把持台・お皿・運搬物も置く）
  本番（エピソードの動画）
    4. 真下に下ろして挟む → 持ち上げる → MLP の回避軌道で運ぶ → お皿に下ろす → 指を開いて離す → 真上に上げる
  記録
    5. 運ぶ間の物体のずれ、置いた位置のずれと傾き、エピソードの長さ（シミュレーション時間）

ログの「エピソード開始」「エピソード終了」の行で、動画を条件ごとに切り分ける（scripts/make_video.py）。
物体は動的な剛体で、保持は左右のパッドの摩擦だけ（固定ジョイント・強制 attach・追従・テレポートは使わない）。
運搬物を把持台に置き直すのは、条件と条件の間の準備で、動画には入れない。障害物は実行の最後に削除しない。

    PAI_DATA_DIR=data/v6 scripts/run_episode_recorded.sh --script scripts/run_mission.py -- [--seed 7]
    （1 条件だけ：-- --conditions left --assign left=box）
"""

import argparse
import csv
import os
import random
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionFK
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import grasp_scene  # noqa: E402
import mission_scene  # noqa: E402
import obstacle_shapes  # noqa: E402
from carried_object import GRASP_GRIPPER_ANGLE, GRIPPER_JOINTS  # noqa: E402
from compute_mission_poses import set_floor  # noqa: E402
from grasp_test import object_pose, rotation  # noqa: E402
from measure_placement_range import RangeNode  # noqa: E402
from run_mlp_gazebo_demo import make_duration  # noqa: E402

BASE_DIR = Path(os.environ.get("PAI_DATA_DIR", SCRIPT_DIR.parent / "data" / "v6")).resolve()
PREDICTIONS_FILE = BASE_DIR / "mlp_test_predictions.csv"
POSES = mission_scene.POSES
ARM_JOINTS = POSES["arm_joints"]
CONDITIONS = ["left", "center", "right"]
OBSTACLE_NAME = "mission_obstacle"

# 本番の各動作の時間 [s]（1 エピソード 60 秒未満）
DURATIONS = {
    "descend": 3.0,     # 把持台の真上 → 真下に下ろす
    "close": 1.5,       # 挟む
    "lift": 2.5,        # 持ち上げる
    "place": 3.0,       # お皿の真上 → お皿に下ろす
    "open": 1.0,        # 指を開いて離す
    "rise": 3.0,        # 真上に上げて指を抜く
}
SETTLE = 0.3            # 挟んだ後・開いた後に待つ時間 [s]
MAX_SLIP = 0.010        # 運ぶ間の物体のずれの上限 [m]
MAX_PLACE_ERROR = 0.010
MAX_TILT = 5.0          # [度]


def as_dict(values):
    return {j: float(v) for j, v in zip(ARM_JOINTS, values)}


def sim_time():
    """Gazebo のシミュレーション時間 [s]。"""
    text = subprocess.run(["gz", "topic", "-e", "-t", "/world/default/stats", "-n", "1"],
                          capture_output=True, text=True).stdout
    block = text[text.find("sim_time {"):]
    sec = nsec = 0
    for line in block.splitlines()[1:4]:
        line = line.strip()
        if line.startswith("sec:"):
            sec = int(line.split()[1])
        elif line.startswith("nsec:"):
            nsec = int(line.split()[1])
    return sec + nsec * 1e-9


def load_demo_trajectories(path):
    """本番用エピソードの MLP の軌道：{(形, 条件): (episode_id, 角度, [30 歩])}"""
    episodes = defaultdict(list)
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if int(row.get("is_demo", 0)):
                episodes[int(row["episode_id"])].append(row)
    demos = {}
    for episode_id, rows in episodes.items():
        rows.sort(key=lambda r: int(r["step"]))
        key = (rows[0]["obs_shape"], rows[0]["obstacle_class"])
        demos[key] = (episode_id, float(rows[0]["obs_pan"]),
                      [[float(r[f"pred_{j}"]) for j in ARM_JOINTS] for r in rows])
    return demos


class MissionNode(RangeNode):
    def __init__(self):
        super().__init__("run_mission")
        self.gripper = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self.gripper.wait_for_server(timeout_sec=10.0)

    # ---------------- 動かす ----------------

    def arm(self, points, seconds, label):
        return self.send_trajectory([as_dict(p) for p in points], seconds, label)

    def move_gripper(self, angle, seconds, label):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(joint_names=GRIPPER_JOINTS)
        goal.trajectory.points = [JointTrajectoryPoint(positions=[float(angle)] * len(GRIPPER_JOINTS),
                                                       time_from_start=make_duration(seconds))]
        self.get_logger().info(f"{label} を送信: {seconds:.1f} s")
        future = self.gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result)
        ok = result.result().status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f"{label}: {'成功' if ok else '失敗'}")
        return ok

    def wait(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ---------------- 確かめる ----------------

    def check(self, points, holding, object_on, label):
        """区間を MoveIt で確かめる。holding：物を持っているか（持っていなければ指は開いた形）。
        object_on：台やお皿の上にある運搬物（pick / place / None）。"""
        self.holding(holding, None if holding else POSES["open_angle"])
        mission_scene.apply(self, [mission_scene.object_on(object_on, True) if object_on
                                   else mission_scene.object_on("pick", False)])
        safe, checks, invalid = self.validate_points([as_dict(p) for p in points])
        print(f"  {label}: {'SAFE' if safe else 'UNSAFE'}（{invalid}/{checks} 無効）", flush=True)
        return safe

    def object_in_gripper(self):
        """グリッパ（gripper_link）から見た物体の中心の位置。"""
        state = None
        while state is None:
            self.latest_joint_state = None
            rclpy.spin_once(self, timeout_sec=0.5)
            state = self.latest_joint_state
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.fk_link_names = ["gripper_link"]
        request.robot_state.joint_state = state
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        pose = future.result().pose_stamped[0].pose
        world, _ = object_pose()
        if world is None:
            return None
        return rotation(pose.orientation).T @ (world - np.array([pose.position.x, pose.position.y, pose.position.z]))


def go_to_start(node, obstacle):
    """準備：高い姿勢を通って、把持台の真上（carry_start）へ戻る。指は開いたまま。

    obstacle：いま表示している障害物（(形, 角度)、なければ None）。運搬物はお皿の上か把持台の上にある。"""
    current = node.current_arm_joints()
    current = [current[j] for j in ARM_JOINTS]
    if np.max(np.abs(np.array(current) - np.array(POSES["carry_start"]))) < 0.01:
        return True
    above = list(POSES["transit_start_side"])
    above[0] = current[0]    # 今の shoulder_pan のまま真上へ
    path = [current, above, POSES["transit_start_side"]]
    if obstacle:
        node.set_obstacle(obstacle[0], obstacle[1], obstacle_shapes.CLEARANCE)
    # 運搬物はお皿の上（前の条件の後）にあるとして確かめる
    if not node.check(path, False, "place", "高い姿勢を通って把持台の上へ戻る"):
        return False
    if not node.move_gripper(POSES["open_angle"], 1.0, "準備：グリッパを開く"):
        return False
    return node.arm(path[1:], 8.0, "準備：高い姿勢を通って把持台の上へ戻る")


def run_condition(node, condition, shape, demos, args, previous_obstacle):
    episode_id, pan, mlp = demos[(shape, condition)]
    label = obstacle_shapes.SHAPES[shape]["label"]
    print(f"\n========== 条件 {condition}：{label}（shoulder_pan {pan:.2f} rad、episode {episode_id}）==========", flush=True)

    # ---- 準備（動画に入れない） ----
    if not go_to_start(node, previous_obstacle):
        return None
    grasp_scene.remove(OBSTACLE_NAME)
    obstacle_shapes.spawn_gazebo(OBSTACLE_NAME, shape, pan)
    node.set_obstacle(shape, pan, args.clearance)
    # 運搬物を把持台に置き直す（台とお皿がなければ置く）
    subprocess.run([sys.executable, str(SCRIPT_DIR / "mission_scene.py"), "reset"], check=True)

    print(f"全区間の確認（MoveIt、障害物は {args.clearance * 1000:.0f} mm 広げる）：", flush=True)
    p = POSES
    checks = [
        node.check([p["transit_start_side"], p["carry_start"]], False, "pick", "把持台の真上へ下りる"),
        node.check([p["carry_start"], p["pick"]], False, "pick", "真下に下ろす（指は開く）"),
        node.check([p["pick"], p["carry_start"]], True, None, "持ち上げる（物を持つ）"),
        node.check([p["carry_start"]] + mlp + [p["carry_goal"]], True, None, "MLP の回避軌道（物を持つ）"),
        node.check([p["carry_goal"], p["place"]], True, None, "お皿に下ろす（物を持つ）"),
        node.check([p["place"]] + p["post_place_path"] + [p["post_place"]], False, "place", "指を開いて真上に上げる"),
    ]
    mission_scene.apply(node, [mission_scene.object_on("pick", True)])
    if not all(checks):
        print("安全でない区間があるため、この条件は実行しません。", flush=True)
        return {"condition": condition, "shape": shape, "episode": episode_id, "safe": False, "executed": False}
    # 準備の最後に、把持台の真上へ下りる（指は開いたまま）
    if not node.arm([p["carry_start"]], 3.0, "準備：把持台の真上へ下りる"):
        return None
    if args.preflight_only:
        return {"condition": condition, "shape": shape, "episode": episode_id, "safe": True, "executed": False}

    # ---- 本番（エピソードの動画） ----
    node.wait(1.0)
    start_sim = sim_time()
    node.get_logger().info(f"エピソード開始: {condition} {shape} episode {episode_id}")
    ok = True
    ok &= node.arm([p["pick"]], DURATIONS["descend"], "真下に下ろして、物体を左右の指の間に入れる")
    ok &= node.move_gripper(GRASP_GRIPPER_ANGLE, DURATIONS["close"], "左右のゴムのパッドで挟む（保持は摩擦だけ）")
    node.wait(SETTLE)
    grasped = node.object_in_gripper()
    ok &= node.arm([p["carry_start"]], DURATIONS["lift"], "持ち上げる")
    mission_scene.apply(node, [mission_scene.object_on("pick", False)])
    ok &= node.arm(mlp + [p["carry_goal"]], args.trajectory_seconds, f"MLP の回避軌道で{label}を避けて運ぶ")
    carried = node.object_in_gripper()
    ok &= node.arm([p["place"]], DURATIONS["place"], "お皿に下ろす")
    ok &= node.move_gripper(POSES["release_angle"], DURATIONS["open"], "指を開いて離す")
    node.wait(SETTLE)
    ok &= node.arm(p["post_place_path"] + [p["post_place"]], DURATIONS["rise"], "真上に上げて、指を物体から抜く")
    node.get_logger().info(f"エピソード終了: {condition} {shape} episode {episode_id}")
    episode_seconds = sim_time() - start_sim
    node.wait(1.0)

    # ---- 記録 ----
    placed, tilt = object_pose()
    target = np.array(POSES["place_object"])
    slip = float(np.linalg.norm(carried - grasped)) if grasped is not None and carried is not None else float("inf")
    place_error = float(np.linalg.norm(placed[:2] - target[:2])) if placed is not None else float("inf")
    height_ok = placed is not None and abs(placed[2] - target[2]) < 0.005
    result = {
        "condition": condition, "shape": shape, "episode": episode_id, "safe": True, "executed": bool(ok),
        "seconds": episode_seconds, "slip_mm": slip * 1000, "place_error_mm": place_error * 1000, "tilt_deg": tilt,
        "success": bool(ok) and slip < MAX_SLIP and place_error < MAX_PLACE_ERROR and height_ok
                   and tilt is not None and tilt < MAX_TILT and episode_seconds < 60.0,
    }
    print(f"結果：{'成功' if result['success'] else '失敗'}　エピソードの長さ {episode_seconds:.1f} 秒（シミュレーション時間）、"
          f"運ぶ間のずれ {slip * 1000:.1f} mm、置いた位置のずれ {place_error * 1000:.1f} mm、傾き {tilt:.1f}°", flush=True)
    return result


def parse_assignment(args):
    """条件 → 形。--assign があればそれを使い、なければ乱数で各形を 1 回ずつ割り当てる。"""
    if args.assign:
        assignment = dict(item.split("=") for item in args.assign.split(","))
    else:
        shapes = list(obstacle_shapes.SHAPES)
        random.Random(args.seed).shuffle(shapes)
        assignment = dict(zip(CONDITIONS, shapes))
    return assignment


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=None, help="形の割り当ての乱数の種（省略すると毎回変わる）")
    parser.add_argument("--assign", help="形の割り当てを指定する（例：left=box,center=pyramid,right=cylinder）")
    parser.add_argument("--conditions", default="left,center,right", help="実行する条件（カンマ区切り）")
    parser.add_argument("--clearance", type=float, default=obstacle_shapes.CLEARANCE, help="障害物との余裕 [m]")
    parser.add_argument("--trajectory-seconds", type=float, default=15.0, help="MLP の回避軌道の時間 [s]")
    parser.add_argument("--preflight-only", action="store_true", help="確かめるだけで、本番の動作はしない")
    args = parser.parse_args()
    if args.seed is None and not args.assign:
        args.seed = random.SystemRandom().randrange(1000)

    demos = load_demo_trajectories(PREDICTIONS_FILE)
    assignment = parse_assignment(args)
    conditions = [c for c in args.conditions.split(",") if c]
    print("ミッション：つかむ → MLP の回避軌道で運ぶ → お皿に置く（オンラインの OMPL 再計画なし）", flush=True)
    print(f"データ：{BASE_DIR}　障害物との余裕：{args.clearance * 1000:.0f} mm　MLP 軌道の時間：{args.trajectory_seconds:.0f} s", flush=True)
    print("障害物の割り当て" + (f"（乱数の種 {args.seed}）" if args.seed is not None else "") + "："
          + "、".join(f"{c} = {obstacle_shapes.SHAPES[assignment[c]]['label']}" for c in CONDITIONS if c in assignment),
          flush=True)

    rclpy.init()
    node = MissionNode()
    set_floor(node, True)
    mission_scene.apply(node, mission_scene.moveit_objects(True))
    results = []
    previous = None
    names = grasp_scene.model_names()
    if OBSTACLE_NAME in names:
        print("前の実行の障害物が表示されたままです。準備で置き換えます。", flush=True)
    try:
        for condition in conditions:
            shape = assignment[condition]
            result = run_condition(node, condition, shape, demos, args, previous)
            if result is None:
                print("準備に失敗しました。中止します。", flush=True)
                break
            results.append(result)
            previous = (shape, demos[(shape, condition)][1])
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("\n===== ミッションの集計 =====")
    total = 0.0
    for r in results:
        label = obstacle_shapes.SHAPES[r["shape"]]["label"]
        if r.get("executed"):
            total += r["seconds"]
            print(f"{r['condition']:6s} {label}（episode {r['episode']}）：{'成功' if r['success'] else '失敗'}　"
                  f"{r['seconds']:.1f} 秒、ずれ {r['slip_mm']:.1f} mm、置いた位置 {r['place_error_mm']:.1f} mm、傾き {r['tilt_deg']:.1f}°")
        else:
            print(f"{r['condition']:6s} {label}（episode {r['episode']}）：{'確認のみ（SAFE）' if r['safe'] else '実行しない（UNSAFE）'}")
    if any(r.get("executed") for r in results):
        print(f"エピソードの合計：{total:.1f} 秒（{'180 秒未満' if total < 180 else '180 秒以上'}）")
    if args.preflight_only:
        return 0 if results and all(r["safe"] for r in results) else 1
    return 0 if len(results) == len(conditions) and all(r.get("success") for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
