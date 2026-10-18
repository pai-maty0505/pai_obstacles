#!/usr/bin/env python3
"""
障害物 3 種類（scripts/obstacle_shapes.py）を置ける範囲を、MoveIt だけで測る（issue #5 の「置ける範囲」）。

障害物は運ぶ道（円弧）の上に置き、置く角度 shoulder_pan を少しずつ変えて、次の 3 つを調べる。
床・把持台・お皿も MoveIt に置く。運ぶ間は物体を持った状態（指は挟む角度、物体は Attached Collision Object）で判定する。

  1. ふさぐ   ：障害物がないときの運び方（carry_start → carry_goal を shoulder_pan だけ回す）が、
                 本物の大きさの障害物に当たる
  2. 邪魔しない：つかむ・置く動作のすべての姿勢が、余裕 C を含む障害物と当たらない
  3. 避けられる：物を持ったまま、余裕 C を含む障害物を避けて carry_start から carry_goal へ行く経路を OMPL が見つける
                 （見つけた経路は、間を細かく補間して全点を確かめる）

    python3 scripts/measure_placement_range.py [--step 0.05] [--from 0.2] [--to 1.4]

結果は data/placement_range.json に保存する。
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan, GetPositionFK

REPO_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_DIR / "scripts"))
import mission_scene  # noqa: E402
import obstacle_shapes  # noqa: E402
from compute_mission_poses import set_floor  # noqa: E402
from run_mlp_gazebo_demo import GROUP_NAME, MLPDemoNode  # noqa: E402

POSES = mission_scene.POSES
ARM_JOINTS = POSES["arm_joints"]
OUTPUT = REPO_DIR / "data" / "placement_range.json"
OBSTACLE_ID = "placement_obstacle"
STRAIGHT_STEPS = 60

# つかむ・置く動作の姿勢（姿勢、物を持つか、グリッパの角度（持たないとき）、台・お皿の上の物体）
HANDLING = [
    ("carry_start", False, POSES["open_angle"], "pick"),
    ("pick", False, POSES["open_angle"], "pick"),
    ("pick", True, None, None),
    ("carry_start", True, None, None),
    ("carry_goal", True, None, None),
    ("place", True, None, None),
    ("place", False, POSES["release_angle"], "place"),
] + [(k, False, POSES["release_angle"], "place") for k in range(len(POSES["post_place_path"]))] + [
    ("post_place", False, POSES["release_angle"], "place"),
]


def joints_of(name):
    joints = POSES["post_place_path"][name] if isinstance(name, int) else POSES[name]
    return dict(zip(ARM_JOINTS, joints))


class RangeNode(MLPDemoNode):
    def __init__(self, name="measure_placement_range"):
        super().__init__(name)
        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.plan_client.wait_for_service(timeout_sec=10.0)
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.fk_client.wait_for_service(timeout_sec=10.0)

    def set_obstacle(self, shape, pan, clearance):
        mission_scene.apply(self, [obstacle_shapes.moveit_object(OBSTACLE_ID, shape, pan, clearance)])

    def holding(self, value, angle=None):
        self.carried_object = value
        self.gripper_angle = angle

    def straight_blocked(self):
        start, goal = np.array(POSES["carry_start"]), np.array(POSES["carry_goal"])
        self.holding(True)
        for t in np.linspace(0.0, 1.0, STRAIGHT_STEPS + 1):
            if not self.state_is_valid(dict(zip(ARM_JOINTS, start + (goal - start) * t))):
                return True
        return False

    def handling_safe(self):
        bad = []
        for name, holding, angle, where in HANDLING:
            self.holding(holding, angle)
            mission_scene.apply(self, [mission_scene.object_on(where, True) if where
                                       else mission_scene.object_on("pick", False)])
            safe, _ = mission_scene.state_check(self, list(joints_of(name).values()))
            if not safe:
                bad.append(name if isinstance(name, str) else f"post_place_path[{name}]")
        mission_scene.apply(self, [mission_scene.object_on("pick", False)])
        return bad

    def plan(self, attempts=10):
        """物を持ったまま carry_start → carry_goal の経路。見つからなければ None。

        OMPL は同じ条件でもたまたま失敗することがあるので、何回か試す。
        self.failures に、経路が見つからなかった回数と、見つかった経路が細かい確認で衝突した回数を残す。
        """
        self.holding(True)
        self.failures = {"not_found": 0, "invalid": 0}
        for _ in range(attempts):
            request = GetMotionPlan.Request()
            motion = request.motion_plan_request
            motion.group_name = GROUP_NAME
            motion.start_state = self.make_robot_state(joints_of("carry_start"))
            constraints = Constraints()
            for name, value in joints_of("carry_goal").items():
                constraints.joint_constraints.append(JointConstraint(
                    joint_name=name, position=float(value), tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
            motion.goal_constraints = [constraints]
            motion.allowed_planning_time = 3.0
            motion.num_planning_attempts = 10
            motion.pipeline_id = "ompl"
            future = self.plan_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response is None or response.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
                self.failures["not_found"] += 1
                continue
            trajectory = response.motion_plan_response.trajectory.joint_trajectory
            index = [trajectory.joint_names.index(j) for j in ARM_JOINTS]
            points = [{j: float(p.positions[i]) for j, i in zip(ARM_JOINTS, index)} for p in trajectory.points]
            safe, _, _ = self.validate_points(points)
            if safe:
                return points
            self.failures["invalid"] += 1
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--step", type=float, default=0.05)
    parser.add_argument("--from", dest="start", type=float, default=0.20)
    parser.add_argument("--to", dest="stop", type=float, default=1.40)
    parser.add_argument("--clearance", type=float, default=obstacle_shapes.CLEARANCE)
    args = parser.parse_args()

    rclpy.init()
    node = RangeNode()
    set_floor(node, True)
    mission_scene.apply(node, mission_scene.moveit_objects(True))
    pans = np.round(np.arange(args.start, args.stop + 1e-9, args.step), 3)
    results = {}
    t0 = time.monotonic()
    for shape in obstacle_shapes.SHAPES:
        label = obstacle_shapes.SHAPES[shape]["label"]
        print(f"===== {label} =====", flush=True)
        rows = []
        for pan in pans:
            node.set_obstacle(shape, float(pan), 0.0)
            blocked = node.straight_blocked()
            node.set_obstacle(shape, float(pan), args.clearance)
            bad = node.handling_safe()
            path = node.plan() if blocked and not bad else None
            peak = None
            if path is not None:
                # 経路の中で物体が最も高くなるところ（障害物を上から越えたか、脇を回ったかの目安）
                peak = max(node_center_height(node, p) for p in path)
            ok = blocked and not bad and path is not None
            failures = getattr(node, "failures", None) if blocked and not bad else None
            rows.append({"pan": float(pan), "blocked": blocked, "handling_unsafe": bad,
                         "path_found": path is not None, "plan_failures": failures, "peak_height": peak, "ok": ok})
            x, y = obstacle_shapes.on_arc(float(pan))
            print(f"  pan {pan:4.2f}（中心 {x:+.3f}, {y:+.3f}）：ふさぐ {'○' if blocked else '×'}  "
                  f"邪魔しない {'○' if not bad else '×（' + '、'.join(bad[:3]) + '）'}  "
                  f"避けられる {'○' if path is not None else ('×' if blocked and not bad else '―')}"
                  f"{f'（物体の最高 {peak:.3f} m）' if peak else ''}"
                  f"{f'（失敗：見つからない {failures['not_found']}、衝突 {failures['invalid']}）' if failures and path is None else ''}  → {'置ける' if ok else '置けない'}", flush=True)
        results[shape] = rows
        usable = [r["pan"] for r in rows if r["ok"]]
        print(f"  {label}を置ける角度：{usable}", flush=True)

    mission_scene.apply(node, mission_scene.moveit_objects(False)
                        + [obstacle_shapes.moveit_object(OBSTACLE_ID, "box", 0.0, add=False)])
    set_floor(node, False)
    OUTPUT.write_text(json.dumps({"clearance": args.clearance, "results": results}, indent=2, ensure_ascii=False))
    print(f"保存しました：{OUTPUT}（{time.monotonic() - t0:.0f} 秒）")
    node.destroy_node()
    rclpy.shutdown()
    return 0


def node_center_height(node, joints):
    """物体の中心の高さ（FK）。"""
    from compute_grasp_poses import rotation
    from sensor_msgs.msg import JointState
    request = GetPositionFK.Request()
    request.header.frame_id = "base_link"
    request.fk_link_names = ["gripper_link"]
    request.robot_state.joint_state = JointState(name=ARM_JOINTS, position=[joints[j] for j in ARM_JOINTS])
    future = node.fk_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    pose = future.result().pose_stamped[0].pose
    center = np.array([pose.position.x, pose.position.y, pose.position.z]) + \
        rotation(pose.orientation) @ np.array(POSES["object_in_gripper"])
    return float(center[2])


if __name__ == "__main__":
    sys.exit(main())
