#!/usr/bin/env python3

import os
import argparse
import csv
import json
import math
import subprocess
import tempfile
import time

from collections import defaultdict
from pathlib import Path

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)

from moveit_msgs.msg import (
    CollisionObject,
    PlanningScene,
    RobotState,
)

from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetStateValidity,
)

import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from carried_object import (  # noqa: E402
    GRASP_GRIPPER_ANGLE,
    GRIPPER_JOINTS,
    make_carried_object,
)


# ============================================================
# 基本設定
# ============================================================

# データの置き場所。環境変数 PAI_DATA_DIR で切り替えられる（例：PAI_DATA_DIR=data/v5）
BASE_DIR = Path(
    os.environ.get(
        "PAI_DATA_DIR",
        Path(__file__).resolve().parent.parent / "data",
    )
).resolve()

PREDICTIONS_FILE = (
    BASE_DIR
    / "mlp_test_predictions.csv"
)

DEMO_CANDIDATES_FILE = (
    BASE_DIR
    / "mlp_demo_candidates.csv"
)


ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


# 教師データ生成時と同じ開始姿勢
START_JOINTS = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
}


GROUP_NAME = "arm"
BASE_FRAME = "base_link"

ARM_ACTION = (
    "/arm_controller/"
    "follow_joint_trajectory"
)

GAZEBO_WORLD = "default"

GAZEBO_OBSTACLE_NAME = (
    "mlp_demo_obstacle"
)

MOVEIT_OBSTACLE_ID = (
    "mlp_demo_obstacle"
)

# 障害物の接触センサのトピック（Gazebo）
GAZEBO_OBSTACLE_CONTACT_TOPIC = (
    "/pai/obstacle_contacts"
)


# ============================================================
# 障害物サイズ
# V4.2と同一
# ============================================================

OBSTACLE_SIZE_X = 0.055
OBSTACLE_SIZE_Y = 0.055
OBSTACLE_SIZE_Z = 0.22


# ============================================================
# 安全性チェック
# ============================================================

INTERPOLATION_SUBSTEPS = 5


# ============================================================
# 動作時間
#
# 1エピソード
# リセット 4秒 + AI軌道30秒
# → 約34秒を目標
# ============================================================

DEFAULT_RESET_SECONDS = 4.0
DEFAULT_TRAJECTORY_SECONDS = 30.0

# 前のエピソードの軌道を逆にたどって開始姿勢へ戻る時間
DEFAULT_RETURN_SECONDS = 10.0

# この差（rad）以内なら同じ姿勢とみなす
SAME_POSE_TOLERANCE = 0.05

# 最後に実行したエピソードと障害物の位置（次の実行で戻る経路を決めるのに使う）
STATE_FILE = (
    Path(__file__).resolve().parent.parent
    / "logs"
    / "demo_last_state.json"
)

PAUSE_BETWEEN_EPISODES = 1.0


# ============================================================
# フォールバック用デモ候補
# ============================================================

FALLBACK_EPISODES = {
    "left": 4,
    "center": 173,
    "right": 203,
}


# ============================================================
# Duration生成
# ============================================================

def make_duration(seconds):

    whole = int(seconds)

    nanoseconds = int(
        round(
            (
                seconds
                - whole
            )
            * 1e9
        )
    )

    if nanoseconds >= 1_000_000_000:

        whole += 1
        nanoseconds -= 1_000_000_000

    return Duration(
        sec=whole,
        nanosec=nanoseconds,
    )


# ============================================================
# 予測CSV読み込み
# ============================================================

def load_predictions(path):

    if not path.exists():

        raise FileNotFoundError(
            f"予測ファイルが見つかりません: "
            f"{path}"
        )

    episodes = defaultdict(list)

    with path.open(
        "r",
        newline="",
    ) as f:

        reader = csv.DictReader(f)

        for row in reader:

            episode_id = int(
                row["episode_id"]
            )

            episodes[
                episode_id
            ].append(row)

    for episode_id in episodes:

        episodes[
            episode_id
        ].sort(
            key=lambda row: int(
                row["step"]
            )
        )

    return episodes


# ============================================================
# デモ候補読み込み
# ============================================================

def load_demo_candidates(
    path,
):

    result = {}

    if path.exists():

        with path.open(
            "r",
            newline="",
        ) as f:

            reader = csv.DictReader(f)

            for row in reader:

                obstacle_class = (
                    row[
                        "obstacle_class"
                    ]
                )

                result[
                    obstacle_class
                ] = int(
                    row[
                        "episode_id"
                    ]
                )

    # 足りない場合は既知の安全候補を使用
    for class_name, episode_id in (
        FALLBACK_EPISODES.items()
    ):

        if class_name not in result:

            result[
                class_name
            ] = episode_id

    return result


# ============================================================
# 予測関節角度取得
# ============================================================

def predicted_joints(row):

    return {
        joint: float(
            row[
                f"pred_{joint}"
            ]
        )
        for joint in ARM_JOINTS
    }


# ============================================================
# 2姿勢間の補間
# ============================================================

def interpolate_joints(
    joints_a,
    joints_b,
    alpha,
):

    return {
        joint: (
            joints_a[joint]
            + alpha
            * (
                joints_b[joint]
                - joints_a[joint]
            )
        )
        for joint in ARM_JOINTS
    }


# ============================================================
# Gazebo障害物SDF作成
# ============================================================

def create_obstacle_sdf():

    text = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{GAZEBO_OBSTACLE_NAME}">
    <static>true</static>

    <link name="link">

      <collision name="collision">
        <geometry>
          <box>
            <size>
              {OBSTACLE_SIZE_X}
              {OBSTACLE_SIZE_Y}
              {OBSTACLE_SIZE_Z}
            </size>
          </box>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <box>
            <size>
              {OBSTACLE_SIZE_X}
              {OBSTACLE_SIZE_Y}
              {OBSTACLE_SIZE_Z}
            </size>
          </box>
        </geometry>

        <material>
          <ambient>
            0.8 0.1 0.1 1
          </ambient>

          <diffuse>
            0.8 0.1 0.1 1
          </diffuse>
        </material>
      </visual>

      <!-- 障害物に触れたものを記録する（接触の相手のリンク名が分かる） -->
      <sensor name="contact" type="contact">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <contact>
          <collision>collision</collision>
          <topic>{GAZEBO_OBSTACLE_CONTACT_TOPIC}</topic>
        </contact>
      </sensor>

    </link>
  </model>
</sdf>
"""

    path = Path(
        tempfile.gettempdir()
    ) / "so101_mlp_demo_obstacle.sdf"

    path.write_text(
        text
    )

    return path


# ============================================================
# Gazebo障害物削除
# ============================================================

def remove_gazebo_obstacle():

    command = [
        "gz",
        "service",

        "-s",
        f"/world/"
        f"{GAZEBO_WORLD}/remove",

        "--reqtype",
        "gz.msgs.Entity",

        "--reptype",
        "gz.msgs.Boolean",

        "--timeout",
        "3000",

        "--req",
        (
            f'name: "'
            f'{GAZEBO_OBSTACLE_NAME}'
            f'", type: MODEL'
        ),
    ]

    subprocess.run(
        command,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


# ============================================================
# Gazebo障害物生成
# ============================================================

def spawn_gazebo_obstacle(
    x,
    y,
    z,
):

    remove_gazebo_obstacle()

    sdf_path = (
        create_obstacle_sdf()
    )

    command = [
        "ros2",
        "run",
        "ros_gz_sim",
        "create",

        "--world",
        GAZEBO_WORLD,

        "--file",
        str(
            sdf_path
        ),

        "--name",
        GAZEBO_OBSTACLE_NAME,

        "-x",
        str(x),

        "-y",
        str(y),

        "-z",
        str(z),
    ]

    result = subprocess.run(
        command,
        capture_output=True,
        text=True,
        check=False,
    )

    if result.returncode != 0:

        print(
            "Gazebo 障害物の "
            "生成に失敗しました。"
        )

        print(
            result.stdout
        )

        print(
            result.stderr
        )

        return False

    time.sleep(
        0.5
    )

    return True


# ============================================================
# ROS ノード
# ============================================================

class MLPDemoNode(Node):

    def __init__(
        self,
        node_name="run_mlp_gazebo_demo",
    ):

        super().__init__(
            node_name
        )

        self.scene_client = (
            self.create_client(
                ApplyPlanningScene,
                "/apply_planning_scene",
            )
        )

        self.validity_client = (
            self.create_client(
                GetStateValidity,
                "/check_state_validity",
            )
        )

        self.arm_action = ActionClient(
            self,
            FollowJointTrajectory,
            ARM_ACTION,
        )

        self.latest_joint_state = None

        # 衝突判定の条件（main で引数から設定する）
        #   clearance：障害物を各面この量だけ広げて判定する（m）
        #   carried_object：手先に仮の把持物を付けて判定する
        self.clearance = 0.0

        self.carried_object = False

        # 物を持っていない区間で、指の開き具合を指定して判定するときの角度（None なら指定しない）
        self.gripper_angle = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_state,
            10,
        )

        self.wait_for_interfaces()

    # --------------------------------------------------------
    # インタフェース待ち
    # --------------------------------------------------------

    def wait_for_interfaces(self):

        self.get_logger().info(
            "/apply_planning_scene "
            "を待機中 ..."
        )

        if not (
            self.scene_client
            .wait_for_service(
                timeout_sec=10.0
            )
        ):

            raise RuntimeError(
                "/apply_planning_scene "
                "が利用できません"
            )

        self.get_logger().info(
            "/check_state_validity "
            "を待機中 ..."
        )

        if not (
            self.validity_client
            .wait_for_service(
                timeout_sec=10.0
            )
        ):

            raise RuntimeError(
                "/check_state_validity "
                "が利用できません"
            )

        self.get_logger().info(
            "アームの軌道アクション "
            "を待機中 ..."
        )

        if not (
            self.arm_action
            .wait_for_server(
                timeout_sec=10.0
            )
        ):

            raise RuntimeError(
                f"{ARM_ACTION} "
                "が利用できません"
            )

        self.get_logger().info(
            "デモ用のすべてのインタフェースの "
            "準備ができました。"
        )

    # --------------------------------------------------------
    # 関節状態の受信
    # --------------------------------------------------------

    def on_joint_state(
        self,
        msg,
    ):

        self.latest_joint_state = msg

    # --------------------------------------------------------
    # 現在の関節角度
    # --------------------------------------------------------

    def current_arm_joints(
        self,
    ):

        self.latest_joint_state = None

        while self.latest_joint_state is None:

            rclpy.spin_once(
                self,
                timeout_sec=1.0,
            )

        positions = dict(
            zip(
                self.latest_joint_state.name,
                self.latest_joint_state.position,
            )
        )

        return {
            joint: float(
                positions[joint]
            )
            for joint in ARM_JOINTS
        }

    # --------------------------------------------------------
    # RobotState
    # --------------------------------------------------------

    def make_robot_state(
        self,
        joints,
    ):

        state = RobotState()

        state.joint_state = (
            JointState()
        )

        state.joint_state.name = (
            ARM_JOINTS.copy()
        )

        state.joint_state.position = [
            float(
                joints[joint]
            )
            for joint in ARM_JOINTS
        ]

        if self.carried_object:

            # 物体を持っている間は、指を物体の幅まで開いた形で判定する
            state.joint_state.name = (
                list(state.joint_state.name)
                + GRIPPER_JOINTS
            )

            state.joint_state.position = (
                list(state.joint_state.position)
                + [GRASP_GRIPPER_ANGLE] * len(GRIPPER_JOINTS)
            )

            state.attached_collision_objects = [
                make_carried_object()
            ]

        elif self.gripper_angle is not None:

            state.joint_state.name = (
                list(state.joint_state.name)
                + GRIPPER_JOINTS
            )

            state.joint_state.position = (
                list(state.joint_state.position)
                + [self.gripper_angle] * len(GRIPPER_JOINTS)
            )

        return state

    # --------------------------------------------------------
    # 状態の妥当性確認
    # --------------------------------------------------------

    def state_is_valid(
        self,
        joints,
    ):

        request = (
            GetStateValidity.Request()
        )

        request.robot_state = (
            self.make_robot_state(
                joints
            )
        )

        request.group_name = (
            GROUP_NAME
        )

        future = (
            self.validity_client
            .call_async(
                request
            )
        )

        rclpy.spin_until_future_complete(
            self,
            future,
        )

        response = future.result()

        if response is None:
            return False

        return bool(
            response.valid
        )

    # --------------------------------------------------------
    # MoveIt障害物追加
    # --------------------------------------------------------

    def add_moveit_obstacle(
        self,
        x,
        y,
        z,
        padding=None,
    ):
        # padding：障害物の各面を外側に広げる量（m）。省略すると --clearance の値を使う
        if padding is None:
            padding = self.clearance

        collision_object = (
            CollisionObject()
        )

        collision_object.id = (
            MOVEIT_OBSTACLE_ID
        )

        collision_object.header.frame_id = (
            BASE_FRAME
        )

        primitive = (
            SolidPrimitive()
        )

        primitive.type = (
            SolidPrimitive.BOX
        )

        primitive.dimensions = [
            OBSTACLE_SIZE_X + 2.0 * padding,
            OBSTACLE_SIZE_Y + 2.0 * padding,
            OBSTACLE_SIZE_Z + 2.0 * padding,
        ]

        pose = Pose()

        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)

        pose.orientation.w = 1.0

        collision_object.primitives = [
            primitive
        ]

        collision_object.primitive_poses = [
            pose
        ]

        collision_object.operation = (
            CollisionObject.ADD
        )

        scene = PlanningScene()

        scene.is_diff = True

        scene.world.collision_objects = [
            collision_object
        ]

        request = (
            ApplyPlanningScene.Request()
        )

        request.scene = scene

        future = (
            self.scene_client
            .call_async(
                request
            )
        )

        rclpy.spin_until_future_complete(
            self,
            future,
        )

        response = future.result()

        return (
            response is not None
            and response.success
        )

    # --------------------------------------------------------
    # MoveIt障害物削除
    # --------------------------------------------------------

    def remove_moveit_obstacle(
        self,
    ):

        obj = CollisionObject()

        obj.id = (
            MOVEIT_OBSTACLE_ID
        )

        obj.header.frame_id = (
            BASE_FRAME
        )

        obj.operation = (
            CollisionObject.REMOVE
        )

        scene = PlanningScene()

        scene.is_diff = True

        scene.world.collision_objects = [
            obj
        ]

        request = (
            ApplyPlanningScene.Request()
        )

        request.scene = scene

        future = (
            self.scene_client
            .call_async(
                request
            )
        )

        rclpy.spin_until_future_complete(
            self,
            future,
        )

        return future.result()

    # --------------------------------------------------------
    # 予測軌道の安全性確認
    #
    # START→pred0 も確認
    # pred0→pred1...も確認
    # --------------------------------------------------------

    def validate_prediction(
        self,
        rows,
    ):

        predicted = [
            predicted_joints(row)
            for row in rows
        ]

        return self.validate_points(
            [
                START_JOINTS
            ] + predicted
        )

    # --------------------------------------------------------
    # 関節角度の列を、間を補間しながら衝突判定する
    # --------------------------------------------------------

    def validate_points(
        self,
        all_points,
    ):

        total_checks = 0

        invalid_checks = 0

        for i in range(
            len(all_points) - 1
        ):

            q0 = all_points[i]
            q1 = all_points[i + 1]

            for substep in range(
                INTERPOLATION_SUBSTEPS
            ):

                alpha = (
                    substep
                    / float(
                        INTERPOLATION_SUBSTEPS
                    )
                )

                q = interpolate_joints(
                    q0,
                    q1,
                    alpha,
                )

                total_checks += 1

                if not self.state_is_valid(
                    q
                ):

                    invalid_checks += 1

                    return (
                        False,
                        total_checks,
                        invalid_checks,
                    )

        # 最終点
        total_checks += 1

        if not self.state_is_valid(
            all_points[-1]
        ):

            invalid_checks += 1

        safe = (
            invalid_checks == 0
        )

        return (
            safe,
            total_checks,
            invalid_checks,
        )

    # --------------------------------------------------------
    # FollowJointTrajectory実行
    # --------------------------------------------------------

    def send_trajectory(
        self,
        joint_points,
        total_seconds,
        label,
    ):

        goal = (
            FollowJointTrajectory.Goal()
        )

        trajectory = (
            JointTrajectory()
        )

        trajectory.joint_names = (
            ARM_JOINTS.copy()
        )

        count = len(
            joint_points
        )

        for i, joints in enumerate(
            joint_points
        ):

            point = (
                JointTrajectoryPoint()
            )

            point.positions = [
                float(
                    joints[joint]
                )
                for joint in ARM_JOINTS
            ]

            # 最初の点も0秒にはしない
            t = (
                total_seconds
                * (
                    i + 1
                )
                / count
            )

            point.time_from_start = (
                make_duration(t)
            )

            trajectory.points.append(
                point
            )

        goal.trajectory = trajectory

        self.get_logger().info(
            f"{label} を送信: "
            f"{count} 点, "
            f"{total_seconds:.1f} s"
        )

        send_future = (
            self.arm_action
            .send_goal_async(
                goal
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = (
            send_future.result()
        )

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):

            self.get_logger().error(
                f"{label} のゴールが拒否されました。"
            )

            return False

        result_future = (
            goal_handle
            .get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        wrapped_result = (
            result_future.result()
        )

        if wrapped_result is None:

            return False

        status = (
            wrapped_result.status
        )

        result = (
            wrapped_result.result
        )

        success = (
            status
            == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == 0
        )

        if success:

            self.get_logger().info(
                f"{label}: 成功"
            )

        else:

            self.get_logger().error(
                f"{label}: 失敗 "
                f"status={status}, "
                f"error_code="
                f"{result.error_code}, "
                f"error_string="
                f"{result.error_string}"
            )

        return success

    # --------------------------------------------------------
    # 開始姿勢へ戻す
    # --------------------------------------------------------

    def move_to_start(
        self,
        seconds,
    ):

        return self.send_trajectory(
            [
                START_JOINTS
            ],
            seconds,
            "reset-to-start",
        )

    # --------------------------------------------------------
    # AI軌道実行
    # --------------------------------------------------------

    def execute_prediction(
        self,
        rows,
        seconds,
    ):

        points = [
            predicted_joints(row)
            for row in rows
        ]

        return self.send_trajectory(
            points,
            seconds,
            "MLP-predicted-trajectory",
        )


# ============================================================
# 姿勢の比較
# ============================================================

def joints_close(
    joints_a,
    joints_b,
):

    return all(
        abs(
            joints_a[joint]
            - joints_b[joint]
        )
        <= SAME_POSE_TOLERANCE
        for joint in ARM_JOINTS
    )


# ============================================================
# 最後に実行したエピソードの記録
# ============================================================

def load_last_state():

    try:
        return json.loads(
            STATE_FILE.read_text()
        )
    except (OSError, ValueError):
        return None


def update_last_state(
    **values,
):

    # episode / obstacle：最後に動かしたエピソードとその障害物（戻る経路に使う）
    # displayed_obstacle：いま Gazebo に表示している障害物（なければ None）
    state = load_last_state() or {}

    state.update(values)

    STATE_FILE.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    STATE_FILE.write_text(
        json.dumps(state)
    )


def save_last_state(
    episode_id,
    obs_x,
    obs_y,
    obs_z,
):

    update_last_state(
        episode=episode_id,
        obstacle=[obs_x, obs_y, obs_z],
        data_dir=str(BASE_DIR),
    )


# ============================================================
# MoveIt（RViz）の障害物を、Gazebo の表示と一致させる
#
# 衝突判定では障害物を一時的に広げる（--clearance）ので、判定が終わったら
# 実物の大きさに戻す。Gazebo に表示がなければ MoveIt からも消す。
# ============================================================

def sync_moveit_with_gazebo(
    node,
):

    state = load_last_state() or {}

    displayed = state.get(
        "displayed_obstacle",
        state.get("obstacle"),
    )

    if displayed:

        node.add_moveit_obstacle(
            *displayed,
            padding=0.0,
        )

    else:

        node.remove_moveit_obstacle()


# ============================================================
# 開始姿勢へ戻る
#
# Gazebo の障害物を表示したまま戻るため、前のエピソードの
# MLP 軌道を逆にたどる（その障害物を避けた軌道なので、
# 逆向きでも当たらない）。動かす前に MoveIt で確認する。
# 戻る経路が分からないときだけ、障害物を一時的に片付けて
# まっすぐ戻る。
# ============================================================

def return_to_start(
    node,
    episodes,
    last_state,
    args,
):

    current = node.current_arm_joints()

    if joints_close(
        current,
        START_JOINTS,
    ):

        print(
            "すでに開始姿勢（START）にいます。"
        )

        return True

    # 前回と同じデータ（PAI_DATA_DIR）のときだけ、その軌道を逆にたどれる
    previous = (
        last_state.get("episode")
        if last_state
        and last_state.get("data_dir", str(BASE_DIR)) == str(BASE_DIR)
        else None
    )

    if previous in episodes:

        previous_points = [
            predicted_joints(row)
            for row in episodes[previous]
        ]

        if joints_close(
            current,
            previous_points[-1],
        ):

            path = list(
                reversed(
                    previous_points
                )
            ) + [
                START_JOINTS
            ]

            node.add_moveit_obstacle(
                *last_state["obstacle"]
            )

            (
                safe,
                checks,
                invalid,
            ) = node.validate_points(
                [current] + path
            )

            sync_moveit_with_gazebo(node)

            print(
                f"戻る経路（episode {previous} の逆順）の安全性検証: "
                f"{'SAFE' if safe else 'UNSAFE'} "
                f"({invalid}/{checks} 無効)"
            )

            if safe:

                print(
                    "障害物を表示したまま、"
                    f"episode {previous} の軌道を逆にたどって"
                    "開始姿勢（START）へ戻ります..."
                )

                return node.send_trajectory(
                    path,
                    args.return_seconds,
                    "return-to-start",
                )

    print(
        "戻る経路が分からないため、"
        "Gazebo の障害物を一時的に片付けて"
        "開始姿勢（START）へ戻ります..."
    )

    remove_gazebo_obstacle()

    update_last_state(
        displayed_obstacle=None,
    )

    sync_moveit_with_gazebo(node)

    return node.move_to_start(
        args.reset_seconds
    )


# ============================================================
# メイン処理
# ============================================================

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--execute",
        action="store_true",
        help=(
            "Gazebo のロボットを実際に "
            "動かす。このフラグがない場合は "
            "事前確認（preflight）のみ実行する。"
        ),
    )

    parser.add_argument(
        "--reset-seconds",
        type=float,
        default=DEFAULT_RESET_SECONDS,
    )

    parser.add_argument(
        "--trajectory-seconds",
        type=float,
        default=DEFAULT_TRAJECTORY_SECONDS,
    )

    parser.add_argument(
        "--clearance",
        type=float,
        default=0.0,
        help=(
            "障害物との余裕（m）。MoveIt の障害物を各面この量だけ広げて"
            "衝突判定する（例：0.010）。"
        ),
    )

    parser.add_argument(
        "--carried-object",
        action="store_true",
        help=(
            "手先に仮の把持物（3 cm 角）を付けた状態で衝突判定する。"
        ),
    )

    parser.add_argument(
        "--return-seconds",
        type=float,
        default=DEFAULT_RETURN_SECONDS,
        help=(
            "前のエピソードの軌道を逆にたどって"
            "開始姿勢へ戻る時間（秒）。"
        ),
    )

    parser.add_argument(
        "--episode",
        type=int,
        default=None,
        help=(
            "指定した1つの episode ID のみ実行する。"
        ),
    )

    args = parser.parse_args()

    print(
        "================================"
    )

    print(
        "SO-101 MLP Gazebo デモ"
    )

    print(
        "================================"
    )

    print(
        "オンライン OMPL 再計画: OFF"
    )

    print(
        "軌道の生成元: "
        "MLP の予測"
    )

    print()

    episodes = load_predictions(
        PREDICTIONS_FILE
    )

    candidates = load_demo_candidates(
        DEMO_CANDIDATES_FILE
    )

    ordered_candidates = [
        (
            "left",
            candidates["left"],
        ),
        (
            "center",
            candidates["center"],
        ),
        (
            "right",
            candidates["right"],
        ),
    ]

    if args.episode is not None:

        filtered = []

        for class_name, episode_id in (
            ordered_candidates
        ):

            if episode_id == (
                args.episode
            ):

                filtered.append(
                    (
                        class_name,
                        episode_id,
                    )
                )

        # 候補にないIDも指定可能
        if not filtered:

            if args.episode not in episodes:

                raise RuntimeError(
                    f"episode "
                    f"{args.episode} "
                    f"が見つかりません。"
                )

            class_name = (
                episodes[
                    args.episode
                ][0][
                    "obstacle_class"
                ]
            )

            filtered = [
                (
                    class_name,
                    args.episode,
                )
            ]

        ordered_candidates = (
            filtered
        )

    print(
        "デモ対象の episode:"
    )

    for class_name, episode_id in (
        ordered_candidates
    ):

        print(
            f"  {class_name:6s}: "
            f"episode "
            f"{episode_id}"
        )

    print()

    if not args.execute:

        print(
            "事前確認モード（PRE-FLIGHT）"
        )

        print(
            "ロボットは動きません。"
        )

    else:

        print(
            "実行モード"
        )

        print(
            "Gazebo のロボットが動きます。"
        )

    print()

    rclpy.init()

    node = MLPDemoNode()

    node.clearance = args.clearance

    node.carried_object = args.carried_object

    print(
        f"衝突判定の条件: 障害物との余裕 {args.clearance * 1000:.1f} mm、"
        f"仮の把持物 {'あり' if args.carried_object else 'なし'}"
    )

    # 前回の実行で最後に動かしたエピソード（戻る経路を決めるのに使う）
    last_state = load_last_state()

    results = []

    total_session_start = None
    total_session_end = None

    try:

        for index, (
            expected_class,
            episode_id,
        ) in enumerate(
            ordered_candidates,
            start=1,
        ):

            if episode_id not in episodes:

                print(
                    f"episode "
                    f"{episode_id} "
                    f"が見つかりません。スキップします。"
                )

                continue

            rows = episodes[
                episode_id
            ]

            first = rows[0]

            actual_class = (
                first[
                    "obstacle_class"
                ]
            )

            obs_x = float(
                first["obs_x"]
            )

            obs_y = float(
                first["obs_y"]
            )

            obs_z = float(
                first["obs_z"]
            )

            print()
            print(
                "--------------------------------"
            )

            print(
                f"デモ {index}/"
                f"{len(ordered_candidates)}"
            )

            print(
                f"episode: {episode_id}"
            )

            print(
                f"クラス: {actual_class}"
            )

            print(
                f"障害物: "
                f"x={obs_x:.3f}, "
                f"y={obs_y:.3f}, "
                f"z={obs_z:.3f}"
            )

            # -----------------------------------------------
            # 前エピソードの障害物
            #
            # Gazebo の障害物は削除せず、画面に表示し続ける。
            # MoveIt（RViz）の障害物も Gazebo と同じものを表示し続け、
            # 衝突判定のときだけ一時的に広げる。
            # 事前確認（preflight）では Gazebo に触れない。
            # -----------------------------------------------

            # -----------------------------------------------
            # 実行時は、障害物を表示したまま開始姿勢へ戻り、
            # 止まっている間に障害物を入れ替える
            # -----------------------------------------------

            episode_wall_start = None

            if args.execute:

                if (
                    total_session_start
                    is None
                ):

                    total_session_start = (
                        time.monotonic()
                    )

                episode_wall_start = (
                    time.monotonic()
                )

                reset_ok = return_to_start(
                    node,
                    episodes,
                    last_state,
                    args,
                )

                if not reset_ok:

                    print(
                        "開始姿勢へ戻れませんでした。"
                        "スキップします。"
                    )

                    continue

                print(
                    "Gazebo の障害物を "
                    "このエピソードの位置に置きます..."
                )

                if not (
                    spawn_gazebo_obstacle(
                        obs_x,
                        obs_y,
                        obs_z,
                    )
                ):

                    print(
                        "Gazebo 障害物の "
                        "生成に失敗しました。"
                    )

                    continue

                update_last_state(
                    displayed_obstacle=[obs_x, obs_y, obs_z],
                )

            # -----------------------------------------------
            # MoveIt障害物（衝突判定のため、--clearance の分だけ広げて置く）
            # -----------------------------------------------

            if not (
                node.add_moveit_obstacle(
                    obs_x,
                    obs_y,
                    obs_z,
                )
            ):

                print(
                    "MoveIt 障害物の "
                    "生成に失敗しました。"
                )

                continue

            # -----------------------------------------------
            # AI推論軌道安全性再確認
            # -----------------------------------------------

            (
                safe,
                checks,
                invalid,
            ) = node.validate_prediction(
                rows
            )

            print(
                f"安全性検証: "
                f"{'SAFE' if safe else 'UNSAFE'} "
                f"("
                f"{invalid}/"
                f"{checks} 無効"
                f")"
            )

            if not safe:

                print(
                    "安全でない軌道です。"
                    "実行を中止しました。"
                )

                sync_moveit_with_gazebo(node)

                continue

            # -----------------------------------------------
            # 事前確認（preflight）だけならここで終了
            # -----------------------------------------------

            if not args.execute:

                results.append(
                    {
                        "episode":
                            episode_id,

                        "class":
                            actual_class,

                        "safe":
                            True,

                        "executed":
                            False,

                        "trajectory_seconds":
                            0.0,

                        "episode_wall":
                            0.0,
                    }
                )

                sync_moveit_with_gazebo(node)

                continue

            # -----------------------------------------------
            # 判定が済んだので、MoveIt（RViz）の障害物を実物の大きさに戻す
            # -----------------------------------------------

            sync_moveit_with_gazebo(node)

            # -----------------------------------------------
            # AI軌道実行
            # -----------------------------------------------

            print(
                "MLP 軌道を "
                "実行中..."
            )

            trajectory_start = (
                time.monotonic()
            )

            execute_ok = (
                node.execute_prediction(
                    rows,
                    args.trajectory_seconds,
                )
            )

            # 次のエピソード（次回の実行を含む）で、この軌道を逆にたどって戻れるように記録する
            save_last_state(
                episode_id,
                obs_x,
                obs_y,
                obs_z,
            )

            last_state = load_last_state()

            trajectory_end = (
                time.monotonic()
            )

            episode_wall_end = (
                time.monotonic()
            )

            trajectory_wall = (
                trajectory_end
                - trajectory_start
            )

            episode_wall = (
                episode_wall_end
                - episode_wall_start
            )

            print(
                f"軌道の実時間: "
                f"{trajectory_wall:.2f} s"
            )

            print(
                f"episode の合計時間: "
                f"{episode_wall:.2f} s"
            )

            print(
                f"60秒以内の要件: "
                f"{'PASS' if episode_wall < 60.0 else 'FAIL'}"
            )

            results.append(
                {
                    "episode":
                        episode_id,

                    "class":
                        actual_class,

                    "safe":
                        safe,

                    "executed":
                        execute_ok,

                    "trajectory_seconds":
                        trajectory_wall,

                    "episode_wall":
                        episode_wall,
                }
            )

            # -----------------------------------------------
            # 障害物は Gazebo・MoveIt（RViz）とも表示したままにする
            # -----------------------------------------------

            sync_moveit_with_gazebo(node)

            if index < len(
                ordered_candidates
            ):

                time.sleep(
                    PAUSE_BETWEEN_EPISODES
                )

        if args.execute:

            total_session_end = (
                time.monotonic()
            )

    finally:

        # Gazebo の障害物は実行後も残し、MoveIt（RViz）の表示もそれに合わせる
        sync_moveit_with_gazebo(node)

        node.destroy_node()

        rclpy.shutdown()

    # ========================================================
    # 集計
    # ========================================================

    print()
    print(
        "================================"
    )

    print(
        "===== デモの集計 ====="
    )

    print(
        "================================"
    )

    for result in results:

        print(
            f"{result['class']:6s} "
            f"episode "
            f"{result['episode']:3d}: "
            f"安全="
            f"{result['safe']} "
            f"実行済み="
            f"{result['executed']} "
            f"episode 時間="
            f"{result['episode_wall']:.2f}s"
        )

    if (
        args.execute
        and total_session_start
        is not None
        and total_session_end
        is not None
    ):

        total_wall = (
            total_session_end
            - total_session_start
        )

        print()

        print(
            f"デモの合計時間: "
            f"{total_wall:.2f} s"
        )

        print(
            f"3分以内の要件: "
            f"{'PASS' if total_wall < 180.0 else 'FAIL'}"
        )


if __name__ == "__main__":

    main()
