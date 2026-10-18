# シミュレーション関連スクリプトの共通設定（source して使う）

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${REPO_DIR}/logs"
PID_FILE="${LOG_DIR}/sim.pid"

# プロセスはコマンド行の先頭（実行ファイルのパス）で判別する。
# 単語だけで探すと、grep やエディタなど無関係のプロセスまで一致してしまうため。
ROS_LIB="/opt/ros/jazzy/lib"

# ros2 launch
LAUNCH_PATTERN="^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch so101_bringup so101_gazebo_moveit\.launch\.py"

# 停止時に取り残されやすいプロセス
STRAY_PATTERNS=(
    "^gz sim "
    "^ruby .*/gz sim "
    "^/bin/sh -c ruby .*/gz sim "
    "^${ROS_LIB}/moveit_ros_move_group/move_group"
    "^${ROS_LIB}/rviz2/rviz2"
    "^${ROS_LIB}/robot_state_publisher/robot_state_publisher"
    "^${ROS_LIB}/ros_gz_bridge/"
    "^/usr/bin/python3 ${ROS_LIB}/controller_manager/spawner"
)

source_ros() {
    # setup.bash は未定義変数を参照するため、nounset を一時的に外す
    set +u
    source /opt/ros/jazzy/setup.bash
    source "${HOME}/so101_ws/install/setup.bash"
}

launch_pids() {
    pgrep -f "${LAUNCH_PATTERN}" || true
}

stray_pids() {
    local pattern
    for pattern in "${STRAY_PATTERNS[@]}"; do
        pgrep -f "${pattern}" || true
    done
}
