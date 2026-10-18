#!/usr/bin/env bash
# Gazebo + MoveIt 2 + RViz をバックグラウンドで起動し、準備が整うまで待つ。
# ワールドと RViz の設定は本リポジトリの sim/ にあるものを使い、
# 起動後に Gazebo のカメラを作業領域に向ける。
#
# 使い方：scripts/sim_start.sh [--world <名前またはパス>] [--rviz <名前またはパス>] [--timeout <秒>] [--camera]
#   --world    sim/worlds のワールド名、またはファイルのパス（既定：pai_avoid.world）
#   --rviz     sim/rviz の設定名、またはファイルのパス（既定：pai_demo.rviz）
#   --timeout  準備完了を待つ最大秒数（既定：180）
#   --camera   ロボットのカメラセンサを有効にする（既定：無効。実時間係数を下げないため）
#
# ログは logs/sim_<日時>.log、停止は scripts/sim_stop.sh

set -eo pipefail

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
source "${SCRIPT_DIR}/sim_env.sh"

WORLD="pai_avoid.world"
RVIZ="pai_demo.rviz"
TIMEOUT=180
USE_CAMERA=false

# Gazebo の GUI カメラ：作業領域 (0.15, -0.09, 0.12) を右斜め前から見る（開始姿勢〜右へ約 80° の目標まで入る）
GZ_CAMERA_POSE='pose: {position: {x: 0.606, y: -0.505, z: 0.452}, orientation: {w: 0.3497, x: -0.2279, y: 0.0881, z: 0.9044}}'

while [[ $# -gt 0 ]]; do
    case "$1" in
        --world) WORLD="$2"; shift 2 ;;
        --rviz) RVIZ="$2"; shift 2 ;;
        --timeout) TIMEOUT="$2"; shift 2 ;;
        --camera) USE_CAMERA=true; shift ;;
        -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
        *) echo "不明な引数です：$1" >&2; exit 2 ;;
    esac
done

resolve_file() {
    # resolve_file <名前またはパス> <既定のディレクトリ>：絶対パスを返す
    local name=$1 dir=$2
    if [[ -f "${name}" ]]; then
        realpath "${name}"
    elif [[ -f "${dir}/${name}" ]]; then
        realpath "${dir}/${name}"
    else
        echo "ファイルが見つかりません：${name}（${dir} も探しました）" >&2
        return 1
    fi
}

if [[ -n "$(launch_pids)" ]]; then
    echo "シミュレーションは既に起動しています。状態を確認します。"
    echo "（設定を変えて起動し直す場合は scripts/sim_stop.sh を実行してから起動してください）"
    exec "${SCRIPT_DIR}/sim_check.sh"
fi

if [[ -n "$(stray_pids)" ]]; then
    echo "前回のシミュレーションのプロセスが残っています。scripts/sim_stop.sh で停止してから起動してください。" >&2
    exit 1
fi

WORLD_PATH="$(resolve_file "${WORLD}" "${REPO_DIR}/sim/worlds")"
RVIZ_PATH="$(resolve_file "${RVIZ}" "${REPO_DIR}/sim/rviz")"

source_ros

# so101_moveit_config の launch は「<パッケージ>/rviz/<ファイル名>」の形でしか RViz 設定を受け取らないため、
# そこからの相対パスで本リポジトリの設定を指定する
MOVEIT_RVIZ_DIR="$(realpath "$(ros2 pkg prefix so101_moveit_config)/share/so101_moveit_config/rviz")"
RVIZ_ARG="$(realpath -m --relative-to="${MOVEIT_RVIZ_DIR}" "${RVIZ_PATH}")"

mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/sim_$(date +%Y%m%d_%H%M%S).log"

# 新しいワールドには障害物がなく、ロボットは開始姿勢にいるので、デモの前回の状態（表示中の障害物など）を消す
rm -f "${LOG_DIR}/demo_last_state.json"

# Gazebo に生成するロボットの SDF を作る（メッシュの衝突形状を凸分解にする）
ROBOT_SDF="${LOG_DIR}/so101_gazebo.sdf"
CAMERA_OPTION=()
if [[ "${USE_CAMERA}" == true ]]; then
    CAMERA_OPTION=(--use-camera)
fi
/usr/bin/python3 "${SCRIPT_DIR}/make_robot_sdf.py" "${ROBOT_SDF}" "${CAMERA_OPTION[@]}"

echo "起動します"
echo "  ワールド：${WORLD_PATH}"
echo "  RViz 設定：${RVIZ_PATH}"
echo "  ロボットの SDF：${ROBOT_SDF}"
echo "  カメラセンサ：${USE_CAMERA}"
echo "  ログ：${LOG_FILE}"

# 新しいセッションで起動し、停止時にプロセスグループごと終了できるようにする
setsid nohup ros2 launch so101_bringup so101_gazebo_moveit.launch.py \
    world_file:="${WORLD_PATH}" \
    use_camera:="${USE_CAMERA}" \
    rviz_config_file:="${RVIZ_ARG}" \
    robot_sdf_file:="${ROBOT_SDF}" \
    >"${LOG_FILE}" 2>&1 </dev/null &
LAUNCH_PID=$!
echo "${LAUNCH_PID}" >"${PID_FILE}"

start_time=${SECONDS}
echo -n "準備完了を待っています"
until "${SCRIPT_DIR}/sim_check.sh" --quiet; do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo
        echo "起動に失敗しました（ros2 launch が終了しました）。ログの末尾：" >&2
        tail -n 30 "${LOG_FILE}" >&2
        rm -f "${PID_FILE}"
        exit 1
    fi
    if (( SECONDS - start_time > TIMEOUT )); then
        echo
        echo "${TIMEOUT} 秒以内に準備が整いませんでした。" >&2
        "${SCRIPT_DIR}/sim_check.sh" >&2 || true
        echo "ログ：${LOG_FILE}　停止：scripts/sim_stop.sh" >&2
        exit 1
    fi
    # 凸分解の計算中は物理計算が止まり、コントローラの有効化が時間切れになることがある。
    # Gazebo のロボットが生成済みで、コントローラが inactive のまま残っていれば有効にし直す
    for controller in joint_state_broadcaster arm_controller gripper_controller; do
        if timeout 10 ros2 control list_controllers 2>/dev/null \
                | grep -qE "^${controller}[[:space:]].*[[:space:]]inactive"; then
            timeout 20 ros2 control set_controller_state "${controller}" active >/dev/null 2>&1 || true
        fi
    done
    echo -n "."
    sleep 3
done
echo

# Gazebo のカメラを作業領域に向ける（失敗しても起動自体は成功とする）
if timeout 5 gz service -s /gui/move_to/pose \
        --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean \
        --timeout 3000 --req "${GZ_CAMERA_POSE}" >/dev/null 2>&1; then
    echo "Gazebo のカメラを作業領域に向けました。"
else
    echo "Gazebo のカメラ位置を設定できませんでした（表示には影響しません）。" >&2
fi

"${SCRIPT_DIR}/sim_check.sh"
echo "起動にかかった時間：$(( SECONDS - start_time )) 秒"
echo "停止するときは scripts/sim_stop.sh を実行してください。"
