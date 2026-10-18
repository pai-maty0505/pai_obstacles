#!/usr/bin/env bash
# Gazebo でデモを実行し、必ず画面を撮影しながら、接触と実時間係数も記録する。
# 撮影なしでロボットを動かさないため、Gazebo での実行はこのスクリプトから行う。
#
# 使い方：scripts/run_episode_recorded.sh [--interval <秒>] [--view work|close] [--no-video] [--script <ファイル>] -- <スクリプトの引数>
#   例：scripts/run_episode_recorded.sh -- --execute --episode 4
#   --interval  撮影の間隔（既定 1 秒）
#   --view      Gazebo のカメラ：work（作業領域全体、既定）／close（少し寄る）
#   --script    実行するスクリプト（既定：scripts/run_mlp_gazebo_demo.py）。把持の試験などに使う
#   --no-video  動画を撮らない（既定では、1 回の実行ごとに必ず動画を撮る）。
#               動画は、Gazebo にカメラセンサを置き、シミュレーション時間で 30 fps の画像を保存して
#               字幕付きの MP4（等倍速）を作る（video.mp4）。撮影中は実時間係数が 0.7 前後に下がるが、
#               動画はシミュレーション時間どおりに進むので速さは変わらない
#
# 結果は logs/runs/<日時>_<引数>/ に保存する：
#   shots/（撮影画像）、run.log、contacts.txt、stats.txt（壁時計・シミュレーション時間・RTF）、
#   summary.txt、sheet_all.png（全画像の一覧）、sheet_contacts.png（接触した時刻の前後）
#
# 終了コード：デモが成功し、かつ不許可の接触・めり込みがなければ 0。どちらかがだめなら 1

set -o pipefail

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
source "${SCRIPT_DIR}/sim_env.sh"

INTERVAL=1
VIEW=work
VIDEO=true
VIDEO_FPS=30
SCRIPT=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --interval) INTERVAL="$2"; shift 2 ;;
        --view) VIEW="$2"; shift 2 ;;
        --video) VIDEO=true; shift ;;
        --no-video) VIDEO=false; shift ;;
        --script) SCRIPT="$2"; shift 2 ;;
        --) shift; break ;;
        -h|--help) sed -n '2,21p' "$0"; exit 0 ;;
        *) echo "不明な引数です：$1（デモの引数は -- の後に書いてください）" >&2; exit 2 ;;
    esac
done
DEMO_ARGS=("$@")

source_ros

if ! "${SCRIPT_DIR}/sim_check.sh" --quiet; then
    echo "シミュレーションの準備が整っていません。scripts/sim_start.sh で起動してください。" >&2
    "${SCRIPT_DIR}/sim_check.sh" >&2
    exit 1
fi

TAG="$(printf '%s' "${DEMO_ARGS[*]}" | tr -c 'A-Za-z0-9' '_' | sed 's/__*/_/g; s/^_//; s/_$//')"
RUN_DIR="${LOG_DIR}/runs/$(date +%Y%m%d_%H%M%S)_${TAG}"
mkdir -p "${RUN_DIR}/shots"
echo "記録先：${RUN_DIR}"

# Gazebo のカメラ（向きは同じで、距離だけ変える）
case "${VIEW}" in
    close) CAMERA='pose: {position: {x: 0.476, y: -0.386, z: 0.357}, orientation: {w: 0.3497, x: -0.2279, y: 0.0881, z: 0.9044}}' ;;
    *)     CAMERA='pose: {position: {x: 0.606, y: -0.505, z: 0.452}, orientation: {w: 0.3497, x: -0.2279, y: 0.0881, z: 0.9044}}' ;;
esac
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean \
    --timeout 3000 --req "${CAMERA}" >/dev/null 2>&1 || echo "カメラ位置を設定できませんでした。" >&2
sleep 1

BACKGROUND_PIDS=()
stop_background() {
    local pid
    for pid in "${BACKGROUND_PIDS[@]}"; do
        kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    done
    sleep 2
}
trap stop_background EXIT

# 1. 撮影
setsid "${SCRIPT_DIR}/gz_screenshots.sh" "${RUN_DIR}/shots" "${INTERVAL}" >/dev/null 2>&1 &
BACKGROUND_PIDS+=($!)

# 2. 障害物と運搬物の接触センサ
setsid gz topic -e -t /pai/obstacle_contacts >"${RUN_DIR}/contacts.txt" 2>/dev/null &
BACKGROUND_PIDS+=($!)
setsid gz topic -e -t /pai/object_contacts >"${RUN_DIR}/object_contacts.txt" 2>/dev/null &
BACKGROUND_PIDS+=($!)
# 床の接触センサ（sim/worlds/pai_avoid.world）
setsid gz topic -e -t /pai/ground_contacts >"${RUN_DIR}/ground_contacts.txt" 2>/dev/null &
BACKGROUND_PIDS+=($!)
# 把持台・お皿（scripts/mission_scene.py）の接触センサ
setsid gz topic -e -t /pai/scene_contacts >"${RUN_DIR}/scene_contacts.txt" 2>/dev/null &
BACKGROUND_PIDS+=($!)

# 3. 壁時計・シミュレーション時間・実時間係数（0.5 秒ごと）
setsid bash -c '
    while true; do
        msg="$(gz topic -e -t /world/default/stats -n 1 2>/dev/null)"
        sec=$(sed -n "/^sim_time {/,/}/p" <<<"$msg" | awk "/sec:/ && !/nsec/ {print \$2}")
        nsec=$(sed -n "/^sim_time {/,/}/p" <<<"$msg" | awk "/nsec:/ {print \$2}")
        rtf=$(awk "/^real_time_factor:/ {print \$2}" <<<"$msg")
        echo "$(date +%s.%N) ${sec:-0}.$(printf %09d ${nsec:-0}) ${rtf:-0}"
        sleep 0.5
    done' >"${RUN_DIR}/stats.txt" 2>/dev/null &
BACKGROUND_PIDS+=($!)

# 4. 動画（カメラセンサ）
VIDEO_CAMERA_IMAGE_TOPIC="/world/default/model/video_camera/link/link/sensor/camera/image"
remove_video_camera() {
    gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
        --timeout 3000 --req 'name: "video_camera", type: MODEL' >/dev/null 2>&1 || true
}
if ${VIDEO}; then
    mkdir -p "${RUN_DIR}/frames"
    sed -e "s#FPS#${VIDEO_FPS}#; s#HFOV#1.2#; s#WIDTH#1280#; s#HEIGHT#720#; s#SAVE_DIR#${RUN_DIR}/frames#" \
        "${REPO_DIR}/sim/models/video_camera.sdf.template" >"${RUN_DIR}/video_camera.sdf"
    remove_video_camera
    # カメラの位置と向きは、Gazebo の画面の work 視点と同じ
    # ros2 run ros_gz_sim create は、置いた後も戻らないことがあるので、Gazebo の create サービスを直接呼ぶ
    # （向きは R 0、P 0.494、Y 2.404 を四元数にしたもの。置けたかは、次の画像の待ち受けで確かめる）
    gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
        --req "sdf_filename: \"${RUN_DIR}/video_camera.sdf\", name: \"video_camera\", pose: {position: {x: 0.606, y: -0.505, z: 0.452}, orientation: {w: 0.3497, x: -0.2279, y: 0.0881, z: 0.9044}}" >/dev/null 2>&1
    # 購読者がいないとカメラセンサは描画しないので、画像を受け取るだけのプロセスを置く
    setsid gz topic -e -t "${VIDEO_CAMERA_IMAGE_TOPIC}" >/dev/null 2>&1 &
    BACKGROUND_PIDS+=($!)
    echo -n "動画の撮影開始を待っています"
    for _ in $(seq 1 60); do
        if compgen -G "${RUN_DIR}/frames/*.png" >/dev/null; then
            break
        fi
        echo -n "."
        sleep 0.5
    done
    echo
    # 最初の画像のシミュレーション時間（字幕の時刻合わせに使う）
    gz topic -e -t /world/default/stats -n 1 | sed -n '/^sim_time {/,/}/p' \
        | awk '/ sec:/{s=$2} /nsec:/{n=$2} END{printf "%.3f\n", s+n/1e9}' >"${RUN_DIR}/video_start_sim.txt"
fi

sleep 2

# 5. デモの実行
/usr/bin/python3 -u "${SCRIPT:-${SCRIPT_DIR}/run_mlp_gazebo_demo.py}" "${DEMO_ARGS[@]}" 2>&1 | tee "${RUN_DIR}/run.log"
DEMO_STATUS=${PIPESTATUS[0]}

sleep 2
stop_background
trap - EXIT
if ${VIDEO}; then
    remove_video_camera
fi

# 6. 集計と判定
/usr/bin/python3 "${SCRIPT_DIR}/summarize_run.py" "${RUN_DIR}" | tee "${RUN_DIR}/summary.txt"
/usr/bin/python3 "${SCRIPT_DIR}/analyze_contacts.py" "${RUN_DIR}/contacts.txt" >/dev/null
CONTACT_STATUS=$?
# issue #7 の仕様（床に置いた物体）：ログの「床との接触の監視：開始／終了」の間は、運搬物と床の接触を不許可にし
# （持ち上げた後に床に触れたら落下）、それ以外（つかむ前に床に置かれている間）は許可する
GROUND_RULES=()
if grep -q "床との接触の監視：開始" "${RUN_DIR}/run.log"; then
    mapfile -t GROUND_RULES < <(/usr/bin/python3 - "${RUN_DIR}" <<'PYEOF'
import re, sys
from pathlib import Path
run = Path(sys.argv[1])
stats = []
for line in (run / "stats.txt").read_text().splitlines():
    parts = line.split()
    if len(parts) == 3:
        stats.append((float(parts[0]), float(parts[1])))
def to_sim(wall):
    for (w0, s0), (w1, s1) in zip(stats, stats[1:]):
        if w0 <= wall <= w1:
            return s0 + (s1 - s0) * (wall - w0) / (w1 - w0) if w1 > w0 else s0
    return stats[-1][1] if wall > stats[-1][0] else stats[0][1]
start = None
for line in (run / "run.log").read_text(encoding="utf-8").splitlines():
    m = re.match(r"^\[INFO\] \[(\d+\.\d+)\] \[[^\]]+\]: 床との接触の監視：(開始|終了)", line)
    if not m:
        continue
    if m.group(2) == "開始":
        start = to_sim(float(m.group(1)))
    elif start is not None:
        print(f"--deny-during=ground_plane:carried_object@{start:.3f}-{to_sim(float(m.group(1))):.3f}")
        start = None
if start is not None:
    print(f"--deny-during=ground_plane:carried_object@{start:.3f}-inf")
PYEOF
)
    GROUND_RULES+=("--allow" "ground_plane:carried_object")
    echo "運搬物と床の接触：持ち上げた後（${#GROUND_RULES[@]} 個の指定）は不許可、つかむ前は許可"
fi
# 運搬物：左右のパッドと把持台・配置台・お皿との接触だけを許可する（issue #7 の仕様では、つかむ前の床との接触も許可）
/usr/bin/python3 "${SCRIPT_DIR}/analyze_contacts.py" "${RUN_DIR}/object_contacts.txt" \
    --allow "pad:carried_object" --allow "_stand:carried_object" --allow "_plate:carried_object" \
    "${GROUND_RULES[@]}" | tee "${RUN_DIR}/object_contacts_summary.txt"
OBJECT_CONTACT_STATUS=${PIPESTATUS[0]}
if [[ ${OBJECT_CONTACT_STATUS} -ne 0 ]]; then
    CONTACT_STATUS=1
fi
# 床：ロボットが触れたら不許可。運搬物は、つかむ前（issue #7 の仕様で床に置かれている間）だけ許可
if [[ -s "${RUN_DIR}/ground_contacts.txt" ]]; then
    GROUND_ALLOW=()
    if [[ ${#GROUND_RULES[@]} -gt 0 ]]; then
        GROUND_ALLOW=("${GROUND_RULES[@]}")
    fi
    # 準備の段階で issue #5 の仕様の台・お皿・障害物などが床に載っているのは許可する（ロボット以外）
    /usr/bin/python3 "${SCRIPT_DIR}/analyze_contacts.py" "${RUN_DIR}/ground_contacts.txt" \
        --allow "ground_plane:_stand" --allow "ground_plane:_plate" --allow "ground_plane:obstacle" \
        "${GROUND_ALLOW[@]}" | tee "${RUN_DIR}/ground_contacts_summary.txt"
    if [[ ${PIPESTATUS[0]} -ne 0 ]]; then
        CONTACT_STATUS=1
    fi
fi
# 把持台・お皿：運搬物が載るのだけを許可する（ロボットが触れたら不許可）
if [[ -s "${RUN_DIR}/scene_contacts.txt" ]]; then
    /usr/bin/python3 "${SCRIPT_DIR}/analyze_contacts.py" "${RUN_DIR}/scene_contacts.txt" \
        --allow "_stand:carried_object" --allow "_plate:carried_object" | tee "${RUN_DIR}/scene_contacts_summary.txt"
    if [[ ${PIPESTATUS[0]} -ne 0 ]]; then
        CONTACT_STATUS=1
    fi
fi

# 対象のエピソードがすべて実行されたか（対象の数と「実行済み=True」の数を比べる。
# preflight で UNSAFE になって飛ばされたエピソードも失敗として数える）
TARGETS=$(grep -cE "^  (left|center|right) *: episode [0-9]+$" "${RUN_DIR}/run.log")
EXECUTED=$(grep -c "実行済み=True" "${RUN_DIR}/run.log")
if [[ -n "${SCRIPT}" ]]; then
    # デモ以外のスクリプトは、終了コードだけで判定する
    TARGETS=1
    EXECUTED=$([[ ${DEMO_STATUS} -eq 0 ]] && echo 1 || echo 0)
fi
if [[ ${TARGETS} -gt 0 && ${EXECUTED} -eq ${TARGETS} ]]; then
    EXECUTED_OK=0
else
    EXECUTED_OK=1
fi

{
    echo "===== 判定 ====="
    echo "  デモの実行：$([[ ${DEMO_STATUS} -eq 0 && ${EXECUTED_OK} -eq 0 ]] && echo 成功 || echo 失敗)（実行できたエピソード ${EXECUTED} / ${TARGETS}）"
    echo "  接触の判定：$([[ ${CONTACT_STATUS} -eq 0 ]] && echo '合格（不許可の接触・めり込みなし）' || echo '不合格（不許可の接触あり）')（運搬物は左右のパッドと台・お皿だけ許可、issue #7 の仕様ではつかむ前の床も許可。台・お皿・床にはロボットが触れない）"
    if [[ ${DEMO_STATUS} -eq 0 && ${EXECUTED_OK} -eq 0 && ${CONTACT_STATUS} -eq 0 ]]; then
        echo "  総合：成功"
    else
        echo "  総合：失敗"
    fi
} | tee -a "${RUN_DIR}/summary.txt"
if ${VIDEO}; then
    /usr/bin/python3 "${SCRIPT_DIR}/make_video.py" "${RUN_DIR}" --fps "${VIDEO_FPS}" | tee -a "${RUN_DIR}/summary.txt"
fi
echo "記録先：${RUN_DIR}"

if [[ ${DEMO_STATUS} -eq 0 && ${EXECUTED_OK} -eq 0 && ${CONTACT_STATUS} -eq 0 ]]; then
    exit 0
fi
exit 1
