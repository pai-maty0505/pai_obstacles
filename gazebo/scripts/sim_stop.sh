#!/usr/bin/env bash
# Gazebo + MoveIt 2 + RViz を停止する。
# sim_start.sh で起動したものに加え、手動で ros2 launch したものも停止する。
#
# 使い方：scripts/sim_stop.sh

set -o pipefail

source "$(dirname "${BASH_SOURCE[0]}")/sim_env.sh"

wait_until_gone() {
    # wait_until_gone <秒>：launch と残留プロセスがすべて終わるまで待つ
    local limit=$1 waited=0
    while [[ -n "$(launch_pids)$(stray_pids)" ]] && (( waited < limit )); do
        sleep 1
        waited=$(( waited + 1 ))
    done
    [[ -z "$(launch_pids)$(stray_pids)" ]]
}

if [[ -z "$(launch_pids)$(stray_pids)" ]]; then
    echo "停止するシミュレーションはありません。"
    rm -f "${PID_FILE}"
    exit 0
fi

# 1. ros2 launch に SIGINT を送り、通常の終了処理をさせる
if [[ -f "${PID_FILE}" ]] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
    echo "sim_start.sh で起動したシミュレーションを停止します。"
    kill -INT -- "-$(cat "${PID_FILE}")" 2>/dev/null || true
else
    echo "ros2 launch を停止します。"
    for pid in $(launch_pids); do
        kill -INT "${pid}" 2>/dev/null || true
    done
fi

if wait_until_gone 20; then
    echo "停止しました。"
    rm -f "${PID_FILE}"
    exit 0
fi

# 2. 残ったプロセスに SIGTERM、それでも残れば SIGKILL
echo "残っているプロセスを終了します。"
for pid in $(launch_pids) $(stray_pids); do
    kill -TERM "${pid}" 2>/dev/null || true
done
if ! wait_until_gone 10; then
    for pid in $(launch_pids) $(stray_pids); do
        kill -KILL "${pid}" 2>/dev/null || true
    done
    wait_until_gone 5 || {
        echo "停止できなかったプロセスがあります：" >&2
        for pid in $(launch_pids) $(stray_pids); do ps -o pid=,args= -p "${pid}" >&2; done
        exit 1
    }
fi

echo "停止しました。"
rm -f "${PID_FILE}"
