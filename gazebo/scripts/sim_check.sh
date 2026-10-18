#!/usr/bin/env bash
# Gazebo・ros2_control・MoveIt 2・RViz の準備が整っているかを確認する。
# すべて整っていれば終了コード 0、1つでも欠けていれば 1 を返す。
#
# 使い方：scripts/sim_check.sh [--quiet]

set -o pipefail

source "$(dirname "${BASH_SOURCE[0]}")/sim_env.sh"
source_ros

QUIET=false
if [[ "${1:-}" == "--quiet" ]]; then
    QUIET=true
fi

ok=true

report() {
    # report <成否 0/1> <項目名>
    if [[ "$1" == 0 ]]; then
        $QUIET || echo "  [OK]   $2"
    else
        $QUIET || echo "  [未完] $2"
        ok=false
    fi
}

$QUIET || echo "シミュレーションの状態："

# Gazebo：ワールド default の生成・削除サービス（障害物の配置に使う）
gz_services="$(timeout 5 gz service -l 2>/dev/null)"
grep -qx "/world/default/create" <<<"${gz_services}" \
    && grep -qx "/world/default/remove" <<<"${gz_services}"
report $? "Gazebo のワールド default（/world/default/create, remove）"

# 関節状態
timeout 5 ros2 topic echo --once /joint_states --field name >/dev/null 2>&1
report $? "/joint_states の受信"

# コントローラ
controllers="$(timeout 10 ros2 control list_controllers 2>/dev/null)"
for name in arm_controller gripper_controller joint_state_broadcaster; do
    grep -qE "^${name}[[:space:]].*[[:space:]]active" <<<"${controllers}"
    report $? "${name} が active"
done

# MoveIt 2 のサービス
services="$(timeout 10 ros2 service list 2>/dev/null)"
for name in /check_state_validity /apply_planning_scene /compute_ik; do
    grep -qx "${name}" <<<"${services}"
    report $? "MoveIt サービス ${name}"
done

# RViz
nodes="$(timeout 10 ros2 node list 2>/dev/null)"
grep -qx "/rviz" <<<"${nodes}"
report $? "RViz（/rviz ノード）"

if $ok; then
    $QUIET || echo "準備完了です。"
    exit 0
fi

$QUIET || echo "まだ準備が整っていません。"
exit 1
