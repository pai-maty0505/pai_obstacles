#!/usr/bin/env python3
"""
v3 の姿勢を計算して data/poses.json に保存する（move_group が 1 つ起動している必要がある）。

姿勢は shoulder_pan = 0 の腕の面の中で解き、各位置では先頭の shoulder_pan だけを置き換えて使う
（運搬物・障害物・お皿がすべて半径 ARC の円弧の上にあるため）。手首（wrist_roll）は WRIST_ROLL のまま。

解き方（v2 の compute_floor_poses.py と同じ）：指の向き（水平からの角度 pitch）を wrist_flex で合わせ、
運搬物の中心が目標の (半径, 高さ) に来るように shoulder_lift・elbow_flex を FK のニュートン法で解く。

  pick / hover / place / rise   上から（pitch = -90°）つかむ・置く姿勢と、その上下の姿勢
  descend_path など             上下に動く区間の 1 cm ごとの経由点
  via_over                       円柱を上から越える経由点の候補（指を傾け、運搬物を高く上げる）
  via_front                      直方体・三角錐の手前（土台の側）を通る経由点の候補
  grasp_angle                    2 cm の運搬物を挟む指の角度（パッドが触れる角度から少し閉じる）

    python3 scripts/compute_poses.py
"""

import json
import math
import sys
from pathlib import Path

import numpy as np
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import moveit_iface as mi  # noqa: E402
import spec  # noqa: E402

OUTPUT = Path(__file__).resolve().parent.parent / "data" / "poses.json"

R_X = spec.PAN_AXIS_X + spec.ARC          # shoulder_pan = 0 のときの運搬物の x
CENTER = spec.PAYLOAD_SIZE[2] / 2.0       # 床に立てた運搬物の中心の高さ
HEIGHTS = {                                # 運搬物の中心の高さ [m]
    "pick": CENTER,
    "hover": 0.069,                        # 指先が運搬物の上端より約 1.5 cm 上（v2 と同じ）
    "place": spec.PLATE_FLOOR + spec.RELEASE + CENTER,
    "rise": 0.079,                         # 離した後、指先が運搬物の上端より上に抜けた高さ（v2 と同じ）
}
OPEN_ANGLE = 0.30                          # 構える・離すときの指の角度（v2 と同じ）
# 円柱を越える経由点：運搬物の下端が 8 cm + 余裕 1.5 cm より上になる高さから
VIA_OVER = [(pitch, h) for pitch in (-90, -80, -70, -60, -50) for h in (0.12, 0.13, 0.14, 0.15, 0.16)]
# 手前を通る経由点：回転軸からの距離 [m]、運搬物の中心の高さ [m]（指は真下）
VIA_FRONT = [(r, h) for r in (0.13, 0.12, 0.11, 0.10) for h in (0.069, 0.08, 0.09, 0.10)]
# 引き寄せる経由点：円柱を越えた後（または前）に、高さを保ったまま土台の側へ引く。回転軸からの距離、高さ、指の向き
VIA_RETRACT = [(r, h, pitch) for r in (0.14, 0.13, 0.12) for h in (0.12, 0.11, 0.10) for pitch in (-60, -70, -80, -90)]
SEED = (-0.0124, 0.4544)                   # v2 の pick（半径 0.1812 m）の shoulder_lift・elbow_flex


class Solver:
    def __init__(self, client):
        self.client = client

    def state(self, q):
        center, r = self.client.payload_center(q)
        finger = -r[:, 2]
        return center, math.atan2(finger[2], finger[0])

    def level(self, lift, elbow, target, flex=1.2):
        for _ in range(12):
            _, p = self.state([0.0, lift, elbow, flex, spec.WRIST_ROLL])
            _, p2 = self.state([0.0, lift, elbow, flex + 0.01, spec.WRIST_ROLL])
            d = (p2 - p) / 0.01
            if abs(d) < 1e-9:
                break
            step = (p - target) / d
            flex -= step
            if abs(step) < 1e-6:
                break
        return flex

    def solve(self, x, z, pitch, seed):
        """shoulder_pan = 0 で、運搬物の中心を (x, 0, z) に、指の向きを pitch [rad] にする関節の角度。"""
        lift, elbow = seed
        flex = 1.2
        for _ in range(80):
            flex = self.level(lift, elbow, pitch, flex)
            center, _ = self.state([0.0, lift, elbow, flex, spec.WRIST_ROLL])
            error = np.array([center[0] - x, center[2] - z])
            if np.linalg.norm(error) < 1e-4:
                q = [0.0, lift, elbow, flex, spec.WRIST_ROLL]
                if all(abs(v) <= lim for v, lim in zip(q, spec.JOINT_LIMITS)):
                    return q
                raise RuntimeError("可動範囲の外")
            jac = np.zeros((2, 2))
            for k, (dl, de) in enumerate([(0.01, 0.0), (0.0, 0.01)]):
                f2 = self.level(lift + dl, elbow + de, pitch, flex)
                c2, _ = self.state([0.0, lift + dl, elbow + de, f2, spec.WRIST_ROLL])
                jac[:, k] = (np.array([c2[0], c2[2]]) - np.array([center[0], center[2]])) / 0.01
            try:
                dl, de = np.linalg.solve(jac, -error)
            except np.linalg.LinAlgError:
                break
            lift += float(np.clip(dl, -0.15, 0.15))
            elbow += float(np.clip(de, -0.15, 0.15))
        raise RuntimeError(f"(x {x:.3f}, z {z:.3f}, pitch {math.degrees(pitch):.0f}°) の姿勢が見つかりません")

    def walk(self, start_q, start_xzp, goal_xzp, n=10):
        """解けている姿勢から、目標へ少しずつ動かして解く（遠い解に飛ばないように）。"""
        q = start_q
        for t in np.linspace(0.0, 1.0, n + 1)[1:]:
            x, z, p = (a + (b - a) * t for a, b in zip(start_xzp, goal_xzp))
            q = self.solve(x, z, p, (q[1], q[2]))
        return q


def grasp_contact_angle(client, pick_q):
    """床に立てた 2 cm の運搬物に、閉じていく指が最初に触れる角度。"""
    client.holding = False
    client.apply([mi.payload_world_object(0.0)])
    angle = None
    for a in np.arange(0.30, -0.01, -0.002):
        ok, bad = client.check(pick_q, gripper=float(a))
        if not ok:
            angle = float(a) + 0.002
            touched = sorted({x for c in bad for x in c[:2] if x != mi.PAYLOAD_WORLD_ID})
            break
    client.apply([mi.payload_world_object(0.0, add=False)])
    if angle is None:
        raise RuntimeError("指を閉じても運搬物に触れません")
    return angle, touched


def main():
    rclpy.init()
    client = mi.MoveItClient("pai_v3_compute_poses")
    client.apply([mi.floor_object(True)])
    solver = Solver(client)
    down = -math.pi / 2

    # v2 の pick（半径 0.1812 m）の近くから、v3 の半径へ動かす
    q0 = [0.0, SEED[0], SEED[1], 1.1288, spec.WRIST_ROLL]
    c0, p0 = solver.state(q0)
    pick = solver.walk(q0, (c0[0], c0[2], p0), (R_X, HEIGHTS["pick"], down), n=12)
    poses = {"pick": pick}
    for name in ("place", "hover", "rise"):
        poses[name] = solver.walk(pick, (R_X, HEIGHTS["pick"], down), (R_X, HEIGHTS[name], down), n=6)

    def vertical(h_from, h_to, q):
        path = []
        step = 0.01 if h_to > h_from else -0.01
        for h in np.arange(h_from + step, h_to - step / 2, step):
            q = solver.solve(R_X, float(h), down, (q[1], q[2]))
            path.append(q)
        return path

    paths = {
        "descend_path": vertical(HEIGHTS["hover"], HEIGHTS["pick"], poses["hover"]),
        "lift_path": vertical(HEIGHTS["pick"], HEIGHTS["hover"], poses["pick"]),
        "lower_path": vertical(HEIGHTS["hover"], HEIGHTS["place"], poses["hover"]),
        "rise_path": vertical(HEIGHTS["place"], HEIGHTS["rise"], poses["place"]),
    }

    contact, touched = grasp_contact_angle(client, poses["pick"])
    grasp_angle = round(contact - 0.008, 4)
    print(f"指が運搬物に触れる角度 {contact:.3f} rad（触れたリンク：{touched}）→ 挟む角度 {grasp_angle:.3f} rad")

    via_over = []
    for pitch_deg, h in VIA_OVER:
        try:
            q = solver.walk(poses["hover"], (R_X, HEIGHTS["hover"], down), (R_X, h, math.radians(pitch_deg)), n=10)
        except RuntimeError:
            continue
        via_over.append({"pitch_deg": pitch_deg, "height": h, "q": q})
    via_front = []
    for r, h in VIA_FRONT:
        try:
            q = solver.walk(poses["hover"], (R_X, HEIGHTS["hover"], down), (spec.PAN_AXIS_X + r, h, down), n=10)
        except RuntimeError:
            continue
        via_front.append({"radius": r, "height": h, "q": q})
    via_retract = []
    for r, h, pitch_deg in VIA_RETRACT:
        try:
            q = solver.walk(poses["hover"], (R_X, HEIGHTS["hover"], down),
                            (spec.PAN_AXIS_X + r, h, math.radians(pitch_deg)), n=10)
        except RuntimeError:
            continue
        via_retract.append({"radius": r, "height": h, "pitch_deg": pitch_deg, "q": q})
    print(f"引き寄せる経由点：{len(via_retract)}/{len(VIA_RETRACT)} 個解けた")
    print(f"円柱を越える経由点：{len(via_over)}/{len(VIA_OVER)} 個解けた "
          f"{[(v['pitch_deg'], v['height']) for v in via_over]}")
    print(f"手前を通る経由点：{len(via_front)}/{len(VIA_FRONT)} 個解けた {[(v['radius'], v['height']) for v in via_front]}")

    for name, q in poses.items():
        c, p = solver.state(q)
        print(f"  {name:6s} 関節 {np.round(q, 4).tolist()}  運搬物の中心 ({c[0]:.4f}, {c[2]:.4f})、指 {math.degrees(p):.1f}°")

    client.apply([mi.floor_object(False)])
    result = {
        "arm_joints": spec.ARM_JOINTS, "arc": spec.ARC, "pan_axis_x": spec.PAN_AXIS_X, "wrist_roll": spec.WRIST_ROLL,
        "heights": HEIGHTS, "open_angle": OPEN_ANGLE, "release_angle": OPEN_ANGLE,
        "grasp_contact_angle": round(contact, 4), "grasp_angle": grasp_angle,
        **{k: [float(v) for v in q] for k, q in poses.items()},
        **{k: [[float(v) for v in q] for q in path] for k, path in paths.items()},
        "via_over": via_over, "via_front": via_front, "via_retract": via_retract,
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"保存しました：{OUTPUT}")
    client.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
