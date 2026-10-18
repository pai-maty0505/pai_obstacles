#!/usr/bin/env python3
"""
v3 の MLP が推論した軌道を、Gazebo（物理シミュレーション）で実際に動かす（2026-10-14）。

v2（~/pai_obstacle_v2）の Gazebo の仕組みをそのまま使う：起動（scripts/sim_start.sh）、撮影・接触の記録
（scripts/run_episode_recorded.sh）、字幕付きの動画（scripts/make_video.py）、アームとグリッパの動かし方
（run_mission.MissionNode）。v3 用に変えるのは、物の置き方（障害物 3 個・皿・荷物を、エピソードの配置どおりに置く）と、
荷物の大きさ（2 × 2 × 4 cm）、つかむ・置く・運ぶ関節の列（v3 のエピソードの waypoints）。

1 エピソードの流れ：
  準備（動画の切り出しには入れない）
    1. 高い姿勢（TRANSIT）に上がり、次の荷物の真上の向きへ回る（前の配置のまま MoveIt で確かめる）
    2. 障害物・皿・荷物を新しい配置に置き直す（Gazebo）。MoveIt にも同じ物を置く（障害物は余裕 --clearance）
    3. 高い姿勢から荷物の真上（approach）へ下りる区間と、エピソードの全区間を MoveIt で確かめる
  本番
    4. 真下に下ろす → 指で挟む（保持は摩擦だけ）→ 真上に持ち上げる → MLP の軌道で運ぶ → 皿に下ろす → 指を開いて離す
       → 真上に上げる
  記録
    5. 運ぶ間の荷物のずれ（指から見た位置の変化）、置いた位置の皿の中心からのずれ、傾き、エピソードの長さ

v2 の run_episode_recorded.sh から実行する（撮影・接触の記録・判定・動画をまとめて行う）：

    cd ~/pai_obstacle_v2 && scripts/sim_start.sh
    scripts/run_episode_recorded.sh --script ~/pai_obstacle/scripts/gazebo_v3.py -- \\
        --episodes ~/pai_obstacle/data/dataset_L17_G15_W2/mlp_s42/test_episodes_raw.jsonl --indices 136,824,646
"""

import argparse
import json
import math
import struct
import sys
import time
from pathlib import Path

import numpy as np
import rclpy

V3_DIR = Path(__file__).resolve().parent
V2_DIR = Path.home() / "pai_obstacle_v2" / "scripts"
sys.path.insert(0, str(V3_DIR))
sys.path.append(str(V2_DIR))
import moveit_iface as mi  # noqa: E402  （v3）
import spec  # noqa: E402  （v3）
import validate_dataset  # noqa: E402  （v3）
import grasp_scene  # noqa: E402  （v2）
from grasp_test import object_pose  # noqa: E402  （v2）
from run_mission import MissionNode, sim_time  # noqa: E402  （v2）

LABEL = {"cylinder": "円柱", "box": "直方体", "pyramid": "三角錐"}
OBSTACLE_CONTACT_TOPIC = "/pai/obstacle_contacts"     # v2 の run_episode_recorded.sh が記録する
PLATE_NAME = "floor_plate"                            # 名前に「_plate」が入ると、荷物との接触が許可される
OBJECT_NAME = grasp_scene.OBJECT_NAME                 # carried_object（v2 の object_pose・接触の記録と同じ名前）
PYRAMID_STL = V3_DIR.parent / "sim" / "models" / "pyramid_v3.stl"
# 高い姿勢（荷物の中心で床から約 30 cm、指は水平）。v2 と同じ形で、手首だけ v3 の向き
TRANSIT = [-0.538, 0.011, 0.528, spec.WRIST_ROLL]
DURATIONS = {"approach": 4.0, "descend": 3.0, "close": 1.5, "lift": 2.5, "lower": 3.0, "open": 1.0, "retreat": 2.5}
TRANSPORT_SPEED = 0.4       # 運ぶ区間で、いちばん大きく動く関節の速さ [rad/s]
TRANSIT_SPEED = 0.5         # 準備の動き [rad/s]
SETTLE = 0.5                # 挟んだ後・開いた後に待つ時間 [s]
MAX_SLIP = 0.010            # 運ぶ間の荷物のずれの上限 [m]（v2 と同じ）
MAX_PLACE_ERROR = 0.010     # 置いた位置のずれの上限 [m]
MAX_TILT = 5.0              # 置いた荷物の傾きの上限 [度]


# ---------------------------------------------------------------- Gazebo の物
def write_pyramid_stl():
    """v3 の三角錐（底面 1 辺 6 cm、高さ 12 cm）のメッシュ。MoveIt と同じ頂点（moveit_iface.pyramid_vertices）。"""
    v = mi.pyramid_vertices()
    faces = [(0, 2, 1), (0, 1, 3), (1, 2, 3), (2, 0, 3)]
    PYRAMID_STL.parent.mkdir(parents=True, exist_ok=True)
    data = bytearray(b"pai_obstacle pyramid".ljust(80, b" ")) + struct.pack("<I", len(faces))
    for a, b, c in faces:
        n = np.cross(v[b] - v[a], v[c] - v[a])
        n = n / np.linalg.norm(n)
        data += struct.pack("<12fH", *n, *v[a], *v[b], *v[c], 0)
    PYRAMID_STL.write_bytes(bytes(data))


def obstacle_sdf(name, shape):
    s = spec.SHAPES[shape]
    if shape == "cylinder":
        geometry = f"<cylinder><radius>{s['radius']}</radius><length>{s['height']}</length></cylinder>"
        offset = s["height"] / 2
    elif shape == "box":
        geometry = f"<box><size>{s['size'][0]} {s['size'][1]} {s['size'][2]}</size></box>"
        offset = s["size"][2] / 2
    else:
        geometry = f"<mesh><uri>file://{PYRAMID_STL}</uri></mesh>"
        offset = 0.0
    color = {"cylinder": "0.85 0.35 0.2 1", "box": "0.25 0.6 0.3 1", "pyramid": "0.6 0.3 0.75 1"}[shape]
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision"><pose>0 0 {offset} 0 0 0</pose><geometry>{geometry}</geometry></collision>
      <visual name="visual"><pose>0 0 {offset} 0 0 0</pose><geometry>{geometry}</geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      <sensor name="contact" type="contact">
        <always_on>true</always_on><update_rate>100</update_rate>
        <contact><collision>collision</collision><topic>{OBSTACLE_CONTACT_TOPIC}</topic></contact>
      </sensor>
    </link>
  </model>
</sdf>
"""


def plate_sdf(pan):
    """皿：v3 の MoveIt と同じ形（moveit_iface.plate_parts：底の箱と、縁の箱を円く並べたもの）。"""
    body = []
    for i, (size, (x, y, z), yaw) in enumerate(mi.plate_parts(pan)):
        geometry = f"<box><size>{size[0]} {size[1]} {size[2]}</size></box>"
        f = grasp_scene.OBJECT_FRICTION
        body.append(f"""
      <collision name="collision_{i}"><pose>{x} {y} {z} 0 0 {yaw}</pose><geometry>{geometry}</geometry>
        <surface><friction><ode><mu>{f}</mu><mu2>{f}</mu2></ode></friction></surface></collision>
      <visual name="visual_{i}"><pose>{x} {y} {z} 0 0 {yaw}</pose><geometry>{geometry}</geometry>
        <material><ambient>0.92 0.92 0.88 1</ambient><diffuse>0.92 0.92 0.88 1</diffuse></material></visual>""")
    collisions = "".join(f"<collision>collision_{i}</collision>" for i in range(len(body)))
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{PLATE_NAME}">
    <static>true</static>
    <link name="link">{''.join(body)}
      <sensor name="contact" type="contact">
        <always_on>true</always_on><update_rate>100</update_rate>
        <contact>{collisions}<topic>/pai/scene_contacts</topic></contact>
      </sensor>
    </link>
  </model>
</sdf>
"""


def place_scene(e):
    """Gazebo に、エピソードの配置どおりに障害物 3 個・皿・荷物を置く（前の物は片付ける）。"""
    for name in [OBJECT_NAME, PLATE_NAME] + [f"obstacle_{s}" for s in spec.SHAPE_NAMES]:
        grasp_scene.remove(name)
    time.sleep(0.5)
    for shape, o in e["obstacles"].items():
        grasp_scene.spawn(f"obstacle_{shape}", obstacle_sdf(f"obstacle_{shape}", shape), o["x"], o["y"], 0.0, o["yaw"])
    d, p = e["plate"], e["payload"]
    grasp_scene.spawn(PLATE_NAME, plate_sdf(d["pan"]), d["x"], d["y"], 0.0)
    # 荷物は床から 2 mm 上に置いて自然に落とし、面をパッドの向きにそろえる（z 軸まわりに −pan。v3 の教師データと同じ）
    grasp_scene.OBJECT_SIZE = spec.PAYLOAD_SIZE
    grasp_scene.spawn(OBJECT_NAME, grasp_scene.object_sdf(), p["x"], p["y"], spec.PAYLOAD_SIZE[2] / 2 + 0.002, -p["pan"])
    time.sleep(1.5)


# ---------------------------------------------------------------- MoveIt での確認
def moveit_scene(client, e, clearance):
    d = e["plate"]
    objects = [mi.floor_object(True), mi.plate_object(d["pan"], radius=d["r"])]
    objects += [mi.obstacle_object(f"obstacle_{s}", s, o["pan"], o["yaw"], clearance, radius=o["r"])
                for s, o in e["obstacles"].items()]
    client.apply(objects)


def segment_safe(client, points, gripper):
    """持っていない状態で、点の列（の間も 0.01 rad ごと）が当たらないか。"""
    client.holding = False
    for a, b in zip(points, points[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(math.ceil(np.max(np.abs(b - a)) / 0.01)))
        for t in np.linspace(0, 1, n + 1):
            ok, bad = client.check(a + (b - a) * t, gripper=gripper)
            if not ok:
                return False, "、".join(f"{x}-{y}" for x, y, _ in bad[:2])
    return True, ""


# ---------------------------------------------------------------- 動かす
def even_speed(points, speed):
    """関節の列を、いちばん大きく動く関節の速さが一定になる点の列と時間にする（送る点は時間で等間隔に並ぶため）。"""
    q = np.asarray(points, float)
    step = np.max(np.abs(np.diff(q, axis=0)), axis=1)
    s = np.concatenate([[0.0], np.cumsum(step)])
    if s[-1] < 1e-9:
        return [list(q[-1])], 1.0
    n = max(2, int(math.ceil(s[-1] / 0.02)))
    t = np.linspace(0, s[-1], n + 1)[1:]
    out = np.array([[np.interp(v, s, q[:, j]) for j in range(q.shape[1])] for v in t])
    return out.tolist(), max(1.0, s[-1] / speed)


def current_arm(node):
    cur = node.current_arm_joints()
    return [cur[j] for j in spec.ARM_JOINTS]


def run_episode(node, client, e, k, total, args):
    order = "→".join(LABEL[s] for s in e["order"])
    print(f"\n========== エピソード {k}/{total}：#{e['index']}　通る順 {order} ==========", flush=True)
    wps = e["waypoints"]
    phases = {}
    for w in wps:
        phases.setdefault(w["phase"], []).append(w)
    approach = wps[0]["q"]
    open_angle = wps[0]["gripper"]
    grasp_angle = phases["grasp"][0]["gripper"]
    release_angle = phases["release"][0]["gripper"]

    # ---- 準備 ----
    here = current_arm(node)
    up = [here[0]] + TRANSIT
    over = [approach[0]] + TRANSIT
    if args.previous is not None:
        moveit_scene(client, args.previous, args.clearance)
        client.apply([mi.payload_world_object(args.previous["plate"]["pan"], True, radius=args.previous["plate"]["r"])])
        ok, why = segment_safe(client, [here, up, over], release_angle)
        if not ok:
            print(f"高い姿勢へ上がる区間が UNSAFE（{why}）。中止します。", flush=True)
            return None
    node.move_gripper(open_angle, 1.0, "準備：グリッパを開く")
    points, seconds = even_speed([here, up, over], TRANSIT_SPEED)
    if not node.arm(points, seconds, "準備：高い姿勢を通って、次の荷物の上へ"):
        return None
    node.get_logger().info("床との接触の監視：終了")      # ここから荷物を置き直す
    place_scene(e)
    moveit_scene(client, e, args.clearance)
    client.apply([mi.payload_world_object(e["payload"]["pan"], False, radius=e["payload"]["r"])])
    ok, why = segment_safe(client, [over, approach], open_angle)
    print(f"  高い姿勢から荷物の真上へ下りる：{'SAFE' if ok else 'UNSAFE ' + why}", flush=True)
    checks, problems = validate_dataset.check_episode(client, mi, e, args.clearance)
    print(f"  エピソードの全区間（{checks} 回の判定、余裕 {args.clearance * 1000:.0f} mm）：{'SAFE' if not problems else 'UNSAFE ' + problems[0]}",
          flush=True)
    present = grasp_scene.model_names()
    missing = [n for n in [OBJECT_NAME, PLATE_NAME] + [f"obstacle_{s}" for s in spec.SHAPE_NAMES] if n not in present]
    if missing or not ok or problems:
        print(f"実行しません（{'Gazebo にない：' + '、'.join(missing) if missing else '安全でない区間がある'}）", flush=True)
        return {"index": e["index"], "executed": False}
    if args.preflight_only:
        return {"index": e["index"], "executed": False, "safe": True}
    points, seconds = even_speed([over, approach], TRANSIT_SPEED)
    if not node.arm(points, max(seconds, DURATIONS["approach"]), "準備：荷物の真上へ下りる"):
        return None

    # ---- 本番 ----
    node.wait(1.0)
    start = sim_time()
    shapes = "→".join(LABEL[s] for s in e["order"])
    node.get_logger().info(f"エピソード開始: #{e['index']} {shapes} episode {e['index']}")
    ok = True
    ok &= node.arm([w["q"] for w in phases["descend"]], DURATIONS["descend"], "真下に下ろして、荷物を左右の指の間に入れる")
    ok &= node.move_gripper(grasp_angle, DURATIONS["close"], "左右のゴムのパッドで挟む（保持は摩擦だけ）")
    node.wait(SETTLE)
    grasped = node.object_in_gripper()
    ok &= node.arm([w["q"] for w in phases["lift"]], DURATIONS["lift"], "真上に持ち上げる")
    node.get_logger().info("床との接触の監視：開始")
    points, seconds = even_speed([phases["lift"][-1]["q"]] + [w["q"] for w in phases["transport"]], TRANSPORT_SPEED)
    ok &= node.arm(points, seconds, f"MLP が推論した軌道で、障害物を避けて運ぶ（{order}）")
    carried = node.object_in_gripper()
    ok &= node.arm([w["q"] for w in phases["lower"]], DURATIONS["lower"], "皿に下ろす")
    ok &= node.move_gripper(release_angle, DURATIONS["open"], "指を開いて離す")
    node.wait(SETTLE)
    ok &= node.arm([w["q"] for w in phases["retreat"]], DURATIONS["retreat"], "真上に上げて、指を荷物から抜く")
    node.get_logger().info(f"エピソード終了: #{e['index']} {shapes} episode {e['index']}")
    seconds_total = sim_time() - start
    node.wait(1.0)

    # ---- 記録 ----
    placed, tilt = object_pose()
    d = e["plate"]
    target = np.array([d["x"], d["y"], spec.PLATE_FLOOR + spec.PAYLOAD_SIZE[2] / 2])
    slip = float(np.linalg.norm(carried - grasped)) if grasped is not None and carried is not None else float("inf")
    place_error = float(np.linalg.norm(placed[:2] - target[:2])) if placed is not None else float("inf")
    height_ok = placed is not None and abs(placed[2] - target[2]) < 0.006
    result = {"index": e["index"], "order": shapes, "executed": bool(ok), "seconds": seconds_total,
              "slip_mm": slip * 1000, "place_error_mm": place_error * 1000, "tilt_deg": tilt if tilt is not None else float("nan"),
              "success": bool(ok) and slip < MAX_SLIP and place_error < MAX_PLACE_ERROR and height_ok
              and tilt is not None and tilt < MAX_TILT}
    print(f"結果：{'成功' if result['success'] else '失敗'}　エピソードの長さ {seconds_total:.1f} 秒（シミュレーション時間）、"
          f"運ぶ間のずれ {slip * 1000:.1f} mm、置いた位置のずれ {place_error * 1000:.1f} mm、"
          f"傾き {result['tilt_deg']:.1f}°、高さ {'OK' if height_ok else 'NG'}", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", required=True, help="エピソードのファイル（validate_mlp.py が作る test_episodes_raw.jsonl など）")
    parser.add_argument("--indices", required=True, help="実行するエピソードの番号（カンマ区切り、この順に実行）")
    parser.add_argument("--clearance", type=float, default=spec.RUN_CLEARANCE, help="MoveIt で確かめるときの障害物の余裕 [m]")
    parser.add_argument("--preflight-only", action="store_true", help="確かめるだけで、本番の動作はしない")
    args = parser.parse_args()
    episodes = {e["index"]: e for e in map(json.loads, Path(args.episodes).read_text().splitlines())}
    indices = [int(v) for v in args.indices.split(",")]
    write_pyramid_stl()
    print("v3：床からつかむ → MLP が推論した軌道で障害物 3 個を避けて運ぶ → 床の皿に置く（オンラインの再計画なし）", flush=True)
    print(f"エピソード：{args.episodes}　{indices}　MoveIt の確認の余裕 {args.clearance * 1000:.0f} mm", flush=True)

    rclpy.init()
    node = MissionNode()
    client = mi.MoveItClient("gazebo_v3_check")
    args.previous = None
    results = []
    try:
        for k, index in enumerate(indices, start=1):
            result = run_episode(node, client, episodes[index], k, len(indices), args)
            if result is None:
                print("準備に失敗しました。中止します。", flush=True)
                break
            results.append(result)
            args.previous = episodes[index]
    finally:
        client.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

    print("\n===== 集計 =====")
    total = 0.0
    for r in results:
        if r.get("executed"):
            total += r["seconds"]
            print(f"#{r['index']}（{r['order']}）：{'成功' if r['success'] else '失敗'}　{r['seconds']:.1f} 秒、"
                  f"ずれ {r['slip_mm']:.1f} mm、置いた位置 {r['place_error_mm']:.1f} mm、傾き {r['tilt_deg']:.1f}°")
        else:
            print(f"#{r['index']}：{'確認のみ（SAFE）' if r.get('safe') else '実行しない'}")
    if any(r.get("executed") for r in results):
        print(f"エピソードの合計：{total:.1f} 秒")
    if args.preflight_only:
        return 0 if results and all(r.get("safe") for r in results) else 1
    return 0 if len(results) == len(indices) and all(r.get("success") for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
