#!/usr/bin/env python3
"""
模倣学習の教師データ（MoveIt で衝突判定を通した関節軌道）を作る作業プロセス。1 つの move_group につなぐ。

並列に作るときは scripts/generate_parallel.py から呼ぶ（ROS_DOMAIN_ID ごとに move_group と作業プロセスを 1 組）。
エピソードの番号 index ごとに乱数の種を決めるので、どの作業プロセスが作っても同じ番号なら同じ配置になる。

1 エピソードの配置（README 1.3・2.1・2.2）
  - 障害物 3 種類を左・中央・右（shoulder_pan 0.45 / 0.80 / 1.15 rad）に 1 個ずつランダムに割り当て、向きもランダム
  - 運搬物とお皿は、障害物の両側（-0.25〜0.00 rad と 1.60〜1.85 rad）。どちらに運搬物を置くかもランダム
  - 運搬物は床に立て、面をパッドの向きにそろえる。お皿は床に置く

1 エピソードの動き（関節の列。各点に指の角度を付ける）
  物体の真上で構える → 下ろす → 挟む → 持ち上げる → 運ぶ（transport）→ お皿に下ろす → 離す → 真上に上げる
  運ぶ区間以外は data/poses.json の姿勢の shoulder_pan を置き換えたもの。運ぶ区間は次のように作る：
    障害物を通る順に経由点を置く。円柱は上を越える経由点（via_over）、直方体と三角錐は手前の経由点（via_front）。
    経由点の間は、関節空間の直線で当たらなければ直線、当たれば OMPL（近道の処理をする）。つないだ経路を
    関節空間の道のりで STEPS 点に取り直し、各区間を 5 分割して確かめる。
    最後に通り方を確かめる：円柱は運搬物が中心から水平 2 cm 以内を通る（上を越えた）、直方体は手前を通る。

衝突判定は余裕 15 mm（spec.TEACHER_CLEARANCE）を含む障害物で行う。持っている運搬物は Attached Collision Object、
床・お皿に置いた運搬物はワールドの物体（載っている接触だけは許す）。

    ROS_DOMAIN_ID=61 python3 scripts/generate_teacher.py --indices 0:10 --out data/dataset_v1/parts/test.jsonl
"""

import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

import numpy as np
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import moveit_iface as mi  # noqa: E402
import spec  # noqa: E402

REPO_DIR = Path(__file__).resolve().parent.parent
POSES = json.loads((REPO_DIR / "data" / "poses.json").read_text())
STEPS = 50
SEED = 20261013
MAX_RESAMPLE = 20         # 1 つの番号で配置を引き直す回数の上限
OBSTACLE_IDS = {slot: f"obstacle_{slot}" for slot in spec.SLOT_NAMES}
RESOLUTION = 0.01         # 関節空間の直線を確かめる間隔（いちばん大きく動く関節で） [rad]
OVER_RADIUS = 0.02        # 円柱の上を越えたとみなす、運搬物の中心と円柱の中心の水平距離 [m]
# 遠回りの経路を捨てる目安（MLP が真似しやすいよう、通り方をそろえるため）
MAX_RADIUS = spec.ARC + 0.03      # 運搬物の中心の、回転軸からの距離の上限（障害物の裏側を通らない） [m]
MAX_BOTTOM = 0.17                 # 運搬物の下端の高さの上限（円柱 8 cm を越えるのに十分な高さ） [m]
MAX_BACKTRACK = 0.05              # shoulder_pan が運ぶ向きと逆に回る量の合計の上限 [rad]
TRANSPORT_TRIES = 5               # 同じ配置で運ぶ区間を作り直す回数


def joints(name, pan):
    return [float(pan)] + [float(v) for v in POSES[name][1:]]


def path_at(name, pan):
    return [[float(pan)] + [float(v) for v in q[1:]] for q in POSES[name]]


def resample(points, steps=STEPS):
    q = np.array(points, dtype=float)
    seg = np.linalg.norm(np.diff(q, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 0:
        return np.repeat(q[:1], steps, axis=0)
    t = np.linspace(0.0, s[-1], steps)
    return np.array([[np.interp(v, s, q[:, j]) for j in range(q.shape[1])] for v in t])


def resample_keep_vertices(points, steps=STEPS):
    """折れ線の頂点を残したまま steps 点にする（各辺に、長さに比例した数の点を割り振る）。

    道のりで等間隔に取り直すと、頂点（障害物のすぐ外側の角）を近道して当たることがあるため。
    頂点が多すぎて残せないときは None。
    """
    q = [np.asarray(p, dtype=float) for p in points]
    # 同じ点が続くものを除く
    q = [p for i, p in enumerate(q) if i == 0 or np.linalg.norm(p - q[i - 1]) > 1e-9]
    edges = len(q) - 1
    if edges <= 0:
        return np.repeat(q[:1], steps, axis=0)
    if edges > steps - 1:
        return None
    lengths = np.array([np.linalg.norm(b - a) for a, b in zip(q, q[1:])])
    share = lengths / lengths.sum() * (steps - 1)
    n = np.maximum(1, np.floor(share).astype(int))
    while n.sum() < steps - 1:
        n[np.argmax(share - n)] += 1
    while n.sum() > steps - 1:
        k = np.argmax(np.where(n > 1, n - share, -np.inf))
        n[k] -= 1
    out = []
    for a, b, m in zip(q, q[1:], n):
        out += [a + (b - a) * t for t in np.arange(m) / m]
    out.append(q[-1])
    return np.array(out)


def path_length(path):
    return float(np.sum(np.linalg.norm(np.diff(np.asarray(path), axis=0), axis=1)))


def sample_layout(index, attempt):
    rng = random.Random(SEED * 1000 + index * 37 + attempt)
    shapes = list(spec.SHAPE_NAMES)
    rng.shuffle(shapes)
    layout = {slot: {"shape": shape, "pan": spec.SLOTS[slot], "yaw": rng.uniform(0.0, spec.YAW_PERIOD[shape])}
              for slot, shape in zip(spec.SLOT_NAMES, shapes)}
    near, far = rng.uniform(*spec.NEAR_RANGE), rng.uniform(*spec.FAR_RANGE)
    forward = rng.random() < 0.5            # True：運搬物が左（shoulder_pan の小さい側）、お皿が右
    payload_pan, plate_pan = (near, far) if forward else (far, near)
    return {"index": index, "attempt": attempt, "obstacles": layout, "payload_pan": payload_pan,
            "plate_pan": plate_pan, "forward": forward}


class Generator:
    def __init__(self, client, shortcut_iterations=20):
        self.c = client
        self.c.grasp_angle = POSES["grasp_angle"]
        self.shortcut_iterations = shortcut_iterations
        self.rng = random.Random(SEED)

    # ---------------------------------------------------------- シーン
    def set_scene(self, layout):
        objects = [mi.floor_object(True), mi.plate_object(layout["plate_pan"])]
        for slot, o in layout["obstacles"].items():
            objects.append(mi.obstacle_object(OBSTACLE_IDS[slot], o["shape"], o["pan"], o["yaw"], spec.TEACHER_CLEARANCE))
        self.c.apply(objects)

    def clear_scene(self):
        self.c.apply([mi.plate_object(0.0, add=False), mi.payload_world_object(0.0, add=False)]
                     + [mi.obstacle_object(i, "box", 0.0, 0.0, 0.0, add=False) for i in OBSTACLE_IDS.values()])

    # ---------------------------------------------------------- つかむ・置く
    def handling(self, layout):
        """つかむ・置く動作の各点：(区間, 関節, 指の角度, 持っているか, ワールドの運搬物の場所)"""
        p_obj, p_plate = layout["payload_pan"], layout["plate_pan"]
        o, g = POSES["open_angle"], POSES["grasp_angle"]
        before = [("approach", joints("hover", p_obj), o, False, (p_obj, False))]
        before += [("descend", q, o, False, (p_obj, False)) for q in path_at("descend_path", p_obj)]
        before += [("descend", joints("pick", p_obj), o, False, (p_obj, False)),
                   ("grasp", joints("pick", p_obj), g, True, None)]
        before += [("lift", q, g, True, None) for q in path_at("lift_path", p_obj)]
        before += [("lift", joints("hover", p_obj), g, True, None)]
        after = [("lower", q, g, True, None) for q in path_at("lower_path", p_plate)]
        after += [("lower", joints("place", p_plate), g, True, None),
                  ("release", joints("place", p_plate), o, False, (p_plate, True))]
        after += [("retreat", q, o, False, (p_plate, True)) for q in path_at("rise_path", p_plate)]
        after += [("retreat", joints("rise", p_plate), o, False, (p_plate, True))]
        return before, after

    def check_handling(self, points):
        where_now = "unset"
        self.c.allow_support = True
        for phase, q, angle, holding, where in points:
            if where != where_now:
                self.c.apply([mi.payload_world_object(*where) if where else mi.payload_world_object(0.0, add=False)])
                where_now = where
            self.c.holding = holding
            ok, bad = self.c.check(q, gripper=angle)
            if not ok:
                self.c.allow_support = False
                return f"{phase}: " + "、".join(f"{a}-{b}" for a, b, _ in bad[:2])
        self.c.allow_support = False
        return None

    # ---------------------------------------------------------- 運ぶ
    def segment_valid(self, a, b, resolution=RESOLUTION):
        a, b = np.asarray(a), np.asarray(b)
        n = max(1, int(math.ceil(np.max(np.abs(b - a)) / resolution)))
        return all(self.c.valid(a + (b - a) * t) for t in np.linspace(0.0, 1.0, n + 1)[1:])

    def shortcut(self, path):
        """近道の処理。まず先頭から、なるべく遠くの点へ直線で飛ぶ（貪欲法）。その後、ランダムな 2 点を結ぶのを繰り返す。"""
        q = [np.asarray(p) for p in path]
        out, i = [q[0]], 0
        while i < len(q) - 1:
            n = len(q) - 1 - i
            for j in sorted({len(q) - 1, i + (3 * n) // 4, i + n // 2, i + n // 4, i + 1}, reverse=True):
                if j > i and (j == i + 1 or self.segment_valid(q[i], q[j])):
                    out.append(q[j])
                    i = j
                    break
        q = out
        for _ in range(self.shortcut_iterations):
            if len(q) < 3:
                break
            i, j = sorted(self.rng.sample(range(len(q)), 2))
            if j - i >= 2 and self.segment_valid(q[i], q[j]):
                q = q[:i + 1] + q[j:]
        return q

    def connect(self, a, b):
        """a → b（物を持った状態）。次の順に試し、最初に当たらなかったものを使う。

        1. 関節空間の直線
        2. L 字：先に腕の形（shoulder_pan 以外）を b に変えてから回る／先に回ってから形を変える
        3. OMPL（2 本作って近道の処理をし、短い方）
        L 字は「持ち上げてから回る」「回ってから下ろす」のような、毎回同じ形の動きになるので、OMPL より優先する。
        """
        a, b = list(a), list(b)
        if self.segment_valid(a, b):
            return [a, b], "straight"
        for m in ([a[0]] + b[1:], [b[0]] + a[1:]):
            if self.segment_valid(a, m) and self.segment_valid(m, b):
                return [a, m, b], "L"
        # 引き寄せてから回る：a の位置で高さを保ったまま土台の側へ引き（u）、b の位置まで回ってから b の形にする
        for u in POSES["via_retract"]:
            m1, m2 = [a[0]] + list(u["q"][1:]), [b[0]] + list(u["q"][1:])
            if self.segment_valid(a, m1) and self.segment_valid(m1, m2) and self.segment_valid(m2, b):
                return [a, m1, m2, b], "retract"
        found, misses = [], 0
        for _ in range(6):
            if len(found) >= 3 or (misses >= 2 and not found):
                break
            raw = self.c.plan(a, b)
            if raw is None:
                misses += 1
                continue
            # OMPL の点の間も同じ間隔で確かめる（OMPL の判定の間隔とそろっていないため）
            if not all(self.segment_valid(p, q) for p, q in zip(raw, raw[1:])):
                continue
            found.append([list(map(float, q)) for q in self.shortcut(raw)])
        if not found:
            return None, "no_path"
        return min(found, key=path_length), "ompl"

    def via_candidates(self, slot, o):
        pan = o["pan"]
        if o["shape"] == "cylinder":
            # 円柱の中心の真上と、左右に少しずらした位置（半径 20 cm で 0.6・1.2 cm。上を越えた判定の 2 cm 以内）
            return [[pan + d] + list(v["q"][1:]) for d in (0.0, 0.03, -0.03, 0.06, -0.06) for v in POSES["via_over"]]
        return [[pan] + list(v["q"][1:]) for v in POSES["via_front"]]

    def payload_xy_samples(self, path, per_segment=4):
        out = []
        for a, b in zip(path, path[1:]):
            for t in np.linspace(0.0, 1.0, per_segment, endpoint=False):
                c, _ = self.c.payload_center(np.asarray(a) + (np.asarray(b) - np.asarray(a)) * t)
                out.append(c)
        c, _ = self.c.payload_center(path[-1])
        out.append(c)
        return np.array(out)

    def policy_ok(self, path, layout):
        centers = self.payload_xy_samples(path)
        radius = np.hypot(centers[:, 0] - spec.PAN_AXIS_X, centers[:, 1])
        if radius.max() > MAX_RADIUS:
            return f"遠回り：障害物の裏側を通る（半径 {radius.max() * 100:.1f} cm）"
        pans = np.array([q[0] for q in path])
        steps = np.diff(pans) * (1 if layout["forward"] else -1)
        if -steps[steps < 0].sum() > MAX_BACKTRACK:
            return f"遠回り：逆向きに回る（{-steps[steps < 0].sum():.2f} rad）"
        bottom = centers[:, 2] - spec.PAYLOAD_SIZE[2] / 2
        if bottom.max() > MAX_BOTTOM:
            return f"遠回り：高く上げすぎ（下端 {bottom.max() * 100:.1f} cm）"
        for slot, o in layout["obstacles"].items():
            cx, cy = spec.on_arc(o["pan"])
            d = np.hypot(centers[:, 0] - cx, centers[:, 1] - cy)
            if o["shape"] == "cylinder" and d.min() > OVER_RADIUS:
                return f"{slot} の円柱の上を通っていない（最も近くて {d.min() * 100:.1f} cm）"
            if o["shape"] == "box":
                pans = np.arctan2(-centers[:, 1], centers[:, 0] - spec.PAN_AXIS_X)
                radius = np.hypot(centers[:, 0] - spec.PAN_AXIS_X, centers[:, 1])
                near = np.abs(pans - o["pan"]) < 0.2
                if np.any(radius[near] > spec.ARC - 0.02):
                    return f"{slot} の直方体の手前を通っていない（半径 {radius[near].max() * 100:.1f} cm）"
        return None

    def transport(self, layout):
        p_obj, p_plate = layout["payload_pan"], layout["plate_pan"]
        start, goal = joints("hover", p_obj), joints("hover", p_plate)
        order = sorted(layout["obstacles"].items(), key=lambda kv: kv[1]["pan"], reverse=not layout["forward"])
        self.c.holding = True
        vias = []
        for slot, o in order:
            chosen = next((q for q in self.via_candidates(slot, o) if self.c.valid(q)), None)
            if chosen is None:
                return None, f"{slot}（{o['shape']}）の経由点が取れない", {}
            vias.append(chosen)
        points = [start] + vias + [goal]
        path, methods = [list(start)], []
        for a, b in zip(points, points[1:]):
            seg, how = self.connect(a, b)
            methods.append(how)
            if seg is None:
                return None, f"経由点の間の経路が見つからない（{len(methods)} 区間目）", {"methods": methods}
            path += seg[1:]
        # 仕上げ：経路全体の近道の処理（区間の継ぎ目の小さな輪を取る）。通り方が変わったら元の経路を使う
        smoothed = [list(map(float, q)) for q in self.shortcut(path)]
        if len(smoothed) < len(path) and self.policy_ok(smoothed, layout) is None:
            path = smoothed
        path = resample_keep_vertices(path)
        if path is None:
            return None, "経路の頂点が多すぎる", {"methods": methods}
        # STEPS 点に取り直した後も、全区間を同じ間隔で確かめる
        if not self.c.valid(path[0]) or not all(self.segment_valid(a, b) for a, b in zip(path, path[1:])):
            return None, "補間後の経路が当たる", {"methods": methods}
        reason = self.policy_ok(path, layout)
        if reason:
            return None, reason, {"methods": methods}
        return path, None, {"methods": methods}

    # ---------------------------------------------------------- 1 エピソード
    def episode(self, index):
        rejects = []
        for attempt in range(MAX_RESAMPLE):
            t0 = time.monotonic()
            layout = sample_layout(index, attempt)
            self.set_scene(layout)
            before, after = self.handling(layout)
            reason = self.check_handling(before + after)
            self.c.apply([mi.payload_world_object(0.0, add=False)])
            info = {}
            if reason is None:
                for _ in range(TRANSPORT_TRIES):
                    path, reason, info = self.transport(layout)
                    if reason is None or not reason.startswith(("遠回り", "経由点の間", "補間後")):
                        break
            if reason is not None:
                rejects.append({"attempt": attempt, "reason": reason})
                continue
            o, g = POSES["open_angle"], POSES["grasp_angle"]
            waypoints = [{"phase": ph, "q": [round(float(v), 6) for v in q], "gripper": a}
                         for ph, q, a, _, _ in before]
            # transport の先頭は lift の最後（荷物の真上）と同じ点なので除き、最後（皿の真上）は含める
            waypoints += [{"phase": "transport", "q": [round(float(v), 6) for v in q], "gripper": g} for q in path[1:]]
            waypoints += [{"phase": ph, "q": [round(float(v), 6) for v in q], "gripper": a} for ph, q, a, _, _ in after]
            return {**layout, "transport": [[round(float(v), 6) for v in q] for q in path],
                    "waypoints": waypoints, "transport_length": round(path_length(path), 4),
                    "methods": info.get("methods", []), "rejects": rejects,
                    "seconds": round(time.monotonic() - t0, 2), "open_angle": o, "grasp_angle": g}
        return {"index": index, "failed": True, "rejects": rejects}


# ---------------------------------------------------------------- まとめるとき（generate_parallel.py）に使う
def layout_key(e):
    return ("-".join(e["obstacles"][s]["shape"] for s in spec.SLOT_NAMES), e["forward"])


def yaw_features(shape, yaw):
    """向きの特徴：形の対称性で 1 周になるように角度を引き伸ばして sin / cos にする（円柱は 0, 1）。"""
    if shape == "cylinder":
        return 0.0, 1.0
    a = yaw * 2 * math.pi / spec.YAW_PERIOD[shape]
    return math.sin(a), math.cos(a)


def feature_columns():
    cols = ["forward", "payload_pan", "plate_pan"]
    for s in spec.SLOT_NAMES:
        cols += [f"{s}_{shape}" for shape in spec.SHAPE_NAMES] + [f"{s}_yaw_sin", f"{s}_yaw_cos"]
    return cols


def feature_values(e):
    row = {"forward": int(e["forward"]), "payload_pan": round(e["payload_pan"], 6), "plate_pan": round(e["plate_pan"], 6)}
    for s in spec.SLOT_NAMES:
        o = e["obstacles"][s]
        for shape in spec.SHAPE_NAMES:
            row[f"{s}_{shape}"] = int(o["shape"] == shape)
        row[f"{s}_yaw_sin"], row[f"{s}_yaw_cos"] = (round(v, 6) for v in yaw_features(o["shape"], o["yaw"]))
    return row


def extra_columns():
    return [f"{s}_yaw" for s in spec.SLOT_NAMES]


def extra_values(e):
    return {f"{s}_yaw": round(e["obstacles"][s]["yaw"], 6) for s in spec.SLOT_NAMES}


def summary_columns():
    return ["left", "center", "right", "forward", "payload_pan", "plate_pan"]


def summary_values(e):
    return [e["obstacles"][s]["shape"] for s in spec.SLOT_NAMES] + \
        [int(e["forward"]), round(e["payload_pan"], 4), round(e["plate_pan"], 4)]


def meta():
    return {"generator": "scripts/generate_teacher.py", "seed": SEED, "slots": spec.SLOTS, "near_range": spec.NEAR_RANGE,
            "far_range": spec.FAR_RANGE}


def parse_indices(text):
    a, b = text.split(":")
    return range(int(a), int(b))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--indices", help="作る番号の範囲 a:b")
    parser.add_argument("--worker", type=int, default=0)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--episodes", type=int, default=0, help="--indices の代わりに 0:episodes を workers で分ける")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    indices = list(parse_indices(args.indices)) if args.indices else \
        [i for i in range(args.episodes) if i % args.workers == args.worker]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    done = set()
    if out.exists():   # 途中から再開する
        for line in out.read_text().splitlines():
            if line.strip():
                done.add(json.loads(line)["index"])
    todo = [i for i in indices if i not in done]

    rclpy.init()
    client = mi.MoveItClient(f"pai_v3_generator_{args.worker}")
    gen = Generator(client)
    t0 = time.monotonic()
    with out.open("a") as f:
        for k, index in enumerate(todo):
            result = gen.episode(index)
            f.write(json.dumps(result, ensure_ascii=False) + "\n")
            f.flush()
            status = "失敗" if result.get("failed") else f"OK（引き直し {len(result['rejects'])}、{'/'.join(result['methods'])}）"
            print(f"[worker {args.worker}] {k + 1}/{len(todo)} index {index}：{status}　経過 {time.monotonic() - t0:.0f} 秒",
                  flush=True)
    gen.clear_scene()
    client.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
