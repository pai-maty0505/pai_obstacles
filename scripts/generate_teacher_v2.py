#!/usr/bin/env python3
"""
dataset_v2 の教師データを作る作業プロセス（1 つの move_group につなぐ。並列には scripts/generate_parallel.py から呼ぶ）。

dataset_v1（scripts/generate_teacher.py）との違い（2026-10-14 のユーザーの指示）
  - 荷物・皿・障害物 3 個を、円弧の上ではなく、回転軸から 20 cm 以内の平面の上にばらばらに置く
    （角度は土台の前の半円 −1.5〜1.5 rad。縁と縁の距離は外接円で、障害物どうし 8 cm 以上（ハンドが間を抜けられる幅）、
    それ以外 5 cm 以上。障害物は土台からも 5 cm 以上、直方体は内側をハンドが通れるよう中心を 17.5 cm 以上）
  - 荷物と皿を両端に、障害物 3 個をその間の角度に置き、荷物から皿へ障害物の間を縫う線に沿って運ぶ
  - 通り方：円柱は上を越える。直方体は横を回る（土台の側を先に試す）。三角錐は越えられれば越え、だめなら横を回る
  - 荷物・皿の距離が毎回違うので、つかむ・置く姿勢は numpy の IK（scripts/kinematics.py）で毎回解く

運ぶ区間（2026-10-14 のユーザーの指示：決まったコースだけでなく、最短のコースを通ってよい）
  - 作業範囲の格子と障害物のまわり（横・上）に候補点を置き、関節空間で近い点どうしを結んだ道路網の上で、
    荷物の真上から皿の真上までの関節空間の道のりが最も短い経路を探す（円柱の上を越える点を必ず 1 つ通る）
  - 近道の処理と、頂点を両隣の中点へ寄せる仕上げをする
  - 見つからないときは、障害物ごとに経由点を 1 つ置いてつなぐ作り方（直線 → L 字 → 間で切り替える →
    引き寄せてから回る → OMPL）に戻る
0.01 rad ごとの衝突判定と、遠回り（逆向きに回る・高く上げすぎ・外へ出すぎ）の除外は dataset_v1 と同じ。

    ROS_DOMAIN_ID=61 python3 scripts/generate_teacher_v2.py --indices 0:10 --out /tmp/test.jsonl
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
import generate_teacher as v1  # noqa: E402
import kinematics as kin  # noqa: E402
import moveit_iface as mi  # noqa: E402
import spec  # noqa: E402

POSES = v1.POSES
SEED = 20261014
MAX_RESAMPLE = 20
TRANSPORT_TRIES = 4
OVER_RADIUS = 0.02                 # 上を越えたとみなす、荷物の中心と障害物の中心の水平距離 [m]
MAX_RADIUS = spec.R_MAX + 0.04     # 荷物の中心の、回転軸からの距離の上限 [m]
MAX_BOTTOM = 0.18                  # 荷物の下端の高さの上限 [m]
MAX_BACKTRACK = 0.05               # shoulder_pan が運ぶ向きと逆に回る量の合計の上限 [rad]
DOWN = -math.pi / 2
PIVOT_FRACTIONS = (0.5, 0.65, 0.35, 0.8, 0.2)   # 「間で切り替える」角度（a から b までの割合）
H = POSES["heights"]
# 上を越える経由点：指の向き [度] と、荷物の中心の高さ [m]（形ごと。下端が障害物の高さ + 余裕 15 mm より上）
OVER = {"cylinder": [(p, h) for p in (-90, -80, -70, -60, -50) for h in (0.12, 0.13, 0.14, 0.15, 0.16)],
        "pyramid": [(p, h) for p in (-90, -80, -70, -60, -50) for h in (0.155, 0.165, 0.175)]}
OVER_SHIFT = (0.0, 0.006, -0.006, 0.012, -0.012)   # 経由点を円周の向きにずらす量 [m]
# 横を回る経由点：障害物と同じ角度で、内側（土台の側）または外側の距離 [m] と、荷物の中心の高さ [m]。
# 内側は障害物から遠い（土台に近すぎない）方から、外側は障害物に近い方から試す。直方体は外側を回れない（腕が上をまたぐ）
AROUND_HEIGHTS = (0.06, 0.05, H["hover"], 0.04)
AROUND_INNER = (0.08, 0.05)          # 内側：距離の下限、障害物の中心からの最小のずれ [m]
AROUND_OUTER = 0.05                  # 外側：障害物の中心からの最小のずれ [m]

# ---------------------------------------------------------------- 最短の経路（2026-10-14 のユーザーの指示）
# 決まったコース（20 cm の円に沿う・障害物ごとに経由点を 1 つ）だけでなく、最も短いコースも通ってよい。
# 作業範囲に格子状の候補点と、障害物のまわり（横・上）の候補点を置き、関節空間で近い点どうしを辺で結んだ道路網の上で、
# 荷物の真上から皿の真上までの、関節空間の道のりが最も短い経路を探す（円柱の上を越える候補は必ず 1 つ通る）。
# 辺が当たるかは、最短の経路に出てきたものだけを確かめ、当たったら除いて探し直す。
RING_ANGLES = 16                      # 障害物のまわりの候補の、向きの数
RING_OFFSETS = (0.04, 0.055, 0.07)    # 障害物の外接円から、荷物の中心までの水平距離 [m]
OVER_OFFSETS = ((0.0, 0.0), (0.008, 0.0), (-0.008, 0.0), (0.0, 0.008), (0.0, -0.008))  # 真上の候補のずれ（半径, 円周） [m]
# 格子・障害物の横の候補の、荷物の中心の高さ [m] と指の向き [度]（真下向きのままでは 8 cm 程度までしか上がらない）
POSES_FREE = [(0.05, -90), (H["hover"], -90), (0.10, -70), (0.13, -60), (0.16, -50)]
# 荷物・皿の真上で、上げ下げだけする候補
LIFT = [(h, p) for h in (0.09, 0.11, 0.13, 0.15) for p in (-90, -75, -60)]
GRID_STEP = 0.015                     # 格子の間隔（距離・円周の向き） [m]
GRID_MARGIN = 0.25                    # 格子を置く角度の、荷物・皿の外への広がり [rad]
NEIGHBORS = 12                        # 1 つの点から辺を結ぶ、関節空間で近い点の数
R_RANGE = (0.09, MAX_RADIUS)          # 候補の、回転軸からの距離の範囲 [m]
R_STEP = 0.005                        # IK を使い回すため、候補の距離をこの間隔にそろえる [m]
SEARCH_ITERATIONS = 20000             # 当たる辺を除いて探し直す回数の上限
SEARCH_SECONDS = 60.0
# shoulder_pan が運ぶ向きと逆にこれより多く戻る辺は、道路網に入れない（None なら入れる。dataset_v2 は None）
BACK_EDGE = None
# 障害物から離れる経路（2026-10-14 のユーザーとの議論、issue #5）：経路の損 = 道のり ×（1 + CLEAR_WEIGHT × 近さの損）。
# 近さの損は、荷物とハンドの点から実際の障害物の表面までの距離 d が CLEAR_RANGE より近いほど大きい（0〜1）。
# CLEAR_WEIGHT = 0 なら道のりだけ（dataset_v2 と同じ最短のコース）
CLEAR_WEIGHT = 0.0
CLEAR_RANGE = 0.04
HAND_POINT_RADIUS = 0.008          # 荷物とハンドを代表する点の、まわりの厚み [m]
RELAX_ROUNDS = 6                      # 仕上げ：頂点を両隣の中点へ寄せる回数（格子のジグザグを取る）


def min_angle(r1, r2, distance):
    """回転軸から r1・r2 の 2 点の距離を distance 以上にする、角度の差の最小値。"""
    c = (r1 * r1 + r2 * r2 - distance * distance) / (2 * r1 * r2)
    return 0.0 if c >= 1.0 else (math.pi if c <= -1.0 else math.acos(c))


def gap(a, b):
    """物 a・b の縁と縁の最小距離。障害物どうしはハンドが間を抜けられる幅、それ以外は GAP。"""
    return spec.GAP_OBSTACLES if a in spec.SHAPES and b in spec.SHAPES else spec.GAP


def sample_layout(index, attempt):
    """荷物・障害物 3 個・皿を、角度の小さい順に並べて置く。

    先に距離（半径）を決め、隣り合う物の縁と縁が GAP 以上になる角度の差の最小値を求めてから、
    余った角度をランダムに配る（全部をばらばらに引いて条件で捨てるやり方では、ほとんど満たさないため）。
    隣り合わない物どうしの間隔も最後に確かめる。
    """
    rng = random.Random(SEED * 1000 + index * 37 + attempt)
    fp = spec.FOOTPRINT
    full = spec.ANGLE_RANGE[1] - spec.ANGLE_RANGE[0]
    for _ in range(1000):
        shapes = list(spec.SHAPE_NAMES)
        rng.shuffle(shapes)
        kinds = ["payload"] + shapes + ["plate"]
        radii = [rng.uniform(*spec.R_HANDLE)] + \
                [rng.uniform(max(spec.BASE_RADIUS + spec.GAP + fp[k], spec.R_MIN_BY_SHAPE.get(k, 0.0)), spec.R_MAX)
                 for k in shapes] + \
                [rng.uniform(*spec.R_HANDLE)]
        need = [min_angle(radii[i], radii[i + 1], fp[kinds[i]] + fp[kinds[i + 1]] + gap(kinds[i], kinds[i + 1]))
                for i in range(4)]
        if sum(need) > full:
            continue
        span = rng.uniform(sum(need), full)
        slack = np.random.default_rng(rng.getrandbits(32)).dirichlet(np.ones(4)) * (span - sum(need))
        lo = rng.uniform(spec.ANGLE_RANGE[0], spec.ANGLE_RANGE[1] - span)
        angles = list(lo + np.concatenate([[0.0], np.cumsum(np.array(need) + slack)]))
        objs = list(zip(kinds, angles, radii))
        xy = [spec.on_arc(a, r) for _, a, r in objs]
        if all(math.dist(xy[i], xy[j]) - fp[objs[i][0]] - fp[objs[j][0]] >= gap(objs[i][0], objs[j][0]) - 1e-9
               for i in range(5) for j in range(i + 1, 5)):
            break
    else:
        raise RuntimeError("間隔の条件を満たす配置が見つかりません")
    forward = rng.random() < 0.5      # True：荷物が shoulder_pan の小さい側、皿が大きい側（shoulder_pan が増える向きに運ぶ）
    if not forward:                   # 荷物と皿を入れ替える
        objs[0], objs[-1] = ("payload", objs[-1][1], objs[-1][2]), ("plate", objs[0][1], objs[0][2])
    out = {"index": index, "attempt": attempt, "forward": forward, "obstacles": {}}
    for kind, a, r in objs:
        x, y = spec.on_arc(a, r)
        item = {"pan": a, "r": r, "x": x, "y": y}
        if kind in ("payload", "plate"):
            out[kind] = item
        else:
            item.update(shape=kind, yaw=rng.uniform(0.0, spec.YAW_PERIOD[kind]))
            out["obstacles"][kind] = item
    order = sorted(out["obstacles"], key=lambda s: out["obstacles"][s]["pan"], reverse=not forward)
    for k, s in enumerate(order):
        out["obstacles"][s]["order"] = k
    out["order"] = order
    # validate_dataset.py などが使う値（dataset_v1 と同じ名前）
    out["payload_pan"], out["payload_r"] = out["payload"]["pan"], out["payload"]["r"]
    out["plate_pan"], out["plate_r"] = out["plate"]["pan"], out["plate"]["r"]
    return out


class Generator(v1.Generator):
    def __init__(self, client):
        super().__init__(client)
        self.fk = kin.FK()
        self.ik = kin.PlanarIK(self.fk, POSES["hover"])
        self.planar_cache = {}
        # OMPL は見つからないとき計画時間を使い切るので、短くする（見つかるときは 1 秒もかからない）
        plan = self.c.plan
        self.c.plan = lambda a, b: plan(a, b, planning_time=2.0, attempts=8)

    # ---------------------------------------------------------- 姿勢
    def planar(self, r, z, pitch, seed_name="hover"):
        key = (round(r, 4), round(z, 4), round(pitch, 4))
        if key not in self.planar_cache:
            self.planar_cache[key] = self.ik.solve(r, z, pitch, seed=POSES[seed_name])
        return self.planar_cache[key]

    def vertical(self, r, h_from, h_to):
        """高さ h_from → h_to の 1 cm ごとの経由点（両端を除く。真下向き）。"""
        out = []
        step = 0.01 if h_to > h_from else -0.01
        for h in np.arange(h_from + step, h_to - step / 2, step):
            q = self.planar(r, float(h), DOWN)
            if q is None:
                return None
            out.append(q)
        return out

    def handling(self, layout):
        p, d = layout["payload"], layout["plate"]
        names = {"hover_p": (p["r"], H["hover"]), "pick": (p["r"], H["pick"]),
                 "hover_d": (d["r"], H["hover"]), "place": (d["r"], H["place"]), "rise": (d["r"], H["rise"])}
        q = {k: self.planar(r, z, DOWN) for k, (r, z) in names.items()}
        paths = {"descend": self.vertical(p["r"], H["hover"], H["pick"]),
                 "lift": self.vertical(p["r"], H["pick"], H["hover"]),
                 "lower": self.vertical(d["r"], H["hover"], H["place"]),
                 "retreat": self.vertical(d["r"], H["place"], H["rise"])}
        if any(v is None for v in list(q.values()) + list(paths.values())):
            return None, None
        o, g = POSES["open_angle"], POSES["grasp_angle"]
        at = kin.at_pan
        pw, dw = (p["pan"], False, p["r"]), (d["pan"], True, d["r"])
        before = [("approach", at(q["hover_p"], p["pan"]), o, False, pw)]
        before += [("descend", at(x, p["pan"]), o, False, pw) for x in paths["descend"]]
        before += [("descend", at(q["pick"], p["pan"]), o, False, pw), ("grasp", at(q["pick"], p["pan"]), g, True, None)]
        before += [("lift", at(x, p["pan"]), g, True, None) for x in paths["lift"]]
        before += [("lift", at(q["hover_p"], p["pan"]), g, True, None)]
        after = [("lower", at(x, d["pan"]), g, True, None) for x in paths["lower"]]
        after += [("lower", at(q["place"], d["pan"]), g, True, None), ("release", at(q["place"], d["pan"]), o, False, dw)]
        after += [("retreat", at(x, d["pan"]), o, False, dw) for x in paths["retreat"]]
        after += [("retreat", at(q["rise"], d["pan"]), o, False, dw)]
        return before, after

    # ---------------------------------------------------------- シーン
    def set_scene(self, layout):
        d = layout["plate"]
        objects = [mi.floor_object(True), mi.plate_object(d["pan"], radius=d["r"])]
        for shape, o in layout["obstacles"].items():
            objects.append(mi.obstacle_object(f"obstacle_{shape}", shape, o["pan"], o["yaw"], spec.TEACHER_CLEARANCE,
                                              radius=o["r"]))
        self.c.apply(objects)

    def clear_scene(self):
        self.c.apply([mi.plate_object(0.0, add=False), mi.payload_world_object(0.0, add=False)]
                     + [mi.obstacle_object(f"obstacle_{s}", "box", 0.0, 0.0, 0.0, add=False) for s in spec.SHAPE_NAMES])

    def check_handling(self, points):
        where_now = "unset"
        self.c.allow_support = True
        try:
            for phase, q, angle, holding, where in points:
                if where != where_now:
                    self.c.apply([mi.payload_world_object(where[0], where[1], radius=where[2]) if where
                                  else mi.payload_world_object(0.0, add=False)])
                    where_now = where
                self.c.holding = holding
                ok, bad = self.c.check(q, gripper=angle)
                if not ok:
                    return f"{phase}: " + "、".join(f"{a}-{b}" for a, b, _ in bad[:2])
            return None
        finally:
            self.c.allow_support = False

    # ---------------------------------------------------------- 経由点
    def via_candidates(self, shape, o, mode):
        if mode == "over":
            for shift in OVER_SHIFT:
                pan = o["pan"] + shift / o["r"]
                for pitch, h in OVER[shape]:
                    q = self.planar(o["r"], h, math.radians(pitch))
                    if q is not None:
                        yield kin.at_pan(q, pan)
        else:
            radii = list(np.arange(o["r"] - AROUND_INNER[1], AROUND_INNER[0] - 1e-9, -0.005))
            if shape != "box":
                radii += list(np.arange(o["r"] + AROUND_OUTER, MAX_RADIUS - 0.005, 0.01))
            for r in radii:
                for h in AROUND_HEIGHTS:
                    q = self.planar(float(r), h, DOWN)
                    if q is not None:
                        yield kin.at_pan(q, o["pan"])

    # ---------------------------------------------------------- 障害物までの距離（経路の損に使う。衝突判定ではない）
    def hand_points(self, q):
        """荷物とハンドを代表する点：荷物の中心、上下の端、指のある左右（gripper_link の x・y 向きに 2 cm）。"""
        g = self.fk.gripper(q)
        c = self.fk.payload_center(q)
        r = g[:3, :3]
        return np.array([c, c + 0.02 * r[:, 2], c - 0.02 * r[:, 2], c + 0.02 * r[:, 0], c - 0.02 * r[:, 0],
                         c + 0.02 * r[:, 1], c - 0.02 * r[:, 1]])

    @staticmethod
    def obstacle_distance(points, layout):
        """点から実際の障害物（膨らませない形）の表面までの距離の最小値 [m]（中は負。三角錐は近似）。"""
        best = np.inf
        for shape, o in layout["obstacles"].items():
            sh = spec.SHAPES[shape]
            dx, dy, z = points[:, 0] - o["x"], points[:, 1] - o["y"], points[:, 2]
            c, sn = math.cos(-o["yaw"]), math.sin(-o["yaw"])
            lx, ly = c * dx - sn * dy, sn * dx + c * dy
            if shape == "cylinder":
                dr, dz = np.hypot(dx, dy) - sh["radius"], np.maximum(z - sh["height"], -z)
                d = np.where((dr > 0) & (dz > 0), np.hypot(dr, dz), np.maximum(dr, dz))
            elif shape == "box":
                q = np.stack([np.abs(lx) - sh["size"][0] / 2, np.abs(ly) - sh["size"][1] / 2,
                              np.abs(z - sh["size"][2] / 2) - sh["size"][2] / 2], axis=1)
                d = np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)
            else:
                h, rr = sh["height"], sh["side"] / math.sqrt(3.0)
                scale = np.clip(1 - z / h, 0, 1)
                # 高さ z の断面（正三角形）の辺までの距離（外が正）。辺の外向きの向きは頂点の向き + 60°
                d2 = np.max([np.cos(a) * lx + np.sin(a) * ly - rr / 2 * scale
                             for a in (math.pi / 3, math.pi, 5 * math.pi / 3)], axis=0)
                d = np.where(z > h, np.hypot(np.maximum(np.hypot(lx, ly), 0), z - h), np.maximum(d2, -z))
            best = min(best, float(d.min()))
        return best - HAND_POINT_RADIUS

    def closeness(self, q, layout):
        """近さの損（0〜1）。"""
        d = self.obstacle_distance(self.hand_points(q), layout)
        return float(np.clip((CLEAR_RANGE - d) / CLEAR_RANGE, 0.0, 1.0))

    def path_cost(self, path, layout, per_segment=4):
        """経路の損 = Σ 区間の道のり ×（1 + CLEAR_WEIGHT × 区間の近さの損の平均）。CLEAR_WEIGHT = 0 なら道のり。"""
        if CLEAR_WEIGHT <= 0:
            return v1.path_length(path)
        total = 0.0
        for a, b in zip(path, path[1:]):
            a, b = np.asarray(a), np.asarray(b)
            pen = np.mean([self.closeness(a + (b - a) * t, layout) for t in np.linspace(0, 1, per_segment + 1)])
            total += float(np.linalg.norm(b - a)) * (1 + CLEAR_WEIGHT * pen)
        return total

    def min_clearance(self, path, layout, per_segment=8):
        """経路の上の、荷物・ハンドの点と実際の障害物の最小の距離 [m]（比べるため）。"""
        best = np.inf
        for a, b in zip(path, path[1:]):
            a, b = np.asarray(a), np.asarray(b)
            for t in np.linspace(0, 1, per_segment + 1):
                best = min(best, self.obstacle_distance(self.hand_points(a + (b - a) * t), layout))
        return best

    def payload_xy_samples(self, path, per_segment=4):
        out = []
        for a, b in zip(path, path[1:]):
            for t in np.linspace(0.0, 1.0, per_segment, endpoint=False):
                out.append(self.fk.payload_center(np.asarray(a) + (np.asarray(b) - np.asarray(a)) * t))
        out.append(self.fk.payload_center(path[-1]))
        return np.array(out)

    def policy_ok(self, path, layout):
        centers = self.payload_xy_samples(path)
        radius = np.hypot(centers[:, 0] - spec.PAN_AXIS_X, centers[:, 1])
        if radius.max() > MAX_RADIUS:
            return f"遠回り：外へ出すぎ（半径 {radius.max() * 100:.1f} cm）"
        pans = np.array([q[0] for q in path])
        steps = np.diff(pans) * (1 if layout["forward"] else -1)
        if -steps[steps < 0].sum() > MAX_BACKTRACK:
            return f"遠回り：逆向きに回る（{-steps[steps < 0].sum():.2f} rad）"
        bottom = centers[:, 2] - spec.PAYLOAD_SIZE[2] / 2
        if bottom.max() > MAX_BOTTOM:
            return f"遠回り：高く上げすぎ（下端 {bottom.max() * 100:.1f} cm）"
        c = layout["obstacles"]["cylinder"]
        dist = np.hypot(centers[:, 0] - c["x"], centers[:, 1] - c["y"])
        if dist.min() > OVER_RADIUS:
            return f"円柱の上を通っていない（最も近くて {dist.min() * 100:.1f} cm）"
        return None

    # ---------------------------------------------------------- 経由点の間をつなぐ
    def connect(self, a, b):
        """dataset_v1 のつなぎ方（直線 → L 字 → 引き寄せてから回る → OMPL）に、「間で切り替える」を加える。

        間で切り替える：a の形のまま 2 つの経由点の間の角度 m まで回り、そこで b の形に変えてから b まで回る。
        内側を低く回る姿勢と、円柱の上を高く越える姿勢の切り替えを、隣り合う障害物の間の空いたところで行うため。
        """
        a, b = list(a), list(b)
        if self.segment_valid(a, b):
            return [a, b], "straight"
        for m in ([a[0]] + b[1:], [b[0]] + a[1:]):
            if self.segment_valid(a, m) and self.segment_valid(m, b):
                return [a, m, b], "L"
        for t in PIVOT_FRACTIONS:
            pan = a[0] + (b[0] - a[0]) * t
            m1, m2 = [pan] + a[1:], [pan] + b[1:]
            if self.segment_valid(a, m1) and self.segment_valid(m1, m2) and self.segment_valid(m2, b):
                return [a, m1, m2, b], "pivot"
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
            if not all(self.segment_valid(p, q) for p, q in zip(raw, raw[1:])):
                continue
            found.append([list(map(float, q)) for q in self.shortcut(raw)])
        if not found:
            return None, "no_path"
        return min(found, key=v1.path_length), "ompl"

    # ---------------------------------------------------------- 運ぶ
    def transport(self, layout, modes):
        start, goal = layout["_start"], layout["_goal"]
        self.c.holding = True
        vias = []
        for shape in layout["order"]:
            o = layout["obstacles"][shape]
            chosen = next((q for q in self.via_candidates(shape, o, modes[shape]) if self.c.valid(q)), None)
            if chosen is None:
                return None, f"{shape}（{modes[shape]}）の経由点が取れない", {}
            vias.append(chosen)
        points = [start] + vias + [goal]
        path, methods = [list(start)], []
        for a, b in zip(points, points[1:]):
            seg, how = self.connect(a, b)
            methods.append(how)
            if seg is None:
                return None, f"経由点の間の経路が見つからない（{len(methods)} 区間目）", {"methods": methods}
            path += seg[1:]
        smoothed = [list(map(float, q)) for q in self.shortcut(path)]
        if len(smoothed) < len(path) and self.policy_ok(smoothed, layout) is None:
            path = smoothed
        path = v1.resample_keep_vertices(path)
        if path is None:
            return None, "経路の頂点が多すぎる", {"methods": methods}
        if not self.c.valid(path[0]) or not all(self.segment_valid(a, b) for a, b in zip(path, path[1:])):
            return None, "補間後の経路が当たる", {"methods": methods}
        reason = self.policy_ok(path, layout)
        if reason:
            return None, reason, {"methods": methods}
        return path, None, {"methods": methods}

    # ---------------------------------------------------------- 最短の経路
    def pose_at(self, x, y, z, pitch):
        """荷物の中心を床の上の (x, y)・高さ z に、指の向きを pitch にする関節（距離は R_STEP にそろえる）。"""
        pan, r = spec.pan_of(x, y)
        r = round(r / R_STEP) * R_STEP
        if not (R_RANGE[0] <= r <= R_RANGE[1]) or abs(pan) > spec.JOINT_LIMITS[0]:
            return None
        q = self.planar(r, z, pitch)
        return None if q is None else kin.at_pan(q, pan)

    def candidates(self, layout, modes):
        """道路網の点：(関節, 円柱の上を越える点か) の列。荷物・皿の真上、障害物のまわり、作業範囲の格子。"""
        nodes = []
        p, d = layout["payload"], layout["plate"]
        for obj in (p, d):
            nodes += [(q, False) for h, a in LIFT if (q := self.pose_at(obj["x"], obj["y"], h, math.radians(a)))]
        for shape in layout["order"]:
            o = layout["obstacles"][shape]
            if modes[shape] == "over":
                ux, uy = (o["x"] - spec.PAN_AXIS_X) / o["r"], o["y"] / o["r"]   # 半径の向き
                for dr, dt in OVER_OFFSETS:
                    x, y = o["x"] + ux * dr - uy * dt, o["y"] + uy * dr + ux * dt
                    nodes += [(q, shape == "cylinder") for pitch, h in OVER[shape]
                              if (q := self.pose_at(x, y, h, math.radians(pitch)))]
            for k in range(RING_ANGLES):
                t = 2 * math.pi * k / RING_ANGLES
                for off in RING_OFFSETS:
                    rr = spec.FOOTPRINT[shape] + off
                    x, y = o["x"] + rr * math.cos(t), o["y"] + rr * math.sin(t)
                    nodes += [(q, False) for h, a in POSES_FREE if (q := self.pose_at(x, y, h, math.radians(a)))]
        lo = min(p["pan"], d["pan"]) - GRID_MARGIN
        hi = max(p["pan"], d["pan"]) + GRID_MARGIN
        for r in np.arange(R_RANGE[0], R_RANGE[1] + 1e-9, GRID_STEP):
            for pan in np.arange(lo, hi + 1e-9, GRID_STEP / r):
                x, y = spec.on_arc(float(pan), float(r))
                nodes += [(q, False) for h, a in POSES_FREE if (q := self.pose_at(x, y, h, math.radians(a)))]
        return nodes

    def edge_valid(self, a, b):
        """関節空間の直線 a → b を、二分の順（中点 → 4 分の 1 …）に確かめる。当たる区間を早く見つけるため。"""
        a, b = np.asarray(a), np.asarray(b)
        n = max(1, int(math.ceil(np.max(np.abs(b - a)) / v1.RESOLUTION)))
        order, seen, step = [], set(), n
        while step >= 1:
            for k in range(step, n + 1, step):
                if k not in seen:
                    seen.add(k)
                    order.append(k)
            step //= 2
        return all(self.c.valid(a + (b - a) * (k / n)) for k in order)

    def shortest(self, layout, modes):
        """荷物の真上から皿の真上までの、関節空間の道のりが最も短い経路（点の列）。見つからなければ (None, 理由)。"""
        import heapq
        from scipy.spatial import cKDTree
        self.c.holding = True
        nodes = [(layout["_start"], False), (layout["_goal"], False)]
        nodes += [(q, over) for q, over in self.candidates(layout, modes) if self.c.valid(q)]
        q = np.array([n for n, _ in nodes], dtype=float)
        over = np.array([o for _, o in nodes])
        if not over.any():
            return None, "円柱の上を越える経由点が取れない"
        tree = cKDTree(q)
        dist, idx = tree.query(q, k=min(NEIGHBORS + 1, len(q)))
        adj = [dict() for _ in range(len(q))]
        sign = 1.0 if layout["forward"] else -1.0
        pen = np.array([self.closeness(x, layout) for x in q]) if CLEAR_WEIGHT > 0 else np.zeros(len(q))
        for i in range(len(q)):
            for dd, j in zip(dist[i][1:], idx[i][1:]):
                j = int(j)
                w = float(dd) * (1 + CLEAR_WEIGHT * (pen[i] + pen[j]) / 2)
                for u, v in ((i, j), (j, i)):
                    if BACK_EDGE is None or (q[v, 0] - q[u, 0]) * sign >= -BACK_EDGE:
                        adj[u][v] = w
        checked = {}
        t0 = time.monotonic()
        for _ in range(SEARCH_ITERATIONS):
            if time.monotonic() - t0 > SEARCH_SECONDS:
                return None, "最短の経路の探索が時間切れ"
            # 状態 = (点, 円柱の上を越えたか)。荷物の真上 (0, 0) から皿の真上 (1, 1) へ
            best, pred = {(0, False): 0.0}, {}
            heap = [(0.0, 0, False)]
            while heap:
                c, i, f = heapq.heappop(heap)
                if (i, f) == (1, True) or c > best.get((i, f), np.inf):
                    if (i, f) == (1, True):
                        break
                    continue
                for j, w in adj[i].items():
                    s2 = (j, f or bool(over[j]))
                    if c + w < best.get(s2, np.inf):
                        best[s2], pred[s2] = c + w, (i, f)
                        heapq.heappush(heap, (c + w, *s2))
            if (1, True) not in best:
                return None, "当たらない経路がない"
            chain = [(1, True)]
            while chain[-1] != (0, False):
                chain.append(pred[chain[-1]])
            chain = [i for i, _ in reversed(chain)]
            bad = None
            for u, v in zip(chain, chain[1:]):
                key = (min(u, v), max(u, v))
                if key not in checked:
                    checked[key] = self.edge_valid(q[u], q[v])
                if not checked[key]:
                    bad = (u, v)
                    break
            if bad is None:
                return [list(map(float, q[k])) for k in chain], None
            adj[bad[0]].pop(bad[1], None)
            adj[bad[1]].pop(bad[0], None)
        return None, "最短の経路の探索が回数切れ"

    def pyramid_mode(self, path, layout):
        """三角錐の上を越えたか（荷物の中心が三角錐の外接円の中を通ったか）、横を通ったか。"""
        o = layout["obstacles"]["pyramid"]
        c = self.payload_xy_samples(path)
        inside = np.hypot(c[:, 0] - o["x"], c[:, 1] - o["y"]) < spec.FOOTPRINT["pyramid"]
        return "over" if inside.any() else "around"

    def relax(self, path):
        """頂点を両隣の中点へ半分ずつ寄せる（前後の直線が当たらないときだけ）。格子をたどった小さなジグザグを取る。"""
        q = [np.asarray(p, dtype=float) for p in path]
        for _ in range(RELAX_ROUNDS):
            moved = False
            for i in range(1, len(q) - 1):
                c = q[i] + 0.5 * ((q[i - 1] + q[i + 1]) / 2 - q[i])
                if np.max(np.abs(c - q[i])) < 0.005:
                    continue
                if self.c.valid(c) and self.edge_valid(q[i - 1], c) and self.edge_valid(c, q[i + 1]):
                    q[i], moved = c, True
            if not moved:
                break
        return [list(map(float, p)) for p in q]

    def transport_shortest(self, layout, modes):
        path, reason = self.shortest(layout, modes)
        if path is None:
            return None, reason, {}
        # 仕上げ：近道の処理 → 頂点を寄せる → もう一度近道。短くなり、通り方（円柱の上など）が変わらないものだけ残す。
        # 障害物すれすれに寄せた経路は、点を取り直した後の確かめで当たることがあるので、仕上げた方から順に試す
        stages = [path]
        for step in (self.shortcut, self.relax, self.shortcut):
            smoothed = [list(map(float, q)) for q in step(stages[-1])]
            if self.path_cost(smoothed, layout) < self.path_cost(stages[-1], layout) and self.policy_ok(smoothed, layout) is None:
                stages.append(smoothed)
        reason = None
        for vertices in reversed(stages):
            path = v1.resample_keep_vertices(vertices)
            if path is None:
                reason = "経路の頂点が多すぎる"
                continue
            if not self.c.valid(path[0]) or not all(self.segment_valid(a, b) for a, b in zip(path, path[1:])):
                reason = "補間後の経路が当たる"
                continue
            reason = self.policy_ok(path, layout)
            if reason is None:
                return path, None, {"methods": ["shortest"], "vertices": len(vertices)}
        return None, reason, {}

    # ---------------------------------------------------------- 1 エピソード
    def episode(self, index):
        rejects = []
        for attempt in range(MAX_RESAMPLE):
            t0 = time.monotonic()
            layout = sample_layout(index, attempt)
            before, after = self.handling(layout)
            if before is None:
                rejects.append({"attempt": attempt, "reason": "つかむ・置く姿勢が解けない"})
                continue
            self.set_scene(layout)
            reason = self.check_handling(before + after)
            self.c.apply([mi.payload_world_object(0.0, add=False)])
            path, info, modes = None, {}, None
            if reason is None:
                layout["_start"] = before[-1][1]   # 荷物の真上（持ち上げた姿勢）
                layout["_goal"] = kin.at_pan(self.planar(layout["plate"]["r"], H["hover"], DOWN), layout["plate"]["pan"])
                # 最短の経路（三角錐は上を越える候補と横を通る候補の両方から選ぶ）。見つからなければ、経由点を
                # 障害物ごとに 1 つ置いてつなぐ作り方（三角錐は、まず上を越え、だめなら横を回る）
                modes = {"cylinder": "over", "box": "around", "pyramid": "over"}
                path, reason, info = self.transport_shortest(layout, modes)
                for k in range(TRANSPORT_TRIES if reason else 0):
                    modes = {"cylinder": "over", "box": "around", "pyramid": "over" if k < 2 else "around"}
                    path, reason, info = self.transport(layout, modes)
                    if reason is None:
                        break
                if reason is None:
                    modes = {**modes, "pyramid": self.pyramid_mode(path, layout)}
                layout.pop("_start")
                layout.pop("_goal")
            if reason is not None:
                rejects.append({"attempt": attempt, "reason": reason})
                continue
            g = POSES["grasp_angle"]
            rnd = lambda q: [round(float(v), 6) for v in q]  # noqa: E731
            waypoints = [{"phase": ph, "q": rnd(q), "gripper": a} for ph, q, a, _, _ in before]
            waypoints += [{"phase": "transport", "q": rnd(q), "gripper": g} for q in path[1:]]
            waypoints += [{"phase": ph, "q": rnd(q), "gripper": a} for ph, q, a, _, _ in after]
            return {**layout, "modes": modes, "transport": [rnd(q) for q in path], "waypoints": waypoints,
                    "transport_length": round(v1.path_length(path), 4), "methods": info.get("methods", []),
                    "rejects": rejects, "seconds": round(time.monotonic() - t0, 2),
                    "open_angle": POSES["open_angle"], "grasp_angle": g}
        return {"index": index, "failed": True, "rejects": rejects}


# ---------------------------------------------------------------- まとめるとき（generate_parallel.py）に使う
def layout_key(e):
    return ("-".join(e["order"]), e["forward"])


def yaw_features(shape, yaw):
    if shape == "cylinder":
        return 0.0, 1.0
    a = yaw * 2 * math.pi / spec.YAW_PERIOD[shape]
    return math.sin(a), math.cos(a)


def feature_columns():
    cols = ["forward", "payload_r", "payload_pan", "plate_r", "plate_pan"]
    for s in spec.SHAPE_NAMES:
        cols += [f"{s}_r", f"{s}_pan", f"{s}_yaw_sin", f"{s}_yaw_cos", f"{s}_order"]
    return cols


def feature_values(e):
    row = {"forward": int(e["forward"]), "payload_r": e["payload"]["r"], "payload_pan": e["payload"]["pan"],
           "plate_r": e["plate"]["r"], "plate_pan": e["plate"]["pan"]}
    for s in spec.SHAPE_NAMES:
        o = e["obstacles"][s]
        row[f"{s}_r"], row[f"{s}_pan"], row[f"{s}_order"] = o["r"], o["pan"], o["order"]
        row[f"{s}_yaw_sin"], row[f"{s}_yaw_cos"] = yaw_features(s, o["yaw"])
    return {k: (round(v, 6) if isinstance(v, float) else v) for k, v in row.items()}


def extra_columns():
    return [f"{s}_yaw" for s in spec.SHAPE_NAMES]


def extra_values(e):
    return {f"{s}_yaw": round(e["obstacles"][s]["yaw"], 6) for s in spec.SHAPE_NAMES}


def summary_columns():
    return ["order", "forward", "payload_r", "payload_pan", "plate_r", "plate_pan", "pyramid_mode"]


def summary_values(e):
    return ["-".join(e["order"]), int(e["forward"]), round(e["payload"]["r"], 4), round(e["payload"]["pan"], 4),
            round(e["plate"]["r"], 4), round(e["plate"]["pan"], 4), e["modes"]["pyramid"]]


def meta():
    return {"generator": "scripts/generate_teacher_v2.py", "seed": SEED, "r_max": spec.R_MAX, "r_handle": spec.R_HANDLE,
            "angle_range": spec.ANGLE_RANGE, "base_radius": spec.BASE_RADIUS, "gap": spec.GAP,
            "gap_obstacles": spec.GAP_OBSTACLES,
            "footprint": spec.FOOTPRINT, "r_min_by_shape": spec.R_MIN_BY_SHAPE, "over": OVER,
            "around_heights": AROUND_HEIGHTS, "around_inner": AROUND_INNER, "around_outer": AROUND_OUTER, "over_radius": OVER_RADIUS,
            "max_radius": MAX_RADIUS, "max_bottom": MAX_BOTTOM, "max_backtrack": MAX_BACKTRACK,
            "transport": "shortest（道路網の最短経路）、見つからなければ経由点を 1 つずつ置く作り方",
            "ring_angles": RING_ANGLES, "ring_offsets": RING_OFFSETS, "over_offsets": OVER_OFFSETS,
            "poses_free": POSES_FREE, "lift": LIFT, "grid_step": GRID_STEP, "grid_margin": GRID_MARGIN,
            "neighbors": NEIGHBORS, "r_range": R_RANGE, "r_step": R_STEP, "relax_rounds": RELAX_ROUNDS}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--indices")
    parser.add_argument("--worker", type=int, default=0)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--episodes", type=int, default=0)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    indices = list(v1.parse_indices(args.indices)) if args.indices else \
        [i for i in range(args.episodes) if i % args.workers == args.worker]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    done = set()
    if out.exists():
        done = {json.loads(line)["index"] for line in out.read_text().splitlines() if line.strip()}
    todo = [i for i in indices if i not in done]
    rclpy.init()
    client = mi.MoveItClient(f"pai_v3_generator_v2_{args.worker}")
    gen = Generator(client)
    t0 = time.monotonic()
    with out.open("a") as f:
        for k, index in enumerate(todo):
            result = gen.episode(index)
            f.write(json.dumps(result, ensure_ascii=False) + "\n")
            f.flush()
            status = "失敗" if result.get("failed") else \
                f"OK（引き直し {len(result['rejects'])}、三角錐 {result['modes']['pyramid']}、{'/'.join(result['methods'])}）"
            print(f"[worker {args.worker}] {k + 1}/{len(todo)} index {index}：{status}　経過 {time.monotonic() - t0:.0f} 秒",
                  flush=True)
    gen.clear_scene()
    client.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
