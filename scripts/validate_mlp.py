#!/usr/bin/env python3
"""
MLP の推論軌道（data/<版>/mlp/test_predictions.jsonl）を、教師データと同じ条件で確かめる。

test の各エピソードの運ぶ区間を推論軌道に差し替え、つかむ・置く区間（教師の値）とつないだエピソードを作り、
scripts/validate_dataset.py で全区間を衝突判定する（余裕は本番の 10 mm）。2 通り作る：
  raw     推論軌道そのまま（始点・終点が荷物・皿の真上からずれていれば、その間も直線で動かして確かめる）
  pinned  始点・終点を荷物・皿の真上（本番でも IK で求まる）に合わせ、ずれを進行度に比例して配って直したもの
あわせて、通り方（円柱の上を越えたか）と、教師の軌道からのずれを FK で調べる。

ROS 2 の環境（xacro・MoveIt）を読み込み、システムの Python で実行する：

    /usr/bin/python3 scripts/validate_mlp.py [--version dataset_v2] [--workers 6]

出力（data/<版>/mlp/）：test_episodes_{raw,pinned}.jsonl、mlp_{raw,pinned}_10mm.csv、mlp_metrics.json
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_DIR / "scripts"))
import kinematics  # noqa: E402
import spec  # noqa: E402

OVER_RADIUS = 0.02     # 円柱の上を越えたとみなす水平距離（generate_teacher_v2 と同じ） [m]


def pin(pred, start, goal):
    """始点・終点を start・goal に合わせる。ずれを進行度に比例して配る。"""
    pred = np.asarray(pred)
    p = np.linspace(0.0, 1.0, len(pred))[:, None]
    return pred + (1 - p) * (np.asarray(start) - pred[0]) + p * (np.asarray(goal) - pred[-1])


def with_transport(e, transport):
    before = [w for w in e["waypoints"] if w["phase"] in ("approach", "descend", "grasp", "lift")]
    after = [w for w in e["waypoints"] if w["phase"] in ("lower", "release", "retreat")]
    g = e["grasp_angle"]
    mid = [{"phase": "transport", "q": [round(float(v), 6) for v in q], "gripper": g} for q in transport]
    return {**e, "transport": [list(map(float, q)) for q in transport], "waypoints": before + mid + after}


def metrics(fk, e, transport):
    q = np.asarray(transport)
    teacher = np.asarray(e["transport"])
    c = np.array([fk.payload_center(v) for v in q])
    cyl = e["obstacles"]["cylinder"]
    over = float(np.min(np.hypot(c[:, 0] - cyl["x"], c[:, 1] - cyl["y"])))
    radius = np.hypot(c[:, 0] - spec.PAN_AXIS_X, c[:, 1])
    return {"index": e["index"], "start_gap_rad": float(np.max(np.abs(q[0] - teacher[0]))),
            "goal_gap_rad": float(np.max(np.abs(q[-1] - teacher[-1]))),
            "max_error_rad": float(np.max(np.abs(q - teacher))), "cylinder_distance_m": over,
            "over_cylinder": over <= OVER_RADIUS, "max_radius_m": float(radius.max()),
            "min_bottom_m": float(c[:, 2].min() - spec.PAYLOAD_SIZE[2] / 2)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--version", default="dataset_v2")
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--clearance", type=float, default=spec.RUN_CLEARANCE)
    parser.add_argument("--mlp", default="mlp", help="data/<版>/ の下のモデルの置き場所")
    parser.add_argument("--domain-base", type=int, default=70, help="move_group の ROS_DOMAIN_ID の始まり（生成と並行するときは変える）")
    args = parser.parse_args()
    data = REPO_DIR / "data" / args.version
    mlp = data / args.mlp
    episodes = {e["index"]: e for e in map(json.loads, (data / "episodes.jsonl").read_text().splitlines())}
    preds = [json.loads(line) for line in (mlp / "test_predictions.jsonl").read_text().splitlines()]
    fk = kinematics.FK()
    summary = {}
    for kind in ("raw", "pinned"):
        rows, made = [], []
        for p in preds:
            e = episodes[p["index"]]
            q = np.asarray(p["transport"])
            if kind == "pinned":
                q = pin(q, e["transport"][0], e["transport"][-1])
            made.append(with_transport(e, q))
            rows.append(metrics(fk, e, q))
        path = mlp / f"test_episodes_{kind}.jsonl"
        path.write_text("".join(json.dumps(m, ensure_ascii=False) + "\n" for m in made))
        print(f"[{kind}] 衝突判定（余裕 {args.clearance * 1000:.0f} mm）", flush=True)
        done = subprocess.run([sys.executable, str(REPO_DIR / "scripts" / "validate_dataset.py"), "--version", args.version,
                               "--workers", str(args.workers), "--clearance", str(args.clearance),
                               "--episodes", str(path), "--name", f"mlp_{kind}", "--domain-base", str(args.domain_base)],
                              capture_output=True, text=True)
        print(done.stdout.strip().splitlines()[-1] if done.stdout.strip() else done.stderr[-2000:], flush=True)
        csv_path = mlp / f"mlp_{kind}_{round(args.clearance * 1000)}mm.csv"
        safe = {}
        if csv_path.exists():
            import csv
            with csv_path.open() as f:
                safe = {int(r["episode_id"]): r["safe"] == "1" for r in csv.DictReader(f)}
        for r in rows:
            r["safe"] = safe.get(r["index"])
        ok = [r for r in rows if r["safe"] and r["over_cylinder"]]
        summary[kind] = {
            "episodes": len(rows), "safe": sum(1 for r in rows if r["safe"]),
            "over_cylinder": sum(1 for r in rows if r["over_cylinder"]),
            "safe_and_over_cylinder": len(ok),
            "start_gap_rad_median": float(np.median([r["start_gap_rad"] for r in rows])),
            "goal_gap_rad_median": float(np.median([r["goal_gap_rad"] for r in rows])),
            "max_error_rad_median": float(np.median([r["max_error_rad"] for r in rows])),
            "per_episode": rows}
        s = summary[kind]
        print(f"[{kind}] SAFE {s['safe']}/{s['episodes']}、円柱の上を越えた {s['over_cylinder']}/{s['episodes']}、"
              f"両方 {s['safe_and_over_cylinder']}/{s['episodes']}。始点のずれ（中央値）{s['start_gap_rad_median']:.3f} rad、"
              f"終点 {s['goal_gap_rad_median']:.3f} rad、教師からの最大のずれ（中央値）{s['max_error_rad_median']:.3f} rad",
              flush=True)
    (mlp / "mlp_metrics.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"保存しました：{mlp / 'mlp_metrics.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
