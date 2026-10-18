#!/usr/bin/env python3
"""
教師データ（data/<版>/episodes.jsonl）を、作ったときとは別に MoveIt 2 で確かめ直す。

全区間（approach〜retreat）の各点と、点の間（いちばん大きく動く関節で 0.01 rad ごと）を衝突判定する。
障害物の余裕は --clearance（既定は本番の 10 mm）。区間ごとの条件は作ったときと同じ：
  - grasp・lift・transport・lower は荷物を持った状態（Attached Collision Object、指は挟む角度）
  - approach・descend は床の荷物、release・retreat は皿の荷物をワールドに置き、指は開いた角度
  - 荷物が床・皿に載っているだけの接触は、transport 以外で許す

    python3 scripts/validate_dataset.py --version dataset_v1 --workers 6 [--clearance 0.010]
    （1 つの move_group で一部だけ：ROS_DOMAIN_ID=61 python3 scripts/validate_dataset.py --worker-mode --indices 0:10 ...）

結果：data/<版>/validation_<余裕 mm>mm.csv（1 行 = 1 エピソード）と、最後に表示する集計
"""

import argparse
import csv
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

REPO_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_DIR / "scripts"))
import spec  # noqa: E402

HOLDING = {"grasp", "lift", "transport", "lower"}
RESOLUTION = 0.01


def check_episode(client, mi, e, clearance):
    # dataset_v1 は半径 ARC の円弧の上、dataset_v2 は物ごとの距離（r）
    plate_r, payload_r = e.get("plate_r", spec.ARC), e.get("payload_r", spec.ARC)
    client.apply([mi.floor_object(True), mi.plate_object(e["plate_pan"], radius=plate_r)]
                 + [mi.obstacle_object(f"obstacle_{key}", o["shape"], o["pan"], o["yaw"], clearance,
                                       radius=o.get("r", spec.ARC))
                    for key, o in e["obstacles"].items()])
    client.grasp_angle = e["grasp_angle"]
    where_now, checks, problems = "unset", 0, []
    previous = None
    for k, w in enumerate(e["waypoints"]):
        phase, q, angle = w["phase"], np.array(w["q"]), w["gripper"]
        holding = phase in HOLDING
        where = None if holding else ((e["payload_pan"], False, payload_r) if phase in ("approach", "descend")
                                      else (e["plate_pan"], True, plate_r))
        if where != where_now:
            client.apply([mi.payload_world_object(where[0], where[1], radius=where[2]) if where
                          else mi.payload_world_object(0.0, add=False)])
            where_now = where
        client.holding = holding
        client.allow_support = phase != "transport"
        # 前の点からの間（指の状態が同じときだけ。挟む・離すは同じ姿勢で指だけ動く）
        states = [q]
        if previous is not None and previous[1] == holding and previous[2] == angle:
            a = previous[0]
            n = max(1, int(math.ceil(np.max(np.abs(q - a)) / RESOLUTION)))
            states = [a + (q - a) * t for t in np.linspace(0.0, 1.0, n + 1)[1:]]
        for s in states:
            checks += 1
            ok, bad = client.check(s, gripper=None if holding else angle)
            if not ok:
                problems.append(f"{k}:{phase}:" + "/".join(f"{a}-{b}({d * 1000:.1f}mm)" for a, b, d in bad[:2]))
                break
        previous = (q, holding, angle)
    client.apply([mi.payload_world_object(0.0, add=False), mi.plate_object(0.0, add=False)]
                 + [mi.obstacle_object(f"obstacle_{key}", "box", 0.0, 0.0, 0.0, add=False) for key in e["obstacles"]])
    return checks, problems


def worker(args):
    import rclpy
    import moveit_iface as mi
    source = Path(args.episodes) if args.episodes else REPO_DIR / "data" / args.version / "episodes.jsonl"
    episodes = [json.loads(line) for line in source.read_text().splitlines()]
    if args.indices:
        a, b = map(int, args.indices.split(":"))
        episodes = [e for e in episodes if a <= e["index"] < b]
    else:
        episodes = [e for k, e in enumerate(episodes) if k % args.workers == args.worker]
    rclpy.init()
    client = mi.MoveItClient(f"pai_v3_validate_{args.worker}")
    out = Path(args.out)
    with out.open("w") as f:
        for e in episodes:
            t0 = time.monotonic()
            checks, problems = check_episode(client, mi, e, args.clearance)
            f.write(json.dumps({"index": e["index"], "split": e["split"], "checks": checks, "problems": problems,
                                "seconds": round(time.monotonic() - t0, 2)}, ensure_ascii=False) + "\n")
            f.flush()
    client.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--version", default="dataset_v1")
    parser.add_argument("--clearance", type=float, default=spec.RUN_CLEARANCE)
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--domain-base", type=int, default=70)
    parser.add_argument("--worker-mode", action="store_true")
    parser.add_argument("--worker", type=int, default=0)
    parser.add_argument("--indices", default="")
    parser.add_argument("--out", default="")
    # 別のエピソードのファイル（例：MLP の推論軌道を差し込んだもの）と、結果の名前
    parser.add_argument("--episodes", default="")
    parser.add_argument("--name", default="validation")
    args = parser.parse_args()
    if args.worker_mode:
        worker(args)
        return 0

    from generate_parallel import READY, ROS_SETUP, start_move_group, stop
    data = REPO_DIR / "data" / args.version
    logs = REPO_DIR / "logs"
    logs.mkdir(exist_ok=True)
    tag = f"{round(args.clearance * 1000)}mm"
    groups, workers, parts = [], [], []
    try:
        for k in range(args.workers):
            groups.append(start_move_group(args.domain_base + k, logs / f"move_group_{args.domain_base + k}.log"))
        deadline = time.monotonic() + 120
        for k in range(args.workers):
            log = logs / f"move_group_{args.domain_base + k}.log"
            while READY not in log.read_text(errors="ignore"):
                if groups[k].poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(f"move_group が起動しません：{log}")
                time.sleep(1)
        t0 = time.monotonic()
        for k in range(args.workers):
            part = logs / f"validate_{args.version}_{args.name}_{tag}_{k}.jsonl"
            parts.append(part)
            command = (f"{ROS_SETUP}; export ROS_DOMAIN_ID={args.domain_base + k}; exec /usr/bin/python3 "
                       f"{Path(__file__).resolve()} --worker-mode --version {args.version} --clearance {args.clearance} "
                       f"--worker {k} --workers {args.workers} --out {part}"
                       + (f" --episodes {Path(args.episodes).resolve()}" if args.episodes else ""))
            workers.append(subprocess.Popen(["bash", "-c", command], stdout=open(logs / f"validate_worker_{k}.log", "w"),
                                            stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True))
        for w in workers:
            w.wait()
        print(f"確かめ終わりました（{(time.monotonic() - t0) / 60:.1f} 分）", flush=True)
    finally:
        for w in workers:
            stop(w)
        for g in groups:
            stop(g)

    results = sorted((json.loads(line) for p in parts if p.exists() for line in p.read_text().splitlines() if line),
                     key=lambda r: r["index"])
    out = data / f"{args.name}_{tag}.csv"
    if args.episodes:
        out = Path(args.episodes).resolve().parent / f"{args.name}_{tag}.csv"
    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["episode_id", "split", "safe", "checks", "problems"])
        for r in results:
            w.writerow([r["index"], r["split"], int(not r["problems"]), r["checks"], " | ".join(r["problems"])])
    safe = sum(1 for r in results if not r["problems"])
    by_split = {s: (sum(1 for r in results if r["split"] == s and not r["problems"]),
                    sum(1 for r in results if r["split"] == s)) for s in ("train", "val", "test")}
    print(f"余裕 {tag}：{safe}/{len(results)} 件が SAFE（"
          + "、".join(f"{s} {a}/{b}" for s, (a, b) in by_split.items())
          + f"）、判定 {sum(r['checks'] for r in results)} 回 → {out}")
    for r in results:
        if r["problems"]:
            print(f"  #{r['index']}：{r['problems'][0]}")
    return 0 if safe == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
