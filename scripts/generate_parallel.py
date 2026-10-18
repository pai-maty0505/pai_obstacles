#!/usr/bin/env python3
"""
教師データを並列に作る。ROS_DOMAIN_ID を分けて move_group を workers 個起動し、
それぞれに作業プロセス（scripts/generate_teacher.py）を 1 つずつつなぐ。Gazebo は使わない。

作業プロセスは番号 index を workers 個に分けて受け持ち、data/<版>/parts/worker_<k>.jsonl に 1 行ずつ書く。
途中で止めても、同じコマンドで続きから作る（できた番号は飛ばす）。最後に 1 つにまとめて、
学習用の CSV（運ぶ区間）と分割（train / val / test）を作る（--merge-only でまとめるだけ）。

    python3 scripts/generate_parallel.py --episodes 1000 --workers 6 [--version dataset_v1]
    python3 scripts/generate_parallel.py --episodes 1000 --workers 6 --version dataset_v2 --generator generate_teacher_v2

出力（data/<版>/）
  episodes.jsonl        1 行 = 1 エピソード（配置、全区間の関節の列と指の角度、運ぶ区間、引き直しの記録）
  transport.csv         運ぶ区間（STEPS 点）。1 行 = 1 点。MLP の入力にする配置の列と、5 関節
  transport_{train,val,test}.csv
  summary.csv           1 行 = 1 エピソード
  meta.json             条件と集計
"""

import argparse
import collections
import csv
import json
import math
import os
import random
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_DIR / "scripts"))
import spec  # noqa: E402

ROS_SETUP = "set +u; source /opt/ros/jazzy/setup.bash; source $HOME/so101_ws/install/setup.bash"
READY = "You can start planning now"


def start_move_group(domain, log):
    command = (f"{ROS_SETUP}; export ROS_DOMAIN_ID={domain}; exec ros2 launch so101_moveit_config move_group.launch.py "
               "use_rviz:=false use_sim_time:=false use_camera:=false")
    return subprocess.Popen(["bash", "-c", command], stdout=open(log, "w"), stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True)


def group_alive(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def stop(process):
    """プロセスグループごと止める（ros2 launch が先に終わっても、move_group が残らないように）。"""
    pgid = process.pid
    for sig, wait in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        if not group_alive(pgid):
            break
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + wait
        while group_alive(pgid) and time.monotonic() < deadline:
            time.sleep(0.2)
    try:
        process.wait(timeout=1)
    except subprocess.TimeoutExpired:
        pass


def generate(args, out_dir):
    parts = out_dir / "parts"
    parts.mkdir(parents=True, exist_ok=True)
    logs = REPO_DIR / "logs"
    logs.mkdir(exist_ok=True)
    groups, workers = [], []
    try:
        for k in range(args.workers):
            domain = args.domain_base + k
            groups.append(start_move_group(domain, logs / f"move_group_{domain}.log"))
        deadline = time.monotonic() + 120
        for k in range(args.workers):
            log = logs / f"move_group_{args.domain_base + k}.log"
            while READY not in log.read_text(errors="ignore"):
                if groups[k].poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(f"move_group（ROS_DOMAIN_ID {args.domain_base + k}）が起動しません：{log}")
                time.sleep(1)
        print(f"move_group を {args.workers} 個起動しました（ROS_DOMAIN_ID {args.domain_base}〜"
              f"{args.domain_base + args.workers - 1}）", flush=True)
        for k in range(args.workers):
            command = (f"{ROS_SETUP}; export ROS_DOMAIN_ID={args.domain_base + k}; exec /usr/bin/python3 "
                       f"{REPO_DIR / 'scripts' / (args.generator + '.py')} --episodes {args.episodes} --worker {k} "
                       f"--workers {args.workers} --out {parts / f'worker_{k}.jsonl'}")
            log = open(logs / f"generate_worker_{k}.log", "a")
            workers.append(subprocess.Popen(["bash", "-c", command], stdout=log, stderr=subprocess.STDOUT,
                                            stdin=subprocess.DEVNULL, start_new_session=True))
        t0 = time.monotonic()
        last = -1
        while any(w.poll() is None for w in workers):
            time.sleep(10)
            done = sum(len(p.read_text().splitlines()) for p in parts.glob("worker_*.jsonl"))
            if done != last and (done - last >= 25 or done == args.episodes):
                elapsed = time.monotonic() - t0
                print(f"  {done}/{args.episodes} 件（経過 {elapsed / 60:.1f} 分）", flush=True)
                last = done
        codes = [w.returncode for w in workers]
        if any(codes):
            print(f"終了コードが 0 でない作業プロセスがあります：{codes}（logs/generate_worker_*.log）", flush=True)
    finally:
        for w in workers:
            stop(w)
        for g in groups:
            stop(g)


def merge(args, out_dir):
    import importlib
    gen = importlib.import_module(args.generator)
    episodes, failed = {}, []
    for part in sorted((out_dir / "parts").glob("worker_*.jsonl")):
        for line in part.read_text().splitlines():
            if not line.strip():
                continue
            e = json.loads(line)
            if e.get("failed"):
                failed.append(e)
            else:
                episodes[e["index"]] = e
    order = sorted(episodes)
    # 分割：障害物の並び × 運ぶ向きごとに 8：1：1
    groups = collections.defaultdict(list)
    for i in order:
        e = episodes[i]
        groups[gen.layout_key(e)].append(i)
    rng = random.Random(42)
    split = {}
    for key, ids in sorted(groups.items()):
        ids = ids[:]
        rng.shuffle(ids)
        n = max(1, round(len(ids) * 0.1))
        for k, i in enumerate(ids):
            split[i] = "test" if k < n else ("val" if k < 2 * n else "train")

    with (out_dir / "episodes.jsonl").open("w") as f:
        for i in order:
            f.write(json.dumps({**episodes[i], "split": split[i]}, ensure_ascii=False) + "\n")

    feature_cols = gen.feature_columns()
    columns = ["episode_id", "split"] + feature_cols + gen.extra_columns() + ["step", "progress"] + spec.ARM_JOINTS
    rows = {"train": [], "val": [], "test": []}
    for i in order:
        e = episodes[i]
        base = {"episode_id": i, "split": split[i], **gen.feature_values(e), **gen.extra_values(e)}
        steps = len(e["transport"])
        for k, q in enumerate(e["transport"]):
            rows[split[i]].append({**base, "step": k, "progress": round(k / (steps - 1), 6),
                                   **dict(zip(spec.ARM_JOINTS, q))})

    def write(path, data):
        with path.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=columns)
            w.writeheader()
            w.writerows(data)

    write(out_dir / "transport.csv", rows["train"] + rows["val"] + rows["test"])
    for name, data in rows.items():
        write(out_dir / f"transport_{name}.csv", data)

    with (out_dir / "summary.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["episode_id", "split"] + gen.summary_columns()
                   + ["transport_length", "waypoints", "methods", "resamples", "seconds"])
        for i in order:
            e = episodes[i]
            w.writerow([i, split[i]] + gen.summary_values(e)
                       + [e["transport_length"], len(e["waypoints"]), "/".join(e["methods"]), len(e["rejects"]),
                          e["seconds"]])

    reasons = collections.Counter()
    for e in list(episodes.values()) + failed:
        for r in e["rejects"]:
            reasons[r["reason"].split("（")[0].split(":")[0]] += 1
    poses = json.loads((REPO_DIR / "data" / "poses.json").read_text())
    meta = {
        "episodes": len(order), "failed_indices": sorted(e["index"] for e in failed),
        "split": {k: sum(1 for v in split.values() if v == k) for k in ("train", "val", "test")},
        "per_layout": {f"{k[0]}/{'forward' if k[1] else 'reverse'}": len(v) for k, v in sorted(groups.items())},
        "resample_reasons": dict(reasons.most_common()),
        "steps": len(episodes[order[0]]["transport"]) if order else 0,
        **gen.meta(), "clearance": spec.TEACHER_CLEARANCE, "shapes": spec.SHAPES, "payload_size": spec.PAYLOAD_SIZE,
        "plate_diameter": spec.PLATE_DIAMETER, "wrist_roll": spec.WRIST_ROLL, "grasp_angle": poses["grasp_angle"],
        "open_angle": poses["open_angle"], "feature_columns": feature_cols, "poses": "data/poses.json",
    }
    (out_dir / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))
    print(f"まとめました：{len(order)} 件（train {meta['split']['train']}、val {meta['split']['val']}、"
          f"test {meta['split']['test']}）、作れなかった番号 {len(failed)} 件 → {out_dir}")
    return meta


def main():
    # SIGTERM でも finally（move_group の後片付け）を通す
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=1000)
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--domain-base", type=int, default=70)
    parser.add_argument("--version", default="dataset_v1")
    parser.add_argument("--generator", default="generate_teacher",
                        help="作業プロセスのスクリプト（scripts/ の中の名前。dataset_v2 は generate_teacher_v2）")
    parser.add_argument("--merge-only", action="store_true")
    args = parser.parse_args()
    try:
        import rclpy  # noqa: F401
    except ImportError:   # ROS 2 の環境を読み込んで、自分を実行し直す
        os.execvp("bash", ["bash", "-c", f"{ROS_SETUP}; exec /usr/bin/python3 " + " ".join([__file__] + sys.argv[1:])])
    out_dir = REPO_DIR / "data" / args.version
    if not args.merge_only:
        generate(args, out_dir)
    merge(args, out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
