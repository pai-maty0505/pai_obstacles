#!/usr/bin/env python3
"""
教師データ（data/<版>/transport_{train,val,test}.csv）で、運ぶ区間の関節軌道を真似る MLP を学習する。

pai_obstacle_v2 と同じ考え方：配置の特徴と進行度（progress、0.0 = 荷物の真上、1.0 = 皿の真上）を入力に、
その時点の 5 関節を出す。運ぶ区間の 1 点（50 点のうちの 1 つ）を 1 サンプルとして学習する。
入力・出力は train の平均と標準偏差で標準化する。検証損失が最も小さいときの重みを残す。

PyTorch の入った Python で実行する（ROS 2 は使わない）：

    ~/miniconda3/bin/python scripts/train_mlp.py [--version dataset_v2] [--hidden 256,256,256,256]

出力（data/<版>/mlp/）
  model.pt               重み・入出力の列・標準化の値・構成
  history.csv            エポックごとの損失
  stats.json             条件と test の誤差
  test_predictions.jsonl test の各エピソードの推論軌道（validate_mlp.py で確かめる）
"""

import argparse
import csv
import json
import math
import random
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

REPO_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_DIR / "scripts"))
import spec  # noqa: E402

SEED = 42
# 精度：5 関節すべての誤差がこの角度以内の点の割合（エポックごとに記録する）
ACC_TOL = math.radians(3.0)
# dataset_v2 の配置の特徴（generate_teacher_v2.feature_columns と同じ）。進行度を最後に足す
FEATURES = {
    "dataset_v2": ["forward", "payload_r", "payload_pan", "plate_r", "plate_pan"]
                  + [f"{s}_{k}" for s in spec.SHAPE_NAMES for k in ("r", "pan", "yaw_sin", "yaw_cos", "order")],
}
OUTPUTS = list(spec.ARM_JOINTS)


def load(path, inputs, keep=None):
    """(入力, 出力, エピソードの番号) の配列。keep を渡すと、その番号のエピソードだけ。"""
    x, y, ids = [], [], []
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            if keep is not None and int(row["episode_id"]) not in keep:
                continue
            x.append([float(row[c]) for c in inputs])
            y.append([float(row[c]) for c in OUTPUTS])
            ids.append(int(row["episode_id"]))
    return np.array(x, dtype=np.float32), np.array(y, dtype=np.float32), np.array(ids)


class MLP(nn.Module):
    def __init__(self, n_in, hidden, n_out, dropout=0.0):
        super().__init__()
        layers, prev = [], n_in
        for h in hidden:
            layers += [nn.Linear(prev, h), nn.ReLU()] + ([nn.Dropout(dropout)] if dropout > 0 else [])
            prev = h
        layers.append(nn.Linear(prev, n_out))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


def build(checkpoint):
    model = MLP(len(checkpoint["inputs"]), checkpoint["hidden"], len(checkpoint["outputs"]), checkpoint.get("dropout", 0.0))
    model.load_state_dict(checkpoint["state_dict"])
    return model.eval()


def predict(model, checkpoint, x, device="cpu"):
    """標準化していない入力 x から、標準化を戻した関節の角度。"""
    xm, xs = np.array(checkpoint["x_mean"]), np.array(checkpoint["x_std"])
    ym, ys = np.array(checkpoint["y_mean"]), np.array(checkpoint["y_std"])
    with torch.no_grad():
        out = model(torch.tensor((x - xm) / xs, dtype=torch.float32, device=device)).cpu().numpy()
    return out * ys + ym


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--version", default="dataset_v2")
    parser.add_argument("--hidden", default="256,256,256,256")
    parser.add_argument("--batch", type=int, default=512)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--weight-decay", type=float, default=1e-5)
    parser.add_argument("--dropout", type=float, default=0.0)
    parser.add_argument("--epochs", type=int, default=3000)
    parser.add_argument("--patience", type=int, default=200)
    parser.add_argument("--out", default="")
    # 最短のコースのエピソードだけで学習・評価する（決まったコースに戻ったものを除く）
    parser.add_argument("--only-shortest", action="store_true")
    # 残す重みの選び方：loss = 検証損失（MSE）が最小、acc = 検証の精度（誤差が ACC_TOL 以内の点の割合）が最大。
    # MSE は大きく外れた点（通る側を取り違えた点）に引っ張られるので、精度で選ぶと後のエポックが選ばれることがある
    parser.add_argument("--select", choices=["loss", "acc"], default="loss")
    # 乱数の種（重みの初期値・サンプルの順番）。同じ条件を種を変えて学習し、たまたまの差をならすため
    parser.add_argument("--seed", type=int, default=SEED)
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    data = REPO_DIR / "data" / args.version
    out = Path(args.out) if args.out else data / "mlp"
    out.mkdir(parents=True, exist_ok=True)
    # dataset_v2 と同じ配置の作り方の版（dataset_v2_fine・dataset_clear*）は、同じ特徴を使う
    inputs = FEATURES.get(args.version, FEATURES["dataset_v2"]) + ["progress"]
    hidden = [int(v) for v in args.hidden.split(",")]

    keep = None
    if args.only_shortest:
        episodes = [json.loads(line) for line in (data / "episodes.jsonl").read_text().splitlines()]
        keep = {e["index"] for e in episodes if e.get("methods") == ["shortest"]}
        print(f"最短のコースのエピソードだけ使う：{len(keep)}/{len(episodes)} 件")
    xs, ys, ids = {}, {}, {}
    for split in ("train", "val", "test"):
        xs[split], ys[split], ids[split] = load(data / f"transport_{split}.csv", inputs, keep)
    x_mean, x_std = xs["train"].mean(0), xs["train"].std(0)
    y_mean, y_std = ys["train"].mean(0), ys["train"].std(0)
    x_std[x_std < 1e-8] = 1.0
    y_std[y_std < 1e-8] = 1.0   # wrist_roll は一定（WRIST_ROLL）
    t = {s: (torch.tensor((xs[s] - x_mean) / x_std, device=device),
             torch.tensor((ys[s] - y_mean) / y_std, device=device)) for s in xs}
    print(f"デバイス：{device}{'（' + torch.cuda.get_device_name(0) + '）' if device.type == 'cuda' else ''}")
    print(f"サンプル数：train {len(xs['train'])}、val {len(xs['val'])}、test {len(xs['test'])}"
          f"（エピソード {len(set(ids['train']))} / {len(set(ids['val']))} / {len(set(ids['test']))}）")

    model = MLP(len(inputs), hidden, len(OUTPUTS), args.dropout).to(device)
    print(f"構成：{len(inputs)} → {' → '.join(map(str, hidden))} → {len(OUTPUTS)}"
          f"（パラメータ {sum(p.numel() for p in model.parameters())}）")
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, factor=0.5, patience=60, min_lr=1e-6)
    loss_fn = nn.MSELoss()
    checkpoint = {"inputs": inputs, "outputs": OUTPUTS, "hidden": hidden, "dropout": args.dropout, "version": args.version,
                  "x_mean": x_mean.tolist(), "x_std": x_std.tolist(),
                  "y_mean": y_mean.tolist(), "y_std": y_std.tolist()}
    best, best_epoch, history, t0 = math.inf, 0, [], time.monotonic()
    xt, yt = t["train"]
    for epoch in range(1, args.epochs + 1):
        model.train()
        order = torch.randperm(len(xt), device=device)
        total = 0.0
        for k in range(0, len(xt), args.batch):
            b = order[k:k + args.batch]
            optimizer.zero_grad()
            loss = loss_fn(model(xt[b]), yt[b])
            loss.backward()
            optimizer.step()
            total += loss.item() * len(b)
        model.eval()
        with torch.no_grad():
            val = loss_fn(model(t["val"][0]), t["val"][1]).item()
            scale = torch.tensor(y_std, device=device)
            acc = {}
            for split in ("train", "val"):
                err = (model(t[split][0]) - t[split][1]).abs() * scale   # 標準化を戻した誤差 [rad]
                acc[f"{split}_mae_deg"] = math.degrees(err.mean().item())
                acc[f"{split}_acc"] = (err.max(dim=1).values <= ACC_TOL).float().mean().item()
        scheduler.step(val)
        lr = optimizer.param_groups[0]["lr"]
        history.append({"epoch": epoch, "train_loss": total / len(xt), "val_loss": val, **acc, "lr": lr})
        score = val if args.select == "loss" else -acc["val_acc"]
        if score < best - 1e-9:
            best, best_epoch = score, epoch
            torch.save({**checkpoint, "state_dict": model.state_dict()}, out / "model.pt")
        if epoch == 1 or epoch % 100 == 0:
            print(f"エポック {epoch:5d}：学習損失 {total / len(xt):.6f}、検証損失 {val:.6f}、"
                  f"精度 学習 {acc['train_acc'] * 100:.1f} %・検証 {acc['val_acc'] * 100:.1f} %、学習率 {lr:.1e}"
                  f"（最良 @ {best_epoch}、{time.monotonic() - t0:.0f} 秒）", flush=True)
        if epoch - best_epoch >= args.patience:
            print(f"エポック {epoch} で止めました（{args.patience} エポック、{'検証損失' if args.select == 'loss' else '検証の精度'}が良くならない）")
            break
    with (out / "history.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(history[0]))
        w.writeheader()
        w.writerows(history)

    saved = torch.load(out / "model.pt", map_location="cpu", weights_only=False)
    model = build(saved)
    result = {}
    for split in ("val", "test"):
        pred = predict(model, saved, xs[split])
        err = np.abs(pred - ys[split])
        # エピソードごとの最大の誤差（いちばんずれた点・関節）
        worst = [float(err[ids[split] == i].max()) for i in sorted(set(ids[split]))]
        result[split] = {"mae_rad": float(err.mean()), "mae_per_joint_rad": dict(zip(OUTPUTS, err.mean(0).tolist())),
                         "rmse_rad": float(np.sqrt(((pred - ys[split]) ** 2).mean())),
                         "episode_max_error_rad": {"median": float(np.median(worst)), "max": float(np.max(worst))}}
        if split == "test":
            with (out / "test_predictions.jsonl").open("w") as f:
                for i in sorted(set(ids[split])):
                    m = ids[split] == i
                    order = np.argsort(xs[split][m][:, -1])
                    f.write(json.dumps({"index": int(i), "transport": np.round(pred[m][order], 6).tolist()}) + "\n")
    stats = {"version": args.version, "only_shortest": args.only_shortest, "acc_tol_deg": math.degrees(ACC_TOL), "inputs": inputs, "outputs": OUTPUTS, "hidden": hidden, "dropout": args.dropout,
             "batch": args.batch,
             "lr": args.lr, "weight_decay": args.weight_decay, "seed": args.seed, "device": str(device),
             "select": args.select, "best_epoch": best_epoch, "best_val_loss": history[best_epoch - 1]["val_loss"], "epochs": len(history),
             "seconds": round(time.monotonic() - t0, 1), **result}
    (out / "stats.json").write_text(json.dumps(stats, ensure_ascii=False, indent=2))
    for split in ("val", "test"):
        r = result[split]
        print(f"{split}：MAE {r['mae_rad']:.4f} rad（{math.degrees(r['mae_rad']):.2f}°）、"
              f"関節ごと " + "、".join(f"{j} {v:.4f}" for j, v in r["mae_per_joint_rad"].items()) +
              f"。エピソードごとの最大誤差 中央値 {r['episode_max_error_rad']['median']:.3f}・最大 "
              f"{r['episode_max_error_rad']['max']:.3f} rad")
    print(f"保存しました：{out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
