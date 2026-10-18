#!/usr/bin/env python3
"""
Gazebo の接触センサのログ（`gz topic -e -t <トピック>` の出力）を集計し、
不許可の接触とめり込みがあったかを判定する（PLAN.md §4.2）。

    python3 scripts/analyze_contacts.py <接触ログ> [--allow <名前の一部>:<名前の一部> ...]

- 深さ（depth）が CONTACT_DEPTH 以上の点を「接触」、PENETRATION_DEPTH を超える点を「めり込み」とする。
  bullet は離れていても少し手前から接触を報告する（深さが負）ので、それは「近接」として数えるだけにする。
- --allow で許可する組を指定する（例：--allow pad:carried_object）。指定のない組の接触はすべて不許可。
- --deny-during で、許可した組でも、指定した時間帯（シミュレーション時間）の接触は不許可にする
  （例：--deny-during ground_plane:carried_object@120.5-160.2。床に置いた物体を持ち上げた後に床に触れたら落下とみなす）。
- 不許可の接触が1つでもあれば終了コード 1 を返す。
"""

import argparse
import re
import sys
from collections import defaultdict

CONTACT_DEPTH = 0.0        # [m] これ以上なら接触
PENETRATION_DEPTH = 0.001  # [m] これを超えたらめり込み（1 mm）


def short_name(name):
    """
    表示と判定に使う名前。'model::link::collision' のうち、衝突形状の名前は
    パッド（pad）や指（finger）を見分けるのに必要なので残し、URDF の変換で付く長い前置きだけを省く。
    """
    parts = name.split("::")
    if len(parts) < 3:
        return name
    collision = parts[2].split("fixed_joint_lump__")[-1]
    return f"{parts[0]}::{parts[1]}::{collision}"


def parse(path):
    """接触ログを読み、(組, 時刻, 深さの一覧) を返す。"""
    text = open(path, encoding="utf-8", errors="replace").read()
    for block in re.split(r"^contact \{", text, flags=re.M)[1:]:
        names = re.findall(r'collision[12] \{\s*id: \d+\s*name: "([^"]+)"', block)
        if len(names) != 2:
            continue
        sec = re.search(r"sec: (\d+)", block)
        nsec = re.search(r"nsec: (\d+)", block)
        stamp = (int(sec.group(1)) if sec else 0) + (int(nsec.group(1)) if nsec else 0) * 1e-9
        depths = [float(d) for d in re.findall(r"depth: (-?[\d.eE+-]+)", block)]
        pair = tuple(sorted(short_name(n) for n in names))
        yield pair, stamp, depths


def is_allowed(pair, allow_rules):
    a, b = pair
    for left, right in allow_rules:
        if (left in a and right in b) or (left in b and right in a):
            return True
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", help="gz topic -e で保存した接触ログ")
    parser.add_argument("--allow", action="append", default=[], help="許可する組（<名前の一部>:<名前の一部>）")
    parser.add_argument("--deny-during", action="append", default=[],
                        help="この時間帯は不許可にする組（<名前の一部>:<名前の一部>@<開始>-<終了>、終了は inf でもよい）")
    args = parser.parse_args()

    allow_rules = [tuple(rule.split(":", 1)) for rule in args.allow]
    deny_rules = []
    for rule in args.deny_during:
        names, span = rule.rsplit("@", 1)
        start, end = span.split("-", 1)
        deny_rules.append((tuple(names.split(":", 1)), float(start), float(end)))

    stats = defaultdict(lambda: {"messages": 0, "contact": 0, "penetration": 0, "near": 0,
                                 "max_depth": None, "first": None, "last": None})
    for pair, stamp, depths in parse(args.log):
        if any(is_allowed(pair, [names]) and start <= stamp <= end for names, start, end in deny_rules):
            pair = (pair[0], pair[1] + "（不許可の時間帯）")
        s = stats[pair]
        s["messages"] += 1
        for d in depths:
            if d > PENETRATION_DEPTH:
                s["penetration"] += 1
            if d >= CONTACT_DEPTH:
                s["contact"] += 1
            else:
                s["near"] += 1
            if s["max_depth"] is None or d > s["max_depth"]:
                s["max_depth"] = d
        if s["contact"]:
            s["first"] = stamp if s["first"] is None else min(s["first"], stamp)
            s["last"] = stamp if s["last"] is None else max(s["last"], stamp)

    print(f"===== 接触の集計：{args.log} =====")
    if not stats:
        print("接触の記録はありません。")
        print("判定：合格（不許可の接触・めり込みなし）")
        return 0

    failed = False
    for pair, s in sorted(stats.items()):
        allowed = is_allowed(pair, allow_rules) and not pair[1].endswith("（不許可の時間帯）")
        bad = s["contact"] > 0 and not allowed
        failed = failed or bad
        label = "許可" if allowed else ("不許可" if s["contact"] else "近接のみ")
        span = ""
        if s["first"] is not None:
            span = f"、接触していた時刻 {s['first']:.2f}〜{s['last']:.2f} s（シミュレーション時間）"
        print(
            f"  [{label}] {pair[0]} ↔ {pair[1]}：接触点 {s['contact']}、めり込み点 {s['penetration']}、"
            f"近接点 {s['near']}、最大深さ {s['max_depth'] * 1000:.1f} mm{span}"
        )

    if failed:
        print("判定：不合格（不許可の接触があります）")
        return 1
    print("判定：合格（不許可の接触・めり込みなし）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
