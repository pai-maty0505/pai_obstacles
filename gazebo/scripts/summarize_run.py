#!/usr/bin/env python3
"""
scripts/run_episode_recorded.sh の記録を集計する。

- 接触の判定（scripts/analyze_contacts.py）
- 実時間係数（RTF）の平均と最小
- 撮影画像の一覧（sheet_all.png）と、接触が起きた時刻の前後の画像（sheet_contacts.png）
- 障害物に最も近づいた時刻の前後の画像を、縮小せずに並べたもの（sheet_closest.png）

    python3 scripts/summarize_run.py <記録先のディレクトリ>
"""

import bisect
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_contacts import CONTACT_DEPTH, parse  # noqa: E402

THUMB = (320, 440)
COLUMNS = 6
CONTACT_WINDOW = (-1.0, 3.0)  # 接触の開始時刻の前後に抜き出す範囲 [s]（壁時計）
CLOSEST_FRAMES = 3  # 最も近づいた時刻に近い画像を何枚並べるか


def load_stats(path):
    rows = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) != 3:
            continue
        try:
            wall, sim, rtf = (float(p) for p in parts)
        except ValueError:
            continue
        if sim > 0:
            rows.append((wall, sim, rtf))
    return rows


def sim_to_wall(stats, sim_time):
    """シミュレーション時間を壁時計に変換する（記録した対応表を線形補間する）。"""
    sims = [s for _, s, _ in stats]
    i = bisect.bisect_left(sims, sim_time)
    if i <= 0:
        return stats[0][0] - (stats[0][1] - sim_time)
    if i >= len(stats):
        return stats[-1][0] + (sim_time - stats[-1][1])
    (w0, s0, _), (w1, s1, _) = stats[i - 1], stats[i]
    if s1 == s0:
        return w0
    return w0 + (w1 - w0) * (sim_time - s0) / (s1 - s0)


def shot_time(path):
    """'2026-10-12T13:32:14.278150277.png' から壁時計の時刻を得る。"""
    stem = path.name[: -len(".png")]
    base, _, frac = stem.rpartition(".")
    return datetime.fromisoformat(base).timestamp() + float("0." + frac)


def contact_sheet(shots, out_path, label_for):
    if not shots:
        return False
    rows = (len(shots) + COLUMNS - 1) // COLUMNS
    sheet = Image.new("RGB", (COLUMNS * THUMB[0], rows * THUMB[1]), "white")
    draw = ImageDraw.Draw(sheet)
    for i, path in enumerate(shots):
        image = Image.open(path).convert("RGB")
        image.thumbnail(THUMB)
        x, y = (i % COLUMNS) * THUMB[0], (i // COLUMNS) * THUMB[1]
        sheet.paste(image, (x, y))
        draw.rectangle((x, y, x + THUMB[0] - 1, y + 18), fill="white")
        draw.text((x + 4, y + 3), label_for(path), fill="black")
    sheet.save(out_path)
    return True


def full_size_sheet(shots, out_path, label_for):
    """画像を縮小せずに横に並べる。"""
    images = [Image.open(p).convert("RGB") for p in shots]
    width = sum(i.width for i in images)
    height = max(i.height for i in images) + 20
    sheet = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(sheet)
    x = 0
    for path, image in zip(shots, images):
        sheet.paste(image, (x, 20))
        draw.text((x + 4, 4), label_for(path), fill="black")
        x += image.width
    sheet.save(out_path)


def main():
    run_dir = Path(sys.argv[1])
    shots = sorted((run_dir / "shots").glob("*.png"))
    stats = load_stats(run_dir / "stats.txt")

    print("===== 実行の記録 =====")
    print(f"撮影画像：{len(shots)} 枚")

    if stats:
        rtfs = [r for _, _, r in stats if r > 0]
        print(f"実時間係数（RTF）：平均 {sum(rtfs) / len(rtfs):.3f}、最小 {min(rtfs):.3f}（{len(rtfs)} 点）")

    # 接触の判定
    result = subprocess.run(
        [sys.executable, str(Path(__file__).resolve().parent / "analyze_contacts.py"),
         str(run_dir / "contacts.txt")],
        capture_output=True,
        text=True,
    )
    print(result.stdout.rstrip())

    start = shot_time(shots[0]) if shots else 0.0

    def label(path):
        return f"+{shot_time(path) - start:5.1f} s"

    if contact_sheet(shots, run_dir / "sheet_all.png", label):
        print(f"全画像の一覧：{run_dir / 'sheet_all.png'}")

    # 接触が始まった時刻（深さ 0 以上）の前後の画像
    contact_starts = []
    for pair, stamp, depths in parse(run_dir / "contacts.txt"):
        if any(d >= CONTACT_DEPTH for d in depths):
            contact_starts.append(stamp)
    if contact_starts and stats and shots:
        first = sim_to_wall(stats, min(contact_starts))
        last = sim_to_wall(stats, max(contact_starts))
        print(f"接触していた時刻（撮影開始から）：+{first - start:.1f} 〜 +{last - start:.1f} s")
        window = [p for p in shots if first + CONTACT_WINDOW[0] <= shot_time(p) <= last + CONTACT_WINDOW[1]]
        if contact_sheet(window, run_dir / "sheet_contacts.png", label):
            print(f"接触した時刻の前後の画像（{len(window)} 枚）：{run_dir / 'sheet_contacts.png'}")

    # 最も近づいた時刻（深さが最大の点）の前後の画像
    closest = None
    for pair, stamp, depths in parse(run_dir / "contacts.txt"):
        if depths and (closest is None or max(depths) > closest[1]):
            closest = (stamp, max(depths), pair)
    if closest and stats and shots:
        wall = sim_to_wall(stats, closest[0])
        nearest = sorted(shots, key=lambda p: abs(shot_time(p) - wall))[:CLOSEST_FRAMES]
        nearest.sort(key=shot_time)
        full_size_sheet(nearest, run_dir / "sheet_closest.png", label)
        print(
            f"最も近づいた時刻：+{wall - start:.1f} s（{closest[2][0]} ↔ {closest[2][1]}、"
            f"深さ {closest[1] * 1000:.1f} mm）→ {run_dir / 'sheet_closest.png'}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
