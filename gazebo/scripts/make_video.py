#!/usr/bin/env python3
"""
scripts/run_episode_recorded.sh --video で撮った画像から、字幕付きの MP4 を作る。

画像は Gazebo のカメラセンサがシミュレーション時間で一定間隔に保存したものなので、
同じ間隔で並べると等倍速（早回しなし）の動画になる。

字幕：
  - 上：題名と「等倍速」
  - 下：条件（left / center / right）と episode、いま何をしているか（戻る／回避）
  - 最後：判定の結果

    python3 scripts/make_video.py <記録先のディレクトリ> [--fps 30]

出力：<記録先>/video.mp4
"""

import argparse
import bisect
import re
import subprocess
import sys
from pathlib import Path

FONT = "Noto Sans CJK JP"
TITLE = "SO-101 × MLP 障害物回避（Gazebo シミュレーション）｜等倍速・早回しなし"
GRASP_TITLE = "SO-101 両側グリッパ：摩擦だけでつかんで運ぶ（Gazebo シミュレーション）｜等倍速・早回しなし"
MISSION_TITLE = "SO-101 × MLP：つかんで、障害物を避けて運び、お皿に置く（Gazebo）｜等倍速・早回しなし"
MISSION_CONDITION = {"left": "左", "center": "中央", "right": "右"}
# issue #7 の仕様（床からつかむ）：障害物の高さはすべて 8 cm、円柱は直径 12 cm
FLOOR_TITLE = "SO-101 × MLP：床の物体をつかみ、障害物を避けて運び、床のお皿に置く（Gazebo）｜等倍速・早回しなし"
FLOOR_SHAPE = {"cylinder": "円柱（直径 12 cm、高さ 8 cm）", "box": "直方体（6 × 6 cm、高さ 8 cm）",
               "pyramid": "三角錐（1 辺 10 cm、高さ 8 cm）"}
MISSION_SHAPE = {"cylinder": "円柱（直径 8 cm、高さ 10 cm）", "box": "直方体（6 × 6 cm、高さ 20 cm）",
                 "pyramid": "三角錐（1 辺 10 cm、高さ 12 cm）"}
CLIP_MARGIN = 1.0   # 条件ごとの動画の前後に付ける余白 [s]

# 把持の試験（grasp_test.py）の各段階の字幕（ログの「〜 を送信」に対応する）
STAGE_CAPTIONS = {
    "グリッパを開く": "グリッパを開く",
    "arm→pre_pick": "物体の手前で構える（手首を 90° 回して横から挟む向き）",
    "arm→pick": "前に進んで、物体を左右の指の間に入れる",
    "グリッパを閉じて挟む": "左右のゴムのパッドで挟む（保持は摩擦だけ）",
    "arm→start_roll90": "持ち上げる",
    "arm→start": "手首を戻して開始姿勢へ（物体は挟んだまま）",
    "arm→goal": "目標姿勢へ運ぶ",
    "arm→place_above": "配置台の上で、手首を 90° 回す",
    "arm→place": "配置台に下ろす",
    "arm→release": "台の少し上まで下ろす",
    "グリッパを開いて離す": "指を開いて置く",
    "arm→post_place": "手前に引く",
}

CLASS_DESCRIPTION = {
    "left": "障害物：扇形の開始側",
    "center": "障害物：扇形の中央",
    "right": "障害物：扇形の目標側",
}

LOG_STAMP = re.compile(r"^\[INFO\] \[(\d+\.\d+)\] \[(run_mlp_gazebo_demo|grasp_test|run_mission)\]: (.*)$")


def load_stats(path):
    rows = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) != 3:
            continue
        try:
            wall, sim, _ = (float(p) for p in parts)
        except ValueError:
            continue
        if sim > 0:
            rows.append((wall, sim))
    return rows


def wall_to_sim(stats, wall):
    walls = [w for w, _ in stats]
    i = bisect.bisect_left(walls, wall)
    if i <= 0:
        return stats[0][1] - (stats[0][0] - wall)
    if i >= len(stats):
        return stats[-1][1] + (wall - stats[-1][0])
    (w0, s0), (w1, s1) = stats[i - 1], stats[i]
    if w1 == w0:
        return s0
    return s0 + (s1 - s0) * (wall - w0) / (w1 - w0)


def parse_episodes(run_log):
    """run.log から、エピソードごとの条件・番号と、戻る／回避の開始・終了（壁時計）を読む。"""
    episodes = []
    current = None
    for line in run_log.read_text(encoding="utf-8").splitlines():
        if line.startswith("episode: "):
            current = {"episode": line.split(":", 1)[1].strip()}
            episodes.append(current)
        elif line.startswith("クラス: ") and current is not None:
            current["class"] = line.split(":", 1)[1].strip()
        m = LOG_STAMP.match(line)
        if not m or current is None:
            continue
        stamp, message = float(m.group(1)), m.group(3)
        if message.startswith(("return-to-start を送信", "reset-to-start を送信")):
            current["return_start"] = stamp
        elif message.startswith(("return-to-start:", "reset-to-start:")):
            current["return_end"] = stamp
        elif message.startswith("MLP-predicted-trajectory を送信"):
            current["mlp_start"] = stamp
        elif message.startswith("MLP-predicted-trajectory:"):
            current["mlp_end"] = stamp
    return [e for e in episodes if "mlp_start" in e and "mlp_end" in e]


def parse_mission(run_log):
    """ミッション（run_mission.py）の run.log から、条件ごとのエピソードの開始・終了（壁時計）を読む。"""
    episodes = []
    for line in run_log.read_text(encoding="utf-8").splitlines():
        m = LOG_STAMP.match(line)
        if not m or m.group(2) != "run_mission":
            continue
        stamp, message = float(m.group(1)), m.group(3)
        if message.startswith("エピソード開始: "):
            condition, shape, _, episode = message.split(": ", 1)[1].split()
            episodes.append({"condition": condition, "shape": shape, "episode": episode, "start": stamp})
        elif message.startswith("エピソード終了: ") and episodes:
            episodes[-1]["end"] = stamp
    return [e for e in episodes if "end" in e]


def parse_stages(run_log):
    """把持の試験・ミッションの run.log から、各段階の開始（壁時計）と字幕を読む。"""
    stages = []
    for line in run_log.read_text(encoding="utf-8").splitlines():
        m = LOG_STAMP.match(line)
        if not m or m.group(2) not in ("grasp_test", "run_mission") or " を送信" not in m.group(3):
            continue
        label = m.group(3).split(" を送信")[0]
        stages.append((float(m.group(1)), STAGE_CAPTIONS.get(label, label)))
    return stages


def drawtext(textfile, x, y, size, start=None, end=None, box=True):
    options = [
        f"font='{FONT}'",
        f"textfile='{textfile}'",
        "fontcolor=white",
        f"fontsize={size}",
        f"x={x}",
        f"y={y}",
    ]
    if box:
        options += ["box=1", "boxcolor=black@0.55", "boxborderw=10"]
    if start is not None:
        options.append(f"enable='between(t,{start:.2f},{end:.2f})'")
    return "drawtext=" + ":".join(options)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    run_dir = Path(args.run_dir).resolve()
    frames_dir = run_dir / "frames"
    frames = sorted(frames_dir.glob("*.png"), key=lambda p: int(re.search(r"_(\d+)\.png$", p.name).group(1)))
    if not frames:
        print("画像がありません。", file=sys.stderr)
        return 1
    duration = len(frames) / args.fps

    stats = load_stats(run_dir / "stats.txt")
    video_start_sim = float((run_dir / "video_start_sim.txt").read_text().strip())
    episodes = parse_episodes(run_dir / "run.log")

    captions_dir = run_dir / "captions"
    captions_dir.mkdir(exist_ok=True)

    def caption(name, text):
        path = captions_dir / f"{name}.txt"
        path.write_text(text, encoding="utf-8")
        return path

    def video_time(wall):
        return wall_to_sim(stats, wall) - video_start_sim

    stages = parse_stages(run_dir / "run.log")
    mission = parse_mission(run_dir / "run.log")
    floor = "床からつかむ" in (run_dir / "run.log").read_text(encoding="utf-8")
    shape_caption = FLOOR_SHAPE if floor else MISSION_SHAPE
    found = re.search(r"障害物の高さ (\d+) cm", (run_dir / "run.log").read_text(encoding="utf-8"))
    if floor and found:
        shape_caption = {k: re.sub(r"高さ \d+ cm", f"高さ {found.group(1)} cm", v) for k, v in FLOOR_SHAPE.items()}
    title = (FLOOR_TITLE if floor else MISSION_TITLE) if mission else (GRASP_TITLE if stages and not episodes else TITLE)
    filters = [drawtext(caption("title", title), 20, 20, 26)]

    # 把持の試験：段階ごとの字幕（次の段階が始まるまで出す）
    for i, (stamp, text) in enumerate(stages):
        start = video_time(stamp)
        end = video_time(stages[i + 1][0]) if i + 1 < len(stages) else duration
        filters.append(drawtext(caption(f"stage{i}", text), 20, "h-60", 28, start, end))

    # ミッション：条件の字幕（エピソードの間だけ）
    for i, e in enumerate(mission, start=1):
        label = (f"条件 {i}/{len(mission)}：{MISSION_CONDITION.get(e['condition'], e['condition'])}　"
                 f"障害物：{shape_caption.get(e['shape'], e['shape'])}")
        filters.append(drawtext(caption(f"mission{i}", label), 20, "h-110", 30,
                                video_time(e["start"]) - CLIP_MARGIN, video_time(e["end"]) + CLIP_MARGIN))

    for i, e in enumerate(episodes, start=1):
        label = f"条件 {i}/{len(episodes)}：{e.get('class', '?')}（{CLASS_DESCRIPTION.get(e.get('class'), '')}）  episode {e['episode']}"
        # 条件の字幕は、開始姿勢に戻って障害物を入れ替えた時点から出す
        # （戻っている間は、まだ前の条件の障害物が表示されているため）
        start = video_time(e.get("return_end", e["mlp_start"]))
        end = video_time(e["mlp_end"]) + 1.0
        filters.append(drawtext(caption(f"episode{i}", label), 20, "h-110", 30, start, end))
        if "return_start" in e and "return_end" in e:
            filters.append(drawtext(
                caption(f"return{i}", "前の軌道を逆にたどって開始姿勢へ戻る（障害物は表示したまま）"),
                20, "h-60", 26, video_time(e["return_start"]), video_time(e["return_end"]) + 0.5))
        mlp_seconds = wall_to_sim(stats, e["mlp_end"]) - wall_to_sim(stats, e["mlp_start"])
        filters.append(drawtext(
            caption(f"mlp{i}", f"MLP が推論した関節軌道で障害物を回避（オンライン再計画なし、{mlp_seconds:.0f} 秒）"),
            20, "h-60", 26, video_time(e["mlp_start"]), video_time(e["mlp_end"]) + 0.5))

    result_file = run_dir / "summary.txt"
    if result_file.exists():
        passed = "接触の判定：合格" in result_file.read_text(encoding="utf-8")
        if mission:
            verdict = "接触判定：合格（障害物への接触なし。物体に触れたのはパッドと台・お皿だけ）" if passed else ""
        elif stages and not episodes:
            verdict = "接触判定：合格（物体に触れたのは左右のパッドと台だけ）" if passed else ""
        else:
            verdict = "接触判定：合格（障害物への接触なし）" if passed else ""
        if verdict:
            filters.append(drawtext(caption("verdict", verdict), "w-tw-20", 70, 26, max(0.0, duration - 4.0), duration))

    output = run_dir / "video.mp4"
    command = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-framerate", str(args.fps),
        "-start_number", "0",
        "-i", str(frames_dir / "video_camera::link::camera_%d.png"),
        "-vf", ",".join(filters),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", "-movflags", "+faststart",
        str(output),
    ]
    subprocess.run(command, check=True)
    print(f"動画を作成しました：{output}（{duration:.1f} 秒、{len(frames)} 枚、{args.fps} fps、等倍速）")
    if mission:
        # 条件ごとに切り出し（前後に CLIP_MARGIN 秒の余白）、つなげた動画も作る
        clips = []
        total = 0.0
        for i, e in enumerate(mission, start=1):
            start = max(0.0, video_time(e["start"]) - CLIP_MARGIN)
            end = min(duration, video_time(e["end"]) + CLIP_MARGIN)
            clip = run_dir / f"mission_{i}_{e['condition']}_{e['shape']}.mp4"
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", f"{start:.3f}", "-to", f"{end:.3f}",
                            "-i", str(output), "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", "-movflags", "+faststart", str(clip)],
                           check=True)
            clips.append(clip)
            seconds = wall_to_sim(stats, e["end"]) - wall_to_sim(stats, e["start"])
            total += seconds
            print(f"  条件 {i}：{e['condition']}（{e['shape']}）エピソード {seconds:.1f} 秒"
                  f"（{'60 秒未満' if seconds < 60 else '60 秒以上'}）、切り出し {end - start:.1f} 秒 → {clip.name}")
        listing = run_dir / "mission_clips.txt"
        listing.write_text("".join(f"file '{c}'\n" for c in clips), encoding="utf-8")
        joined = run_dir / "mission_all.mp4"
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(listing),
                        "-c", "copy", "-movflags", "+faststart", str(joined)], check=True)
        joined_seconds = sum(min(duration, video_time(e["end"]) + CLIP_MARGIN)
                             - max(0.0, video_time(e["start"]) - CLIP_MARGIN) for e in mission)
        print(f"  エピソードの合計 {total:.1f} 秒（{'180 秒未満' if total < 180 else '180 秒以上'}）、"
              f"つなげた動画 {joined_seconds:.1f} 秒 → {joined.name}")
        return 0
    total = 0.0
    for i, e in enumerate(episodes, start=1):
        start = video_time(e.get("return_start", e["mlp_start"]))
        end = video_time(e["mlp_end"])
        total += end - start
        print(
            f"  条件 {i}：{e.get('class')} episode {e['episode']}  動画の {start:.1f}〜{end:.1f} 秒"
            f"（エピソードの長さ {end - start:.1f} 秒：{'60 秒未満' if end - start < 60 else '60 秒以上'}）"
        )
    print(f"  エピソードの合計 {total:.1f} 秒（{'180 秒未満' if total < 180 else '180 秒以上'}）、動画の長さ {duration:.1f} 秒"
          f"（{'1〜3 分に収まる' if 60 <= duration <= 180 else '1〜3 分の範囲外'}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
