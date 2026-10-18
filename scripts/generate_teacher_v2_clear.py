#!/usr/bin/env python3
"""
障害物から離れる経路の教師データを作る（2026-10-14 のユーザーとの議論、issue #5）。

作り方は scripts/generate_teacher_v2.py（dataset_v2）と同じで、経路の選び方だけ変える：
  経路の損 = 道のり ×（1 + 重み × 近さの損）が最も小さい経路。近さの損は、荷物とハンドの点が実際の障害物から
  4 cm（CLEAR_RANGE）以内に近づくほど大きい（0〜1）。重み 0 なら dataset_v2 と同じ最短のコース。
  重みは環境変数 PAI_CLEAR_WEIGHT で渡す。shoulder_pan が逆向きに 0.02 rad より多く戻る辺は道路網に入れない。
配置の乱数は dataset_v2 と同じ（同じ番号なら同じ配置）なので、お手本の作り方の違いだけを比べられる。

    PAI_CLEAR_WEIGHT=2 python3 scripts/generate_parallel.py --episodes 1000 --workers 6 --version dataset_clear2 \\
        --generator generate_teacher_v2_clear
"""

import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import generate_teacher_v2 as v2  # noqa: E402

v2.CLEAR_WEIGHT = float(os.environ.get("PAI_CLEAR_WEIGHT", "0"))
v2.BACK_EDGE = 0.02
# 分け方の細かさを比べるため（2026-10-14）：格子の間隔 PAI_GRID_STEP [m]、高さの層 PAI_LAYERS（5 または 17）
v2.GRID_STEP = float(os.environ.get("PAI_GRID_STEP", str(v2.GRID_STEP)))
def _pitch(h):
    """高さ h [m] のときの指の向き [度]。高いところほど指を傾ける（真下向きのままでは 8 cm 程度までしか上がらない）。"""
    return -90 if h <= 0.07 else -80 if h <= 0.08 else -70 if h <= 0.10 else -60 if h <= 0.13 else -50 if h <= 0.17 else -45


if os.environ.get("PAI_LAYERS") == "17":       # 荷物の中心の高さ 4〜20 cm を 1 cm おき
    v2.POSES_FREE = [(round(h / 100, 2), _pitch(h / 100)) for h in range(4, 21)]
elif os.environ.get("PAI_LAYERS") == "10":     # 4〜20 cm を等間隔に 10 段（約 1.8 cm おき）
    v2.POSES_FREE = [(round(float(h), 3), _pitch(h)) for h in np.linspace(0.04, 0.20, 10)]

Generator = v2.Generator
layout_key, feature_columns, feature_values = v2.layout_key, v2.feature_columns, v2.feature_values
extra_columns, extra_values, summary_columns, summary_values = (v2.extra_columns, v2.extra_values, v2.summary_columns,
                                                                 v2.summary_values)


def meta():
    return {**v2.meta(), "generator": "scripts/generate_teacher_v2_clear.py", "clear_weight": v2.CLEAR_WEIGHT,
            "clear_range": v2.CLEAR_RANGE, "hand_point_radius": v2.HAND_POINT_RADIUS, "back_edge": v2.BACK_EDGE,
            "grid_step": v2.GRID_STEP, "poses_free": v2.POSES_FREE}


if __name__ == "__main__":
    sys.exit(v2.main())
