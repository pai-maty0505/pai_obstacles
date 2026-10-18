#!/usr/bin/env python3
"""
両側から挟むグリッパ（左右対称の 2 本の指）用のメッシュを作る。

1. wrist_roll_follower_body_pai.stl
   元の固定ジョウ（wrist_roll_follower_so101_v1.stl）から指の部分を切り取り、手首の部品だけを残す。
   gripper_link 座標で z > CUT_Z の三角形だけを残す。
2. moving_jaw_mirror_pai.stl
   可動ジョウ（moving_jaw_so101_v1.stl）を左右反転したもの（x → -x）。左の指に使う。
   面の向きが裏返らないように、各三角形の頂点の順番を入れ替える。

出力先は so101_description のメッシュのディレクトリ。作った後に so101_description をビルドし直す。

    python3 scripts/make_gripper_meshes.py
    cd ~/so101_ws && colcon build --symlink-install --packages-select so101_description
"""

import struct
from pathlib import Path

import numpy as np

MESH_DIR = Path.home() / "so101_ws/src/so101_ros2/so101_description/meshes/so_arm101"

# 指の付け根の高さ（gripper_link 座標）。これより下が固定ジョウの指
CUT_Z = -0.042

# wrist_roll_follower の、gripper_link に対する置き方（URDF の visual の origin）
FOLLOWER_Z_OFFSET = 0.000949706  # rpy は x 軸まわり 180°（z → -z）


def load_stl(path):
    data = path.read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    facets = np.frombuffer(
        data[84:84 + count * 50],
        dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]),
    )
    return facets["v"].reshape(-1, 3, 3).astype(np.float64)


def save_stl(path, triangles, header):
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, lengths, out=np.zeros_like(normals), where=lengths > 0)
    record = np.zeros(len(triangles), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    record["n"] = normals
    record["v"] = triangles.reshape(-1, 9)
    with path.open("wb") as f:
        f.write(header.encode("ascii")[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(triangles)))
        f.write(record.tobytes())


def main():
    # 1. 手首の部品だけを残す（三角形の重心が CUT_Z より上）
    follower = load_stl(MESH_DIR / "wrist_roll_follower_so101_v1.stl")
    z_link = -follower[:, :, 2].mean(axis=1) + FOLLOWER_Z_OFFSET
    body = follower[z_link > CUT_Z]
    save_stl(MESH_DIR / "wrist_roll_follower_body_pai.stl", body,
             "pai_obstacle: wrist_roll_follower without fixed finger")
    print(f"手首の部品：{len(follower)} → {len(body)} 個の三角形（指の {len(follower) - len(body)} 個を削除）")

    # 2. 可動ジョウを左右反転する
    jaw = load_stl(MESH_DIR / "moving_jaw_so101_v1.stl")
    mirrored = jaw.copy()
    mirrored[:, :, 0] *= -1.0
    mirrored = mirrored[:, [0, 2, 1], :]  # 頂点の順番を入れ替えて、面の向きを保つ
    save_stl(MESH_DIR / "moving_jaw_mirror_pai.stl", mirrored,
             "pai_obstacle: moving_jaw mirrored in x")
    print(f"左の指：可動ジョウ（{len(jaw)} 個の三角形）を左右反転")
    print(f"出力先：{MESH_DIR}")


if __name__ == "__main__":
    main()
