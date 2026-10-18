#!/usr/bin/env python3
"""
Gazebo に生成する SO-101 の SDF を作る。

robot_state_publisher と同じ引数で xacro を展開し、`gz sdf -p` で SDF に変換したうえで、
メッシュの衝突形状に `optimization="convex_decomposition"` を付ける。

bullet-featherstone の既定のメッシュ処理（GImpact）は表面の外側に約 10 mm の余裕を持つため、
Gazebo 上のアームが MoveIt の形状より太くなる。凸分解にするとずれは約 2 mm になる
（docs/results/phase1/README.md §5）。

    python3 scripts/make_robot_sdf.py <出力先の SDF> [--use-camera]
"""

import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

XACRO_FILE = (
    Path.home()
    / "so101_ws/install/so101_description/share/so101_description/urdf/robots/so101.urdf.xacro"
)

# 凸分解の細かさ（1つのメッシュを最大いくつの凸形状に分けるか）
MAX_CONVEX_HULLS = 16


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", help="出力先の SDF ファイル")
    parser.add_argument("--use-camera", action="store_true", help="カメラを有効にする")
    args = parser.parse_args()

    urdf = subprocess.run(
        [
            "xacro", str(XACRO_FILE),
            "robot_name:=so101",
            "prefix:=",
            "use_gazebo:=true",
            f"use_camera:={'true' if args.use_camera else 'false'}",
        ],
        capture_output=True,
        text=True,
        check=True,
    ).stdout

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    urdf_path = output.with_suffix(".urdf")
    urdf_path.write_text(urdf)

    sdf_text = subprocess.run(
        ["gz", "sdf", "-p", str(urdf_path)],
        capture_output=True,
        text=True,
        check=True,
    ).stdout

    root = ET.fromstring(sdf_text)
    count = 0
    for collision in root.iter("collision"):
        for mesh in collision.iter("mesh"):
            mesh.set("optimization", "convex_decomposition")
            decomposition = mesh.find("convex_decomposition")
            if decomposition is None:
                decomposition = ET.SubElement(mesh, "convex_decomposition")
            hulls = decomposition.find("max_convex_hulls")
            if hulls is None:
                hulls = ET.SubElement(decomposition, "max_convex_hulls")
            hulls.text = str(MAX_CONVEX_HULLS)
            count += 1

    ET.ElementTree(root).write(output, encoding="unicode", xml_declaration=True)

    check = subprocess.run(["gz", "sdf", "-k", str(output)], capture_output=True, text=True)
    if check.returncode != 0:
        print(check.stdout + check.stderr, file=sys.stderr)
        print(f"SDF の検証に失敗しました：{output}", file=sys.stderr)
        return 1

    print(f"SDF を作成しました：{output}（凸分解にしたメッシュの衝突形状：{count} 個）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
