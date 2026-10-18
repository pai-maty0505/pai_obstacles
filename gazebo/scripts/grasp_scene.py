#!/usr/bin/env python3
"""
把持の試験用に、Gazebo に把持台と運搬物を置く（置き直す）。

- 把持台：上面に運搬物を載せる静的な柱。位置は開始姿勢から手首を 90° 回し、
  LIFT_HEIGHT だけ下げたときの物体の中心の真下（data/grasp_poses.json の pick_object）
- 配置台：目標姿勢で物体があるところの真下に置く柱（data/grasp_poses.json の place_object）
- 運搬物：プラスチックの直方体（幅 3 × 奥行き 3 × 高さ 4 cm、36 g）の動的な剛体（質量・慣性・摩擦あり）。
  台の上に立てて置く。接触センサ付き（/pai/object_contacts）

固定ジョイントや追従は使わない。運搬物はただ台の上に置くだけ。

    python3 scripts/grasp_scene.py reset      # 運搬物を把持台の上に置き直す（台がなければ置く）
    python3 scripts/grasp_scene.py remove     # 把持台・配置台・運搬物を片付ける
"""

import json
import math
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parent.parent
POSES_FILE = REPO_DIR / "data" / "grasp_poses.json"

WORLD = "default"
OBJECT_NAME = "carried_object"
OBJECT_SIZE = (0.03, 0.03, 0.04)   # x（奥行き：指に沿った向き）、y（幅：挟む向き）、z（高さ） [m]
OBJECT_MASS = 0.036         # [kg]（プラスチック、比重 1.0 程度）
OBJECT_FRICTION = 1.0
OBJECT_CONTACT_TOPIC = "/pai/object_contacts"

PICK_STAND_NAME = "pick_stand"
PLACE_STAND_NAME = "place_stand"
STAND_SIZE_XY = 0.04        # 台の上面の大きさ [m]（物体より少し大きい）


def object_sdf():
    sx, sy, sz = OBJECT_SIZE
    ixx = OBJECT_MASS * (sy * sy + sz * sz) / 12.0
    iyy = OBJECT_MASS * (sx * sx + sz * sz) / 12.0
    izz = OBJECT_MASS * (sx * sx + sy * sy) / 12.0
    geometry = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{OBJECT_NAME}">
    <link name="link">
      <inertial>
        <mass>{OBJECT_MASS}</mass>
        <inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz></inertia>
      </inertial>
      <collision name="collision">
        {geometry}
        <surface><friction><ode><mu>{OBJECT_FRICTION}</mu><mu2>{OBJECT_FRICTION}</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        {geometry}
        <material><ambient>0.1 0.45 0.9 1</ambient><diffuse>0.1 0.45 0.9 1</diffuse></material>
      </visual>
      <sensor name="contact" type="contact">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <contact><collision>collision</collision><topic>{OBJECT_CONTACT_TOPIC}</topic></contact>
      </sensor>
    </link>
  </model>
</sdf>
"""


def stand_sdf(name, height):
    geometry = f"<geometry><box><size>{STAND_SIZE_XY} {STAND_SIZE_XY} {height}</size></box></geometry>"
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        {geometry}
        <surface><friction><ode><mu>{OBJECT_FRICTION}</mu><mu2>{OBJECT_FRICTION}</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        {geometry}
        <material><ambient>0.55 0.55 0.6 1</ambient><diffuse>0.55 0.55 0.6 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""


def remove(name):
    subprocess.run(
        ["gz", "service", "-s", f"/world/{WORLD}/remove", "--reqtype", "gz.msgs.Entity",
         "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", f'name: "{name}", type: MODEL'],
        capture_output=True, check=False)


def spawn(name, sdf, x, y, z, yaw=0.0):
    """Gazebo の create サービスでモデルを置く。

    ros2 run ros_gz_sim create は、モデルを置いた後も戻らないことがあるので使わない。
    サービスを時間切れ付きで直接呼び、モデルが現れたことを確かめる。
    """
    path = Path(tempfile.gettempdir()) / f"{name}.sdf"
    path.write_text(sdf)
    request = (f'sdf_filename: "{path}", name: "{name}", allow_renaming: false, '
               f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
               f'orientation: {{z: {math.sin(yaw / 2.0)}, w: {math.cos(yaw / 2.0)}}}}}')
    for _ in range(3):
        subprocess.run(
            ["gz", "service", "-s", f"/world/{WORLD}/create", "--reqtype", "gz.msgs.EntityFactory",
             "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", request],
            capture_output=True, check=False, timeout=30)
        for _ in range(20):
            if name in model_names():
                return
            time.sleep(0.25)
    raise RuntimeError(f"Gazebo にモデル {name} を置けませんでした")


def model_names():
    out = subprocess.run(["gz", "model", "--list"], capture_output=True, text=True).stdout
    return [line.strip()[2:] for line in out.splitlines() if line.strip().startswith("- ")]


def main():
    command = sys.argv[1] if len(sys.argv) > 1 else "reset"
    if command == "remove":
        remove(OBJECT_NAME)
        remove(PICK_STAND_NAME)
        remove(PLACE_STAND_NAME)
        print("把持台・配置台・運搬物を片付けました。")
        return 0

    poses = json.loads(POSES_FILE.read_text())
    ox, oy, oz = poses["pick_object"]
    stand_height = oz - OBJECT_SIZE[2] / 2.0

    names = model_names()
    if PICK_STAND_NAME not in names:
        spawn(PICK_STAND_NAME, stand_sdf(PICK_STAND_NAME, stand_height), ox, oy, stand_height / 2.0)
    if PLACE_STAND_NAME not in names and "place_object" in poses:
        px, py, pz = poses["place_object"]
        place_height = pz - OBJECT_SIZE[2] / 2.0
        spawn(PLACE_STAND_NAME, stand_sdf(PLACE_STAND_NAME, place_height), px, py, place_height / 2.0)
    remove(OBJECT_NAME)
    time.sleep(0.3)
    # 台の上面から 2 mm 上に置いて、自然に落として載せる
    spawn(OBJECT_NAME, object_sdf(), ox, oy, oz + 0.002)
    time.sleep(1.0)
    print(f"把持台（高さ {stand_height:.3f} m）の上に運搬物を置きました：中心 ({ox:.3f}, {oy:.3f}, {oz:.3f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
