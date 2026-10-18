#!/usr/bin/env python3
"""
最終デモ（issue #5）の障害物 3 種類を、Gazebo と MoveIt に同じ値から置く。

  円柱（cylinder）：直径 8 cm、高さ 10 cm
  直方体（box）    ：6 × 6 cm、高さ 20 cm
  三角錐（pyramid）：底面は 1 辺 10 cm の正三角形、高さ 12 cm（頂点は底面の重心の真上）

障害物は床に置く。位置は、運ぶ道（土台の回転軸のまわりの円弧）の上の shoulder_pan の角度で表す。

MoveIt の障害物は、余裕 C（既定 10 mm）だけ外側に広げた形にする。
  - 円柱・直方体：各面を C だけ外へ（円柱は半径と高さを C ずつ）
  - 三角錐：各面を平行に広げるだけでは、とがった頂点の上で余裕が 4 cm 以上になるので、
    各頂点に半径 C の球（の外接多面体）を置き、それらを包む凸包にする（どの点でも余裕は C 以上）

三角錐は Gazebo ではメッシュ（sim/models/pyramid_10x12.stl）を使い、凸分解で衝突形状にする
（GImpact のままだと表面の外側に約 1 cm の余裕が付くため）。MoveIt には同じ頂点からメッシュを作る。

    python3 scripts/obstacle_shapes.py stl                          # 三角錐の STL を作る
    python3 scripts/obstacle_shapes.py show cylinder:0.45 box:0.8 pyramid:1.15   # Gazebo と MoveIt に置く
    python3 scripts/obstacle_shapes.py remove                       # 片付ける（Gazebo と MoveIt）
"""

import math
import sys
from pathlib import Path

import numpy as np

REPO_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_DIR / "scripts"))

PYRAMID_STL = REPO_DIR / "sim" / "models" / "pyramid_10x12.stl"
CONTACT_TOPIC = "/pai/obstacle_contacts"
CLEARANCE = 0.010

# 運ぶ道：土台の回転軸（shoulder_pan）は base_link の (PAN_AXIS_X, 0)。物体の中心はそこから ARC_RADIUS の円弧の上
PAN_AXIS_X = 0.0388
ARC_RADIUS = 0.2612

SHAPES = {
    "cylinder": {"label": "円柱", "radius": 0.04, "height": 0.10},
    "box": {"label": "直方体", "size": (0.06, 0.06, 0.20)},
    "pyramid": {"label": "三角錐", "side": 0.10, "height": 0.12},
}
COLORS = {"cylinder": "0.85 0.35 0.2 1", "box": "0.25 0.6 0.3 1", "pyramid": "0.6 0.3 0.75 1"}


def use_floor_shapes(r_base, height=0.08):
    """床からつかむ仕様の障害物に切り替える：高さはすべて height、円柱は直径 12 cm。
    issue #7 の仕様（PLAN.md §3.6）は height = 0.09、issue #8 の仕様（§3.6.1）は 0.08。
    置く円弧は、土台から r_base（shoulder_pan = 0 のときの x）。"""
    global ARC_RADIUS, PYRAMID_STL
    SHAPES["cylinder"].update(radius=0.06, height=height)
    SHAPES["box"]["size"] = (0.06, 0.06, height)
    SHAPES["pyramid"]["height"] = height
    ARC_RADIUS = r_base - PAN_AXIS_X
    PYRAMID_STL = REPO_DIR / "sim" / "models" / f"pyramid_10x{round(height * 100)}.stl"
    if not PYRAMID_STL.exists():
        write_stl(PYRAMID_STL, pyramid_vertices(), np.array(PYRAMID_FACES))


def height_from_meta(data_dir):
    """教師データの記録（data_dir/teacher_*_meta.json）にある障害物の高さ。見つからなければ None。"""
    import json
    for meta in sorted(Path(data_dir).glob("teacher_*_meta.json")):
        shapes = json.loads(meta.read_text()).get("shapes", {})
        if "cylinder" in shapes:
            return float(shapes["cylinder"]["height"])
    return None


def on_arc(pan):
    """shoulder_pan の角度 pan での、運ぶ道の上の点（床の上の x, y）。"""
    return PAN_AXIS_X + ARC_RADIUS * math.cos(pan), -ARC_RADIUS * math.sin(pan)


def pyramid_vertices():
    """三角錐の頂点（底面の重心が原点、底面は z = 0、頂点の 1 つは +x 向き）。"""
    side, height = SHAPES["pyramid"]["side"], SHAPES["pyramid"]["height"]
    r = side / math.sqrt(3.0)
    base = [(r * math.cos(a), r * math.sin(a), 0.0) for a in (0.0, 2 * math.pi / 3, 4 * math.pi / 3)]
    return np.array(base + [(0.0, 0.0, height)])


PYRAMID_FACES = [(0, 2, 1), (0, 1, 3), (1, 2, 3), (2, 0, 3)]   # 外向きの法線になる順


def sphere_directions():
    """球の外接多面体の頂点方向（正二十面体を 1 回分割した 42 方向）と、内接球の半径の比。"""
    t = (1 + math.sqrt(5)) / 2
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
         (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    v = [np.array(p) / np.linalg.norm(p) for p in v]
    points = list(v)
    for i in range(12):
        for j in range(i + 1, 12):
            if np.linalg.norm(v[i] - v[j]) < 1.1:   # 隣り合う頂点（辺の長さ 1.05）
                m = v[i] + v[j]
                points.append(m / np.linalg.norm(m))
    points = np.array(points)
    from scipy.spatial import ConvexHull
    hull = ConvexHull(points)
    inner = min(-eq[3] for eq in hull.equations)   # 原点から各面までの距離の最小（内接球の半径）
    return points, inner


def inflated_pyramid(clearance):
    """余裕 clearance を含む三角錐の凸包（頂点と三角形）。"""
    from scipy.spatial import ConvexHull
    base = pyramid_vertices()
    if clearance <= 0.0:
        return base, np.array(PYRAMID_FACES)
    directions, inner = sphere_directions()
    # 多面体は球より小さいので、内接球が半径 clearance になるように大きくする
    points = np.array([v + d * clearance / inner for v in base for d in directions])
    hull = ConvexHull(points)
    vertices = points[hull.vertices]
    index = {old: new for new, old in enumerate(hull.vertices)}
    faces = []
    for simplex, eq in zip(hull.simplices, hull.equations):
        a, b, c = (points[i] for i in simplex)
        if np.dot(np.cross(b - a, c - a), eq[:3]) < 0:   # 外向きの順にそろえる
            simplex = simplex[[0, 2, 1]]
        faces.append([index[i] for i in simplex])
    return vertices, np.array(faces)


def write_stl(path, vertices, faces):
    lines = ["solid pyramid"]
    for f in faces:
        a, b, c = (vertices[i] for i in f)
        n = np.cross(b - a, c - a)
        n = n / np.linalg.norm(n)
        lines.append(f"  facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}")
        lines.append("    outer loop")
        for p in (a, b, c):
            lines.append(f"      vertex {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append("endsolid pyramid")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")


def geometry_sdf(shape):
    """Gazebo の形（モデルの原点は床の上の障害物の中心）。(ジオメトリ, 衝突形状の pose の z)"""
    s = SHAPES[shape]
    if shape == "cylinder":
        return f"<cylinder><radius>{s['radius']}</radius><length>{s['height']}</length></cylinder>", s["height"] / 2
    if shape == "box":
        x, y, z = s["size"]
        return f"<box><size>{x} {y} {z}</size></box>", z / 2
    mesh = (f'<mesh optimization="convex_decomposition"><uri>file://{PYRAMID_STL}</uri>'
            f"<convex_decomposition><max_convex_hulls>1</max_convex_hulls></convex_decomposition></mesh>")
    return mesh, 0.0


def model_sdf(shape, name):
    geometry, z = geometry_sdf(shape)
    visual_geometry = geometry.replace(' optimization="convex_decomposition"', "").replace(
        "<convex_decomposition><max_convex_hulls>1</max_convex_hulls></convex_decomposition>", "")
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <pose>0 0 {z} 0 0 0</pose>
        <geometry>{geometry}</geometry>
      </collision>
      <visual name="visual">
        <pose>0 0 {z} 0 0 0</pose>
        <geometry>{visual_geometry}</geometry>
        <material><ambient>{COLORS[shape]}</ambient><diffuse>{COLORS[shape]}</diffuse></material>
      </visual>
      <sensor name="contact" type="contact">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <contact><collision>collision</collision><topic>{CONTACT_TOPIC}</topic></contact>
      </sensor>
    </link>
  </model>
</sdf>
"""


def obstacle_yaw(shape, pan):
    """障害物の向き：三角錐は頂点の 1 つを土台から外向きにする（土台の側には辺が向く）。直方体は運ぶ道に沿わせる。"""
    return -pan


def moveit_object(object_id, shape, pan, clearance=CLEARANCE, add=True):
    from geometry_msgs.msg import Point, Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive

    obj = CollisionObject()
    obj.id = object_id
    obj.header.frame_id = "base_link"
    if not add:
        obj.operation = CollisionObject.REMOVE
        return obj
    x, y = on_arc(pan)
    yaw = obstacle_yaw(shape, pan)
    s = SHAPES[shape]
    pose = Pose()
    pose.position.x, pose.position.y = x, y
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    if shape == "cylinder":
        h = s["height"] + 2 * clearance
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[h, s["radius"] + clearance])]
        pose.position.z = s["height"] / 2
        obj.primitive_poses = [pose]
    elif shape == "box":
        sx, sy, sz = s["size"]
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                         dimensions=[sx + 2 * clearance, sy + 2 * clearance, sz + 2 * clearance])]
        pose.position.z = sz / 2
        obj.primitive_poses = [pose]
    else:
        vertices, faces = inflated_pyramid(clearance)
        mesh = Mesh()
        mesh.vertices = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in vertices]
        mesh.triangles = [MeshTriangle(vertex_indices=[int(i) for i in f]) for f in faces]
        obj.meshes = [mesh]
        obj.mesh_poses = [pose]
    obj.operation = CollisionObject.ADD
    return obj


def spawn_gazebo(name, shape, pan):
    import grasp_scene
    x, y = on_arc(pan)
    grasp_scene.remove(name)
    grasp_scene.spawn(name, model_sdf(shape, name), x, y, 0.0, obstacle_yaw(shape, pan))


def main():
    command = sys.argv[1] if len(sys.argv) > 1 else "stl"
    if command == "stl":
        vertices = pyramid_vertices()
        write_stl(PYRAMID_STL, vertices, np.array(PYRAMID_FACES))
        print(f"三角錐の STL を書きました：{PYRAMID_STL}")
        v, f = inflated_pyramid(CLEARANCE)
        print(f"余裕 {CLEARANCE * 1000:.0f} mm を含む三角錐（MoveIt 用）：頂点 {len(v)}、三角形 {len(f)}、"
              f"高さ {v[:, 2].max():.4f} m、底面の外接円の半径 {np.hypot(v[:, 0], v[:, 1]).max():.4f} m")
        return 0

    import rclpy
    from mission_scene import apply
    from run_mlp_gazebo_demo import MLPDemoNode

    rclpy.init()
    node = MLPDemoNode("obstacle_shapes")
    import grasp_scene
    if command == "remove":
        names = [n for n in grasp_scene.model_names() if n.startswith("obstacle_")]
        for n in names:
            grasp_scene.remove(n)
        apply(node, [moveit_object(f"obstacle_{k}", "box", 0.0, add=False) for k in range(3)])
        print("障害物を片付けました（Gazebo と MoveIt）。")
    elif command == "show":
        objects = []
        for k, spec in enumerate(sys.argv[2:]):
            shape, pan = spec.split(":")
            spawn_gazebo(f"obstacle_{k}", shape, float(pan))
            objects.append(moveit_object(f"obstacle_{k}", shape, float(pan)))
            x, y = on_arc(float(pan))
            print(f"  obstacle_{k}：{SHAPES[shape]['label']}、shoulder_pan {float(pan):.2f} rad、床の上の中心 ({x:.3f}, {y:.3f})")
        apply(node, objects)
        print("Gazebo（実際の形）と MoveIt（余裕を含む形）に置きました。")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
