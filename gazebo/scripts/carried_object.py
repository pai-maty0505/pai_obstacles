"""
仮の把持物（MoveIt の Attached Collision Object）の定義。

教師データの生成（obstacle_dataset_generator_v5.py）と、デモの preflight
（run_mlp_gazebo_demo.py --carried-object）、余裕の計測（measure_clearance.py --carried-object）で
同じものを使う。Gazebo で物体を固定する意味はなく、MoveIt の衝突判定のためだけに使う。
"""

from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

# 運搬物：プラスチックの直方体。幅 3 cm（指で挟む向き）× 奥行き 3 cm（指に沿った向き）× 高さ 4 cm、36 g。
# 大きさは gripper_frame_link 座標の (x, y, z) の順。x が挟む向き、y が手首の回転軸の向き、z が指に沿った向き。
# （教師データ v5 を作ったときは、3 cm の立方体だった）
CARRIED_OBJECT_ID = "carried_object_v5"
CARRIED_OBJECT_SIZE = (0.03, 0.04, 0.03)
CARRIED_OBJECT_LINK = "gripper_frame_link"

# 物体の中心の位置（gripper_frame_link 座標）。
# 両側から挟むグリッパ（docs/results/phase2/README.md）では、物体は左右のパッドの中心
# （gripper_link 座標で (0, 0, -0.0894)）に来る。gripper_frame_link は gripper_link 座標の
# (-0.0079, -0.0002, -0.0981) にあり、y 軸まわりに 180° 回っているので、そこから見ると次の位置になる。
# （教師データ v5 を作ったときは、gripper_frame_link の原点（(0, 0, 0)）に置いていた）
OBJECT_IN_GRIPPER_LINK = (0.0, 0.0, -0.0894)
CARRIED_OBJECT_OFFSET = (-0.0079, 0.0002, -0.0087)

# 物体を持っている間のグリッパの角度（gripper と gripper_left）。
# パッドが幅 3 cm の物体にちょうど触れるのは 0.163 rad で、少し内側まで閉じて押し付ける
GRASP_GRIPPER_ANGLE = 0.155
GRIPPER_JOINTS = ["gripper", "gripper_left"]

# 指（gripper_link と左右の指）との接触は許可する
CARRIED_OBJECT_TOUCH_LINKS = [
    "gripper_frame_link",
    "gripper_link",
    "moving_jaw_so101_v1_link",
    "left_jaw_link",
]


def use_top_down_grasp(grasp_above):
    """issue #7 の仕様（床から上からつかむ）用に、運搬物の向きと位置を切り替える。

    上から下向きにつかむので、高さ 4 cm の向きを指に沿わせる（gripper_frame_link の z）。
    つかむ点（パッドの中心）を物体の中心より grasp_above 上にずらすので、物体の中心は指先の向きに grasp_above ずれる。
    gripper_frame_link は gripper_link に対して y 軸まわりに 180° 回っているので、指先の向きは gripper_frame_link の +z。
    """
    global CARRIED_OBJECT_SIZE, CARRIED_OBJECT_OFFSET
    CARRIED_OBJECT_SIZE = (0.03, 0.03, 0.04)
    CARRIED_OBJECT_OFFSET = (-0.0079, 0.0002, -0.0087 + grasp_above)


def make_carried_object():
    obj = AttachedCollisionObject()
    obj.link_name = CARRIED_OBJECT_LINK
    obj.touch_links = list(CARRIED_OBJECT_TOUCH_LINKS)
    obj.object.id = CARRIED_OBJECT_ID
    obj.object.header.frame_id = CARRIED_OBJECT_LINK

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(CARRIED_OBJECT_SIZE)

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = CARRIED_OBJECT_OFFSET
    pose.orientation.w = 1.0

    obj.object.primitives = [primitive]
    obj.object.primitive_poses = [pose]
    obj.object.operation = CollisionObject.ADD
    return obj
