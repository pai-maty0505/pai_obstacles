"""
v3 の仕様（README の 1 章と、決めた推奨値）を 1 か所にまとめる。ROS 2 を使わない。

座標は base_link。shoulder_pan の回転軸は (PAN_AXIS_X, 0)。
物体・障害物・お皿は、回転軸から ARC の円弧の上に置き、位置を shoulder_pan の角度で表す
（shoulder_pan が正のとき、円弧の上の点は y が負の側。v2 と同じ向き）。
"""

import math

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER_JOINTS = ["gripper", "gripper_left"]
JOINT_LIMITS = [1.91986, 1.74533, 1.69, 1.65806, 2.74385]

PAN_AXIS_X = 0.0388
ARC = 0.20                       # 回転軸から物体・障害物・お皿までの距離 [m]（README 1.3）

# 手首（wrist_roll）はエピソードの中で回さない。v2（issue #7・#8）の 1.6195 rad から 90° 戻し、上からつかむとき
# 左右の指が半径の向き（土台から外向き）に並ぶようにする。指が円周の向きに並ぶと、円柱の上を越えるときに
# 隣の直方体（高さ 20 cm）に指が当たるため。指を傾けて運ぶときに運搬物が指の中で回りにくい向きでもある（README 2.4）
WRIST_ROLL = 1.6195 - 1.5708

# 運搬物（README 1.2）。大きさは (挟む向き, 指に沿った向きと直角の水平, 高さ)
PAYLOAD_SIZE = (0.02, 0.02, 0.04)
PAYLOAD_MASS = 0.036
# つかむ点（左右のパッドの中心）を、運搬物の中心より GRASP_ABOVE だけ上にする（v2 と同じ。パッドの下端を床から離すため）
GRASP_ABOVE = 0.006
PAD_CENTER_IN_GRIPPER_LINK = (0.0, 0.0, -0.0894)   # v2 の両側から挟むグリッパ（gripper_link 座標）

# お皿（README 1.2）。直径は外径として扱う（README 4 章：内径か外径かは未決定）
PLATE_DIAMETER = 0.04
PLATE_DEPTH = 0.01
PLATE_RIM = 0.003
PLATE_FLOOR = 0.003               # 底板の厚さ（運搬物が載る面の高さ）
PLATE_RIM_SEGMENTS = 16
RELEASE = 0.006                   # 置くときの、お皿の底から運搬物の下端までの浮き（v2 と同じ）

# 障害物（README 1.2）
SHAPES = {
    "cylinder": {"label": "円柱", "radius": 0.03, "height": 0.08},
    "box": {"label": "直方体", "size": (0.04, 0.04, 0.20)},
    "pyramid": {"label": "三角錐", "side": 0.06, "height": 0.12},
}
SHAPE_NAMES = list(SHAPES)
# 置く場所（README 2.1 の推奨値 0.35 rad 間隔を採用）。3 種類を 1 個ずつランダムに割り当てる
SLOTS = {"left": 0.45, "center": 0.80, "right": 1.15}
SLOT_NAMES = list(SLOTS)
# 向きの対称性（この角度で同じ形になる）
YAW_PERIOD = {"cylinder": 2 * math.pi, "box": math.pi / 2, "pyramid": 2 * math.pi / 3}

# 運搬物とお皿の置き場所（README 2.2 の推奨値：最も近い障害物から 0.45 rad 以上。パンの可動域 ±1.92 rad に余裕を残す）
NEAR_RANGE = (-0.25, 0.00)        # 障害物の左側（shoulder_pan の小さい側）
FAR_RANGE = (1.60, 1.85)          # 障害物の右側

# 衝突判定の余裕：教師データ 15 mm、本番（Gazebo で実行する前の判定）8 mm（README 1.10。2026-10-14 に 10 mm から変更。
# 実機では行わないため、Gazebo で起こりうるずれの合計から決めた仮の値）
TEACHER_CLEARANCE = 0.015
RUN_CLEARANCE = 0.008


def on_arc(pan, radius=ARC):
    """shoulder_pan の角度 pan での、円弧の上の点（床の上の x, y）。"""
    return PAN_AXIS_X + radius * math.cos(pan), -radius * math.sin(pan)


def pan_of(x, y):
    """床の上の点 (x, y) の shoulder_pan の角度と、回転軸からの距離。"""
    return math.atan2(-y, x - PAN_AXIS_X), math.hypot(x - PAN_AXIS_X, y)


# ---------------------------------------------------------------- dataset_v2 の配置（2026-10-14 のユーザーの指示）
# 物は円弧の上ではなく、回転軸から R_MAX 以内の平面の上にばらばらに置く。隣り合う物の間には、ハンドが動ける間隔を空ける。
# 荷物から皿へ、障害物の間を縫う線に沿って運ぶ（ユーザーの図：三角錐を越え、直方体の手前を回り、円柱の上を越えて皿へ）。
R_MAX = 0.20                      # 物の中心の、回転軸からの距離の上限 [m]
R_HANDLE = (0.12, 0.20)           # 荷物・皿の距離の範囲（真上からつかむ・置く姿勢は 0.11〜0.21 m で解ける） [m]
ANGLE_RANGE = (-1.50, 1.50)       # 物を置く shoulder_pan の範囲（土台の前の半円） [rad]
BASE_RADIUS = 0.074               # 土台の、回転軸からの最大距離（base_link のメッシュ） [m]
GAP = 0.05                        # 隣り合う物の縁と縁の最小距離（外接円で測る） [m]
# 障害物どうしの間は、荷物を持ったハンドが間を抜けられる幅にする：ハンドの円周の向きの幅 約 5〜6 cm
# ＋ 衝突判定の余裕 1.5 cm × 2（5 cm では、隣り合う障害物の間で経路が見つからないことが多かった）
GAP_OBSTACLES = 0.08
# 外接円の半径（間隔の判定用） [m]
FOOTPRINT = {"cylinder": 0.03, "box": 0.02 * math.sqrt(2), "pyramid": 0.06 / math.sqrt(3),
             "payload": math.hypot(PAYLOAD_SIZE[0], PAYLOAD_SIZE[1]) / 2, "plate": PLATE_DIAMETER / 2}
# 形ごとの、中心の距離の下限 [m]。直方体（高さ 20 cm）は外側を回れない（腕が上をまたいで当たる）ので、
# 土台との間をハンドが通れるように、内側を通れる距離から置く（MoveIt で測った：17 cm でぎりぎり、18 cm で余裕あり）
R_MIN_BY_SHAPE = {"box": 0.175}
