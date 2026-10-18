"""
SO-101（~/so101_ws の URDF を xacro で展開したもの。MoveIt と同じ）の FK と、腕の面の中の IK を numpy で解く。

MoveIt の /compute_fk を何千回も呼ぶと遅いので、エピソードごとに荷物・皿の距離に合わせて姿勢を解くときはこちらを使う。
解いた姿勢は MoveIt の衝突判定で必ず確かめる（ここでは衝突を見ない）。

  FK.payload_center(q)       持っている荷物の中心（base_link）
  PlanarIK.solve(r, z, pitch) shoulder_pan = 0 で、荷物の中心を回転軸から r・高さ z に、指の向きを pitch にする関節
"""

import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import spec

XACRO = Path.home() / "so101_ws/src/so101_ros2/so101_description/urdf/robots/so101.urdf.xacro"


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class FK:
    def __init__(self, urdf_text=None):
        if urdf_text is None:
            urdf_text = subprocess.run(["xacro", str(XACRO), "robot_name:=so101", "use_gazebo:=true",
                                        "use_camera:=false"], capture_output=True, text=True, check=True).stdout
        root = ET.fromstring(urdf_text)
        child = {j.find("child").get("link"): j for j in root.findall("joint")}
        chain, link = [], "gripper_link"
        while link in child:
            j = child[link]
            chain.append(j)
            link = j.find("parent").get("link")
        self.chain = []
        for j in reversed(chain):
            o = j.find("origin")
            m = np.eye(4)
            if o is not None:
                m[:3, :3] = rpy_matrix(*map(float, (o.get("rpy") or "0 0 0").split()))
                m[:3, 3] = list(map(float, (o.get("xyz") or "0 0 0").split()))
            axis = None
            if j.get("type") in ("revolute", "continuous"):
                axis = np.array(list(map(float, j.find("axis").get("xyz").split())))
                axis = axis / np.linalg.norm(axis)
            self.chain.append((j.get("name"), m, axis))
        px, py, pz = spec.PAD_CENTER_IN_GRIPPER_LINK
        self.payload_offset = np.array([px, py, pz - spec.GRASP_ABOVE, 1.0])

    def gripper(self, q):
        values = dict(zip(spec.ARM_JOINTS, q))
        m = np.eye(4)
        for name, origin, axis in self.chain:
            m = m @ origin
            if axis is not None:
                a, t = axis, values.get(name, 0.0)
                k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                n = np.eye(4)
                n[:3, :3] = np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k
                m = m @ n
        return m

    def payload_center(self, q):
        return (self.gripper(q) @ self.payload_offset)[:3]

    def planar(self, q):
        """(回転軸からの距離, 高さ, 指の向き pitch)。指の向きは gripper_link の -z。"""
        m = self.gripper(q)
        c = (m @ self.payload_offset)[:3]
        f = -m[:3, 2]
        pan = q[0]
        radial = f[0] * math.cos(pan) - f[1] * math.sin(pan)   # 腕の向き（土台から外向き）の成分
        return math.hypot(c[0] - spec.PAN_AXIS_X, c[1]), c[2], math.atan2(f[2], radial)


class PlanarIK:
    """shoulder_pan = 0 の腕の面で、shoulder_lift・elbow_flex・wrist_flex を解く（wrist_roll は WRIST_ROLL）。"""

    def __init__(self, fk, seed):
        self.fk = fk
        self.seed = np.array(seed[1:4], dtype=float)   # 解けている姿勢（lift, elbow, flex）
        self.seed_target = np.array(fk.planar([0.0, *self.seed, spec.WRIST_ROLL]))

    def _q(self, x):
        return [0.0, float(x[0]), float(x[1]), float(x[2]), spec.WRIST_ROLL]

    def _newton(self, target, x):
        for _ in range(50):
            e = np.array(self.fk.planar(self._q(x))) - target
            if np.max(np.abs(e[:2])) < 2e-5 and abs(e[2]) < 1e-4:
                return x
            jac = np.zeros((3, 3))
            for k in range(3):
                d = np.zeros(3)
                d[k] = 1e-5
                jac[:, k] = (np.array(self.fk.planar(self._q(x + d))) - target - e) / 1e-5
            step = np.linalg.lstsq(jac, -e, rcond=None)[0]
            x = x + np.clip(step, -0.1, 0.1)
        return None

    def solve(self, r, z, pitch, steps=12, seed=None):
        """解けている姿勢から目標へ少しずつ動かして解く。可動範囲の外や解けないときは None。"""
        x = self.seed.copy() if seed is None else np.array(seed[1:4], dtype=float)
        start = self.seed_target if seed is None else np.array(self.fk.planar([0.0, *x, spec.WRIST_ROLL]))
        goal = np.array([r, z, pitch])
        for t in np.linspace(0.0, 1.0, steps + 1)[1:]:
            x = self._newton(start + (goal - start) * t, x)
            if x is None:
                return None
        q = self._q(x)
        if all(abs(v) <= lim for v, lim in zip(q, spec.JOINT_LIMITS)):
            return q
        return None


def at_pan(planar_q, pan):
    return [float(pan)] + [float(v) for v in planar_q[1:]]
