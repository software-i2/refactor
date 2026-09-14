#!/usr/bin/env python
# Copyright by BeeX [2026]

"""The newest frame on the same topics n_pcloud viz.py publishes."""

from __future__ import print_function

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from n_pcloud import field
from n_pcloud import occupancy as occ
from n_pcloud.field_format import read_field

FRAME = "arm_base"


def cells(head, bit, stride):
    hit = np.argwhere((head["data"] & (1 << bit)) != 0)
    if stride > 1:
        hit = hit[(hit % stride == 0).all(axis=1)]
    return head["lo"] + (hit + 0.5) * head["res"]


def cloud(header, pts):
    pts = np.ascontiguousarray(pts, dtype="<f4")
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(pts)
    msg.fields = [PointField("x", 0, PointField.FLOAT32, 1),
                  PointField("y", 4, PointField.FLOAT32, 1),
                  PointField("z", 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(pts)
    msg.data = pts.tobytes()
    msg.is_dense = True
    return msg


def quat(R):
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2.0
    q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    return tuple(q)


def arrows(header, cand):
    msg = PoseArray()
    msg.header = header
    for point, axis, approach in cand:
        x = approach / max(np.linalg.norm(approach), 1e-12)
        y = axis - x * float(np.dot(axis, x))
        y = y / max(np.linalg.norm(y), 1e-12)
        p = Pose()
        p.position.x, p.position.y, p.position.z = [float(v) for v in point]
        (p.orientation.x, p.orientation.y,
         p.orientation.z, p.orientation.w) = [float(v) for v in quat(np.column_stack([x, y, np.cross(x, y)]))]
        msg.poses.append(p)
    return msg


class Board(object):
    def __init__(self, stride=2, every=7):
        self.stride = stride
        self.every = every
        self.pub = dict((name, rospy.Publisher("viz/" + name, kind, queue_size=1, latch=True))
                        for name, kind in (("field", PointCloud2), ("obstacle", PointCloud2),
                                           ("cloud", PointCloud2), ("target", PointCloud2),
                                           ("scene", PointCloud2),
                                           ("candidates", PoseArray)))

    def show(self, field_path, r):
        head = read_field(field_path)
        names = [n for n, _r in field.links_for(head["res"])]
        header = Header(stamp=rospy.Time.now(), frame_id=FRAME)
        f = r["frame"]
        self.pub["field"].publish(cloud(header, cells(head, names.index("upper_arm"), self.stride)))
        self.pub["obstacle"].publish(cloud(header, cells(head, names.index("jaw"), self.stride)))
        idx, inside = occ.world_to_index(f.points, r["lo"], head["res"], r["label"].shape)
        is_target = np.zeros(len(f.points), dtype=bool)
        is_target[inside] = r["label"][tuple(idx[inside].T)] == occ.TARGET
        self.pub["cloud"].publish(cloud(header, f.to_arm(f.points[~is_target][::self.every])))
        self.pub["target"].publish(cloud(header, f.to_arm(f.points[is_target])))
        seen = np.argwhere(np.isin(r["label"], (occ.OBSTACLE, occ.TARGET)))
        self.pub["scene"].publish(cloud(header, f.to_arm(r["lo"] + (seen + 0.5) * head["res"])))
        self.pub["candidates"].publish(arrows(header, r["cand"][r["keep"]]))
