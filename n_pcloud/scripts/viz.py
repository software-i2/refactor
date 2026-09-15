#!/usr/bin/env python
# Copyright by BeeX [2026]
#
# Melodic only ships rospy for python2.7, so this must NOT say python3.

"""Visualize the obstacle field and handle candidates."""

from __future__ import print_function

import argparse
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

from n_pcloud import features, field, frame
from n_pcloud import occupancy as occ
from n_pcloud.field_format import read_field

FRAME = "arm_base"


def cloud(points, stamp):
    header = rospy.Header(stamp=stamp, frame_id=FRAME)
    fields = [PointField("x", 0, PointField.FLOAT32, 1),
              PointField("y", 4, PointField.FLOAT32, 1),
              PointField("z", 8, PointField.FLOAT32, 1)]
    return point_cloud2.create_cloud(header, fields,
                                     np.asarray(points, dtype=np.float32).tolist())


def plane_points(head, bit, stride):
    """Cell centers for one blocked link plane."""
    hit = np.argwhere((head["data"] & (1 << bit)) != 0)
    if stride > 1:
        hit = hit[(hit % stride == 0).all(axis=1)]
    return head["lo"] + (hit + 0.5) * head["res"]


def _quat(R):
    """Rotation matrix to quaternion."""
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


def poses(cand, stamp):
    """Arrow markers for candidate grasp approach directions."""
    msg = PoseArray()
    msg.header = rospy.Header(stamp=stamp, frame_id=FRAME)
    for point, axis, approach in cand:
        x = approach / max(np.linalg.norm(approach), 1e-12)
        y = axis - x * float(np.dot(axis, x))
        y = y / max(np.linalg.norm(y), 1e-12)
        R = np.column_stack([x, y, np.cross(x, y)])

        p = Pose()
        p.position.x, p.position.y, p.position.z = point
        (p.orientation.x, p.orientation.y,
         p.orientation.z, p.orientation.w) = _quat(R)
        msg.poses.append(p)
    return msg


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--field", required=True, help="the exported obstacle field")
    ap.add_argument("--scene", help="capture id, to draw its candidates too")
    ap.add_argument("--data", default="data", help="folder holding ply/ and json/")
    ap.add_argument("--stride", type=int, default=2,
                    help="draw every Nth cell; the field is solid and 1 hides the arm")
    ap.add_argument("--plane", default="upper_arm", choices=["upper_arm", "jaw"],
                    help="which link's field to draw. The planes differ almost "
                         "entirely by dilation radius (20 mm vs 6.5 mm), so drawing "
                         "both at once just overlays two blobs.")
    args = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node("n_pcloud_viz")
    head = read_field(args.field)
    at = head["xyz"]
    rospy.loginfo("[viz] %s: %s cells at %.0f mm, camera at %s rpy %s, digest %016x",
                  args.field, tuple(head["dims"]), head["res"] * 1000,
                  np.round(at, 3), np.round(head["rpy"], 2), head["digest"])

    links = field.links_for(head["res"])
    names = [n for n, _r in links]
    radius = dict(links)[args.plane]

    stamp = rospy.Time.now()
    pts = plane_points(head, names.index(args.plane), args.stride)
    rospy.Publisher("viz/field", PointCloud2, queue_size=1, latch=True) \
        .publish(cloud(pts, stamp))
    rospy.loginfo("[viz] viz/field: the %s plane, %d cells dilated %.1f mm -- where that "
                  "link may not go. --plane jaw for the blades'.",
                  args.plane, len(pts), radius * 1000)

    jaw_radius = dict(links)["jaw"]
    pure = plane_points(head, names.index("jaw"), args.stride)
    rospy.Publisher("viz/obstacle", PointCloud2, queue_size=1, latch=True) \
        .publish(cloud(pure, stamp))
    if args.plane == "jaw":
        rospy.loginfo("[viz] viz/obstacle: the same plane again -- the jaw plane is already "
                      "the handle-free one. Pick --plane upper_arm to see the difference.")
    else:
        rospy.loginfo("[viz] viz/obstacle: obstacles only, %d cells dilated %.1f mm. This is "
                      "the jaw plane, which field.py builds from a grid with TARGET removed, so "
                      "it is the one layer with no handle in it. What viz/field has and this "
                      "does not is partly the wider %.1f mm dilation and partly every voxel "
                      "carve_target claimed as handle -- the blades are free to enter those.",
                      len(pure), jaw_radius * 1000, radius * 1000)

    if args.scene:
        ply = os.path.join(args.data, "ply", "%s_depth_scene.ply" % args.scene)
        js = os.path.join(args.data, "json", "%s_depth_poses.json" % args.scene)
        # Placed at the field's own placement, not one given again here.
        f = frame.read(args.scene, ply, js, at, head["rpy"])
        cand = features.candidates(f)

        handle = occ.near_grasps(f.points, f.pos, res=head["res"])
        keep = ~occ.flying_pixels(f.points) | handle
        label, lo = occ.classify(f.points, res=head["res"], keep=keep,
                                 trusted=handle)
        label, carved = occ.carve_target(label, lo, head["res"], f.pos)
        label, _cleared = occ.carve_corridor(label, lo, head["res"], f.pos,
                                             f.rot[:, :, features.APPROACH])
        idx, inside = occ.world_to_index(f.points, lo, head["res"], label.shape)
        is_target = np.zeros(len(f.points), dtype=bool)
        is_target[inside] = label[tuple(idx[inside].T)] == occ.TARGET

        pts = f.to_arm(f.points)
        rest = pts[~is_target][::7]
        rospy.Publisher("viz/cloud", PointCloud2, queue_size=1, latch=True) \
            .publish(cloud(rest, stamp))
        rospy.Publisher("viz/target", PointCloud2, queue_size=1, latch=True) \
            .publish(cloud(pts[is_target], stamp))

        rospy.Publisher("viz/candidates", PoseArray, queue_size=1, latch=True) \
            .publish(poses(cand, stamp))
        rospy.loginfo("[viz] viz/target: %d of %d returns, in %d voxels, claimed as handle "
                      "by %d poses -- drawn whole. viz/cloud: the other %d, every 7th. "
                      "Colour them differently; that split is the classification, and it "
                      "is the one the jaw plane was dilated from.",
                      int(is_target.sum()), len(f.points), carved, len(f.pos), len(rest))
        if carved == 0:
            rospy.logwarn("[viz] nothing carved: the poses land on nothing the camera saw, "
                          "so viz/target is empty and every plane treats the handle as solid.")
        rospy.loginfo("[viz] viz/candidates: %d arrows, one per pose, pointing the way the "
                      "jaws would come in.", len(cand))
        rospy.loginfo("[viz] the one n_task picked comes up on task/chosen, from n_task "
                      "itself -- this node never decides anything.")

    rospy.loginfo("[viz] latched; topics stay up until this node is stopped")
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
