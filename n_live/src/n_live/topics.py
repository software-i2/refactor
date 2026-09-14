#!/usr/bin/env python
# Copyright by BeeX [2026]

"""Capture pairs from ROS topics, written into the inbox the builder watches."""

from __future__ import print_function

import json
import os
import threading

import numpy as np

from n_live import inbox

_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def stamp_of(msg):
    return msg.header.stamp.secs, msg.header.stamp.nsecs


def points(cloud):
    by = dict((f.name, f) for f in cloud.fields)
    missing = [n for n in ("x", "y", "z") if n not in by]
    if missing:
        raise ValueError("the cloud has no %s field" % ", ".join(missing))
    order = ">" if cloud.is_bigendian else "<"
    dt = np.dtype({"names": ["x", "y", "z"],
                   "formats": [order + _DTYPES[by[n].datatype] for n in ("x", "y", "z")],
                   "offsets": [by[n].offset for n in ("x", "y", "z")],
                   "itemsize": cloud.point_step})
    arr = np.frombuffer(cloud.data, dtype=dt, count=cloud.width * cloud.height)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1)
    return xyz[np.all(np.isfinite(xyz), axis=1) & np.any(xyz != 0, axis=1)]


def write_ply(path, xyz):
    xyz = np.ascontiguousarray(xyz, dtype="<f4")
    head = ("ply\nformat binary_little_endian 1.0\nelement vertex %d\n"
            "property float x\nproperty float y\nproperty float z\nend_header\n" % len(xyz))
    with open(path, "wb") as fh:
        fh.write(head.encode("ascii"))
        fh.write(xyz.tobytes())


def poses_doc(msg, name):
    return {"frame": name, "units": "meters",
            "poses": [{"id": i,
                       "position_m": [p.position.x, p.position.y, p.position.z],
                       "quaternion_xyzw": [p.orientation.x, p.orientation.y,
                                           p.orientation.z, p.orientation.w]}
                      for i, p in enumerate(msg.poses)]}


def put(folder, fid, cloud, poses):
    xyz = points(cloud)
    part = os.path.join(folder, "." + fid + inbox.PLY + ".part")
    write_ply(part, xyz)
    os.rename(part, os.path.join(folder, fid + inbox.PLY))
    part = os.path.join(folder, "." + fid + inbox.JSON + ".part")
    with open(part, "w") as fh:
        json.dump(poses_doc(poses, fid), fh)
    os.rename(part, os.path.join(folder, fid + inbox.JSON))
    return len(xyz)


class Pairer(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.cloud = None
        self.poses = None

    def add(self, cloud=None, poses=None):
        with self.lock:
            if cloud is not None:
                self.cloud = cloud
            if poses is not None:
                self.poses = poses
            if self.cloud is None or self.poses is None:
                return None
            if stamp_of(self.cloud) != stamp_of(self.poses):
                return None
            pair = self.cloud, self.poses
            self.cloud = self.poses = None
            return pair
