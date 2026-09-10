#!/usr/bin/env python
# Copyright by BeeX [2026]

"""Camera capture data and the conversion from camera frame to arm_base."""

from __future__ import print_function

import json

import numpy as np

# Camera optical frame -> arm_base.
CAM_TO_ARM = np.array([[0.0, 0.0, -1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0]])

_PLY_TYPES = {
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2", "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4",
    "float": "f4", "float32": "f4", "double": "f8", "float64": "f8",
}


def load_ply(path):
    """Read XYZ vertices from a binary PLY."""
    with open(path, "rb") as f:
        if f.readline().strip() != b"ply":
            raise ValueError("%s is not a PLY file" % path)

        fmt, count, props, in_vertex = None, None, [], False
        while True:
            line = f.readline()
            if not line:
                raise ValueError("%s: header never ended" % path)
            tok = line.split()
            if not tok:
                continue
            if tok[0] == b"format":
                fmt = tok[1].decode()
            elif tok[0] == b"element":
                in_vertex = tok[1] == b"vertex"
                if in_vertex:
                    count = int(tok[2])
            elif tok[0] == b"property" and in_vertex:
                if tok[1] == b"list":
                    raise ValueError("%s: list properties on vertex unsupported" % path)
                props.append((str(tok[2].decode()), _PLY_TYPES[tok[1].decode()]))
            elif tok[0] == b"end_header":
                break

        if fmt == "binary_little_endian":
            dt = np.dtype([(n, "<" + t) for n, t in props])
        elif fmt == "binary_big_endian":
            dt = np.dtype([(n, ">" + t) for n, t in props])
        else:
            raise ValueError("%s: %r is not a binary PLY" % (path, fmt))
        data = np.frombuffer(f.read(count * dt.itemsize), dtype=dt, count=count)

    return np.stack([data["x"], data["y"], data["z"]], axis=1).astype(np.float64)


def _quat_matrix(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        raise ValueError("zero-length quaternion")
    s = 2.0 / n
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y)],
    ])


def load_poses(path):
    """Read camera-frame grasp poses from the JSON scene metadata."""
    with open(path) as fh:
        doc = json.load(fh)

    units = doc.get("units", "meters")
    if units not in ("meters", "metres", "m"):
        raise ValueError("%s is in %r; this reader only speaks metres" % (path, units))

    pos, rot = [], []
    for entry in doc.get("poses", []):
        pos.append([float(v) for v in entry["position_m"]])
        quat = entry.get("quaternion_xyzw")
        if quat is not None:
            rot.append(_quat_matrix([float(v) for v in quat]))
        else:
            rot.append(np.asarray(entry["rotation_3x3"], dtype=float))

    if not pos:
        raise ValueError("%s has no poses" % path)
    return np.asarray(pos, dtype=float), np.asarray(rot, dtype=float)


class Frame(object):
    """A cloud and set of grasp poses in arm_base coordinates."""

    def __init__(self, name, points, pos, rot, at):
        self.name = name
        self.points = points        # (N, 3) camera frame
        self.pos = pos              # (M, 3) camera frame
        self.rot = rot              # (M, 3, 3) camera frame
        self.at = np.asarray(at, dtype=float)
        if self.at.shape != (3,):
            raise ValueError("placement must be three numbers, got %r" % (at,))

    def to_arm(self, v):
        """Camera frame to arm_base, for points."""
        return np.asarray(v, dtype=float).dot(CAM_TO_ARM.T) + self.at

    def directions_to_arm(self, v):
        """Directions carry no translation."""
        return np.asarray(v, dtype=float).dot(CAM_TO_ARM.T)


def read(name, ply_path, json_path, at):
    pos, rot = load_poses(json_path)
    return Frame(name, load_ply(ply_path), pos, rot, at)
