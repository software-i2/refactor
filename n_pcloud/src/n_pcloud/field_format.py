#!/usr/bin/env python
# Copyright by BeeX [2026]

"""BXFIELD serialization for the obstacle field."""

from __future__ import print_function

import hashlib

import numpy as np

MAGIC = b"BXFIELD4"

# magic + 3 doubles + lo[3] + dims[3] + nlinks + digest + xyz[3] + rpy[3]
_HEADER_BYTES = 8 + 24 + 24 + 12 + 4 + 8 + 24 + 24


def digest(ply_path, json_path, res, delta, xyz, links, filt=(), rpy=None):
    """Stable input fingerprint for a scene field."""
    h = hashlib.sha256()
    for path in (ply_path, json_path):
        with open(path, "rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b""):
                h.update(chunk)
    for value in (res, delta) + tuple(filt):
        h.update(np.float64(value).tobytes())
    h.update(np.asarray(xyz, dtype="<f8").tobytes())
    for name, radius in links:
        h.update(name.encode("utf-8"))
        h.update(np.float64(radius).tobytes())
    if rpy is not None and np.any(np.asarray(rpy, dtype=float) != 0.0):
        h.update(np.asarray(rpy, dtype="<f8").tobytes())
    return int(np.frombuffer(h.digest()[:8], dtype="<u8")[0])


def read_header(path):
    with open(path, "rb") as f:
        if f.read(8) != MAGIC:
            raise ValueError("%s is not a %s file" % (path, MAGIC.decode()))
        res, step, _reserved = np.frombuffer(f.read(24), "<f8")
        lo = np.frombuffer(f.read(24), "<f8").copy()
        dims = np.frombuffer(f.read(12), "<i4").copy()
        nlinks = int(np.frombuffer(f.read(4), "<i4")[0])
        stamp = int(np.frombuffer(f.read(8), "<u8")[0])
        xyz = np.frombuffer(f.read(24), "<f8").copy()
        rpy = np.frombuffer(f.read(24), "<f8").copy()
        radii = np.frombuffer(f.read(8 * nlinks), "<f8").copy()
    return {"res": float(res), "step": float(step), "lo": lo, "dims": dims,
            "digest": stamp, "xyz": xyz, "rpy": rpy, "radii": radii}


def read_field(path):
    """Header plus the voxel grid, for drawing what the arm checks against."""
    head = read_header(path)
    with open(path, "rb") as f:
        f.seek(_HEADER_BYTES + 8 * len(head["radii"]))
        data = np.frombuffer(f.read(), dtype=np.uint8)

    dims = head["dims"]
    cells = int(dims[0]) * int(dims[1]) * int(dims[2])
    if data.size != cells:
        raise ValueError("%s holds %d voxels, header says %d" % (path, data.size, cells))
    head["data"] = data.reshape(tuple(int(d) for d in dims))
    return head


def write_field(path, packed, lo, res, step, radii, xyz, stamp, rpy=None):
    dims = np.asarray(packed.shape, dtype=np.int64)
    with open(path, "wb") as f:
        f.write(MAGIC)
        f.write(np.float64(res).tobytes())
        f.write(np.float64(step).tobytes())
        f.write(np.float64(0.0).tobytes())            # reserved, was margin
        f.write(np.asarray(lo, dtype="<f8").tobytes())
        f.write(np.asarray(dims, dtype="<i4").tobytes())
        f.write(np.int32(len(radii)).tobytes())
        f.write(np.uint64(stamp).tobytes())
        f.write(np.asarray(xyz, dtype="<f8").tobytes())
        f.write(np.asarray((0.0, 0.0, 0.0) if rpy is None else rpy, dtype="<f8").tobytes())
        f.write(np.asarray(radii, dtype="<f8").tobytes())
        f.write(np.ascontiguousarray(packed, dtype=np.uint8).tobytes())
    return dims
