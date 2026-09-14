#!/usr/bin/env python
# Copyright by BeeX [2026]

"""Precomputed obstacle field for the arm."""

from __future__ import print_function

import math
import os

import numpy as np
from scipy import ndimage

from n_pcloud import occupancy as occ
from n_pcloud.field_format import write_field

CAPSULES = (
    ("upper_arm", 0.020),
    ("forearm", 0.020),
    ("wrist_mount", 0.020),
    ("palm", 0.020),
)


def blade_pitch(res):
    return 0.5 * res


def jaw_radius(res, pitch=None):
    pitch = blade_pitch(res) if pitch is None else pitch
    return (res + pitch) * math.sqrt(3.0) / 2.0


def links_for(res):
    return CAPSULES + (("jaw", jaw_radius(res)),)


def build(label, lo, res, step):
    links = links_for(res)

    exempt = (label == occ.TARGET)
    grid = np.isin(label, occ.BLOCKING) | exempt

    edt = ndimage.distance_transform_edt(~grid, sampling=(res,) * 3)

    jaw_edt = edt
    if np.any(exempt):
        jaw_edt = ndimage.distance_transform_edt(~(grid & ~exempt), sampling=(res,) * 3)

    packed = np.zeros(grid.shape, dtype=np.uint8)
    for bit, (name, radius) in enumerate(links):
        source = jaw_edt if name == "jaw" else edt
        packed |= (source <= radius).astype(np.uint8) << bit

    return {"packed": packed, "lo": np.asarray(lo, dtype=float), "res": res,
            "step": step, "links": links,
            "exempt_voxels": int(np.count_nonzero(exempt))}


def _to_arm(packed, lo, res):
    from n_pcloud.frame import CAM_TO_ARM

    src = np.argmax(np.abs(CAM_TO_ARM), axis=1)
    sign = np.sign(CAM_TO_ARM[np.arange(3), src])
    dims = np.asarray(packed.shape, dtype=np.int64)

    out = np.transpose(packed, axes=tuple(int(s) for s in src))
    lo_new = np.empty(3, dtype=np.float64)
    for a in range(3):
        c = int(src[a])
        if sign[a] > 0:
            lo_new[a] = lo[c]
        else:
            lo_new[a] = -(lo[c] + dims[c] * res)
            out = np.flip(out, axis=a)
    return np.ascontiguousarray(out), lo_new


def tilted(rpy):
    return rpy is not None and np.any(np.asarray(rpy, dtype=float) != 0.0)


def place(label, lo, res, at, rpy):
    from n_pcloud.frame import cam_to_arm

    R = cam_to_arm(rpy)
    at = np.asarray(at, dtype=float)
    dims = np.asarray(label.shape, dtype=np.int64)

    unit = np.array([[i, j, k] for i in (0, 1) for j in (0, 1) for k in (0, 1)], dtype=float)
    corners = (lo + unit * dims * res).dot(R.T) + at
    arm_lo = corners.min(0)
    arm_dims = np.maximum(np.ceil((corners.max(0) - arm_lo) / res - 1e-6).astype(np.int64), 1)

    out = np.full(tuple(arm_dims), occ.UNKNOWN, dtype=np.uint8)
    jj, kk = np.meshgrid(np.arange(arm_dims[1]), np.arange(arm_dims[2]), indexing="ij")
    centre = np.empty(jj.shape + (3,))
    centre[..., 1] = arm_lo[1] + (jj + 0.5) * res
    centre[..., 2] = arm_lo[2] + (kk + 0.5) * res

    for i in range(arm_dims[0]):
        centre[..., 0] = arm_lo[0] + (i + 0.5) * res
        idx = np.floor(((centre - at).dot(R) - lo) / res).astype(np.int64)
        inside = np.all((idx >= 0) & (idx < dims), axis=-1)
        hit = idx[inside]
        out[i][inside] = label[hit[:, 0], hit[:, 1], hit[:, 2]]

    sub = (np.arange(6) + 0.5) / 6.0
    offsets = np.array([[a, b, c] for a in sub for b in sub for c in sub])
    for value in (occ.TARGET, occ.OBSTACLE):
        src = np.argwhere(label == value)
        for off in offsets:
            idx = np.floor(((lo + (src + off) * res).dot(R.T) + at - arm_lo) / res).astype(np.int64)
            idx = idx[np.all((idx >= 0) & (idx < arm_dims), axis=1)]
            cell = (idx[:, 0], idx[:, 1], idx[:, 2])
            if value == occ.TARGET:
                out[cell] = np.where(out[cell] == occ.OBSTACLE, occ.OBSTACLE, occ.TARGET)
            else:
                out[cell] = occ.OBSTACLE
    return out, arm_lo


def export(path, field, at, stamp, rpy=None):
    if tilted(rpy):
        packed, lo = field["packed"], field["lo"]
    else:
        packed, lo = _to_arm(field["packed"], field["lo"], field["res"])
        lo = lo + np.asarray(at, dtype=float)
    radii = [r for _n, r in field["links"]]

    dims = write_field(path, packed, lo, field["res"], field["step"], radii,
                       np.asarray(at, dtype=float), stamp, rpy)
    return os.path.getsize(path), lo, dims
