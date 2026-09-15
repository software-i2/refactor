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

REACH = ((-0.43, -0.43, -0.36), (0.43, 0.43, 0.46))


def blade_pitch(res):
    return 0.5 * res


def jaw_radius(res, pitch=None):
    pitch = blade_pitch(res) if pitch is None else pitch
    return (res + pitch) * math.sqrt(3.0) / 2.0


def links_for(res):
    return CAPSULES + (("jaw", jaw_radius(res)),)


def reach_pad(res):
    return int(math.ceil(max(r for _n, r in links_for(res)) / res)) + 1


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


def _cells(first, last):
    return tuple(slice(int(i), int(j)) for i, j in zip(first, last))


def _window(lo, dims, res, box, pad):
    first = np.floor((np.asarray(box[0], dtype=float) - lo) / res).astype(np.int64) - pad
    last = np.ceil((np.asarray(box[1], dtype=float) - lo) / res).astype(np.int64) + pad
    return np.clip(first, 0, dims), np.clip(last, 0, dims)


def build_within(label, lo, res, step, box):
    dims = np.asarray(label.shape, dtype=np.int64)
    first, last = _window(lo, dims, res, box, 0)
    if np.any(last <= first):
        return None
    grow_first, grow_last = _window(lo, dims, res, box, reach_pad(res))
    built = build(label[_cells(grow_first, grow_last)], lo + grow_first * res, res, step)
    built["packed"] = built["packed"][_cells(first - grow_first, last - grow_first)]
    built["lo"] = lo + first * res
    return built


def box_in_camera(box, at):
    from n_pcloud.frame import CAM_TO_ARM

    lo = np.asarray(box[0], dtype=float)
    hi = np.asarray(box[1], dtype=float)
    unit = np.array([[i, j, k] for i in (0, 1) for j in (0, 1) for k in (0, 1)], dtype=float)
    corners = (lo + unit * (hi - lo) - np.asarray(at, dtype=float)).dot(CAM_TO_ARM)
    return corners.min(0), corners.max(0)


def _square(R):
    src = np.argmax(np.abs(R), axis=1)
    return (len(set(int(s) for s in src)) == 3
            and np.allclose(np.abs(R[np.arange(3), src]), 1.0, atol=1e-9)
            and np.allclose(np.abs(R).sum(axis=1), 1.0, atol=1e-9))


def _permute(grid, lo, res, R):
    src = np.argmax(np.abs(R), axis=1)
    sign = np.sign(R[np.arange(3), src])
    dims = np.asarray(grid.shape, dtype=np.int64)

    out = np.transpose(grid, axes=tuple(int(s) for s in src))
    lo_new = np.empty(3, dtype=np.float64)
    for a in range(3):
        c = int(src[a])
        if sign[a] > 0:
            lo_new[a] = lo[c]
        else:
            lo_new[a] = -(lo[c] + dims[c] * res)
            out = np.flip(out, axis=a)
    return np.ascontiguousarray(out), lo_new


def _to_arm(packed, lo, res):
    from n_pcloud.frame import CAM_TO_ARM

    return _permute(packed, lo, res, CAM_TO_ARM)


def tilted(rpy):
    return rpy is not None and np.any(np.asarray(rpy, dtype=float) != 0.0)


def place(label, lo, res, at, rpy, box=None):
    from n_pcloud.frame import cam_to_arm

    R = cam_to_arm(rpy)
    at = np.asarray(at, dtype=float)

    if _square(R):
        out, arm_lo = _permute(label, lo, res, R)
        arm_lo = arm_lo + at
        if box is not None:
            first, last = _window(arm_lo, np.asarray(out.shape, dtype=np.int64), res, box, reach_pad(res))
            if np.all(last > first):
                return np.ascontiguousarray(out[_cells(first, last)]), arm_lo + first * res
        return out, arm_lo
    dims = np.asarray(label.shape, dtype=np.int64)

    unit = np.array([[i, j, k] for i in (0, 1) for j in (0, 1) for k in (0, 1)], dtype=float)
    corners = (lo + unit * dims * res).dot(R.T) + at
    arm_lo = corners.min(0)
    arm_dims = np.maximum(np.ceil((corners.max(0) - arm_lo) / res - 1e-6).astype(np.int64), 1)

    first, last = np.zeros(3, dtype=np.int64), arm_dims
    if box is not None:
        grow_first, grow_last = _window(arm_lo, arm_dims, res, box, reach_pad(res))
        if np.all(grow_last > grow_first):
            first, last = grow_first, grow_last
    size = last - first

    out = np.full(tuple(size), occ.UNKNOWN, dtype=np.uint8)
    jj, kk = np.meshgrid(np.arange(first[1], last[1]), np.arange(first[2], last[2]),
                         indexing="ij")
    centre = np.empty(jj.shape + (3,))
    centre[..., 1] = arm_lo[1] + (jj + 0.5) * res
    centre[..., 2] = arm_lo[2] + (kk + 0.5) * res

    for n, i in enumerate(range(first[0], last[0])):
        centre[..., 0] = arm_lo[0] + (i + 0.5) * res
        idx = np.floor(((centre - at).dot(R) - lo) / res).astype(np.int64)
        inside = np.all((idx >= 0) & (idx < dims), axis=-1)
        hit = idx[inside]
        out[n][inside] = label[hit[:, 0], hit[:, 1], hit[:, 2]]

    sub = (np.arange(6) + 0.5) / 6.0
    offsets = np.array([[a, b, c] for a in sub for b in sub for c in sub])
    for value in (occ.TARGET, occ.OBSTACLE):
        src = np.argwhere(label == value)
        mid = ((lo + (src + 0.5) * res).dot(R.T) + at - arm_lo) / res - first
        src = src[np.all((mid >= -1.0) & (mid < size + 1.0), axis=1)]
        for off in offsets:
            idx = np.floor(((lo + (src + off) * res).dot(R.T) + at - arm_lo) / res).astype(np.int64)
            idx = idx[np.all((idx >= first) & (idx < last), axis=1)] - first
            cell = (idx[:, 0], idx[:, 1], idx[:, 2])
            if value == occ.TARGET:
                out[cell] = np.where(out[cell] == occ.OBSTACLE, occ.OBSTACLE, occ.TARGET)
            else:
                out[cell] = occ.OBSTACLE
    return out, arm_lo + first * res


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
