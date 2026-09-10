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


def export(path, field, at, stamp):
    packed, lo = _to_arm(field["packed"], field["lo"], field["res"])
    lo = lo + np.asarray(at, dtype=float)
    radii = [r for _n, r in field["links"]]

    dims = write_field(path, packed, lo, field["res"], field["step"], radii,
                       np.asarray(at, dtype=float), stamp)
    return os.path.getsize(path), lo, dims
