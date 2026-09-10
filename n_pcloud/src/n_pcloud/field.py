#!/usr/bin/env python
# Copyright by BeeX [2026]

"""The obstacle field the arm loads: binary occupancy pre-fattened by each link
radius, so a capsule test on the C++ side is a plain lookup along the axis.

The dilation is the link radius and nothing else. Grid quantisation means a
contact can be missed by up to about 3*res*sqrt(3)/2; that is accepted rather
than padded for.
"""

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
    """How finely n_check samples the blade interiors. Half the voxel, because
    at pitch == res the sound radius below is res*sqrt(3) -- wider than the gap
    a handle actually leaves beside it, so every real grasp would be refused.
    Halving the pitch costs eight times the samples and buys back a quarter of
    the radius. n_check derives the same number from the field's own res, so the
    two cannot disagree; check::fitsField proves it on load."""
    return 0.5 * res


def jaw_radius(res, pitch=None):
    """Raw occupancy, inflated just enough that sampling a blade box is sound.

    A blocked voxel can overlap the box while its centre lies outside it, so the
    radius has to cover both hops: up to res*sqrt(3)/2 from that centre to a
    point shared with the box, then up to pitch*sqrt(3)/2 from there to the
    nearest sample. Covering only the second is the easy mistake, and it leaves
    a blade able to pass through a voxel no sample landed in.
    """
    pitch = blade_pitch(res) if pitch is None else pitch
    return (res + pitch) * math.sqrt(3.0) / 2.0


def links_for(res):
    """Name and dilation radius per link, in the bit order n_check/Field.h reads:
    upper_arm, forearm, wrist_mount, palm, jaw."""
    return CAPSULES + (("jaw", jaw_radius(res)),)


def build(label, lo, res, step):
    """Dilate the blocked voxels by each link's radius, one bit per link.

    The jaw plane is dilated from a grid with TARGET removed: a 90 mm claw
    closing on a handle that stands off a hull has to put its tips in the hull's
    space, and there is no posture that avoids it. The palm and the arm have no
    such excuse, so the exemption is built into the jaw plane only -- carving it
    out of every plane is what let the whole gripper drive into the mine.
    """
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
    """frame.CAM_TO_ARM is a signed axis permutation, so the grid transposes and
    flips into arm_base exactly. Anything else would need resampling, which is
    why placement is a translation and not a pose."""
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
    """Rotate into arm_base, shift by the placement, write."""
    packed, lo = _to_arm(field["packed"], field["lo"], field["res"])
    lo = lo + np.asarray(at, dtype=float)
    radii = [r for _n, r in field["links"]]

    dims = write_field(path, packed, lo, field["res"], field["step"], radii,
                       np.asarray(at, dtype=float), stamp)
    return os.path.getsize(path), lo, dims
