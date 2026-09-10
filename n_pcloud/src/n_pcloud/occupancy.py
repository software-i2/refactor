#!/usr/bin/env python
# Copyright by BeeX [2026]

"""What the camera saw, as a labelled voxel grid.

Camera frame throughout. The free-space carve reads along the camera's own
rays -- a voxel nearer than the measured depth on the ray through it was looked
through, so it is free -- and that is not a thing that can be done after the
cloud has been turned into arm_base.

Four labels and no more. GROUND and BOX existed for a plane fit and a manual
box stamper that are both gone.
"""

from __future__ import print_function

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

UNKNOWN = 0   # never on a ray, or beyond what was measured
FREE = 1      # looked through
OBSTACLE = 2  # a return landed here
TARGET = 3    # obstacle that is the handle itself

NAME = {UNKNOWN: "unknown", FREE: "free", OBSTACLE: "obstacle", TARGET: "target"}

# What stops the arm. TARGET is deliberately absent: the jaws have to enter the
# handle's own voxels to close on it, and field.py grants that to the jaw plane
# alone.
BLOCKING = (OBSTACLE,)

# Noise filter tuning. SUPPORT_* govern which pixels survive as returns;
# CARVE_MIN_HITS governs how many returns it takes to overrule the ray carve.
SUPPORT_TOL = 0.004      # m, how far a neighbour may sit off the local surface
SUPPORT_MIN = 2          # neighbours, of the eight, that must agree
SUPPORT_WINDOW = 5       # px, window the local tilt is fitted over
CARVE_MIN_HITS = 2       # returns needed to call a carved-free voxel occupied
HANDLE_RADIUS = 0.005    # m, returns this close to a grasp are never dropped


def _axis_lattice(a, min_gap=1e-5):
    s = np.sort(a)
    d = np.diff(s)
    big = d[d > min_gap]
    if len(big) < 2:
        raise ValueError("no pixel lattice on this axis; this is not a raw depth map")
    step = float(np.median(big))
    return 1.0 / step, -float(s[0]) / step, len(big) + 1


def _intrinsics(xyz):
    """Recovered from the cloud itself: a raw depth map still lies on the pixel
    lattice it was projected from, so fx, fy, cx, cy can be read back off it."""
    z = xyz[:, 2]
    if not (z < 0.0).all():
        raise ValueError("expected a camera frame looking down -Z")
    fx, cx, width = _axis_lattice(xyz[:, 0] / -z)
    fy, cy, height = _axis_lattice(xyz[:, 1] / -z)
    return {"fx": fx, "fy": fy, "cx": cx, "cy": cy, "width": width, "height": height}


def _pixel_of(xyz, intr):
    """Pixel each return projects to, and whether it lands on the sensor."""
    z = -xyz[:, 2]
    px = np.rint(intr["fx"] * xyz[:, 0] / z + intr["cx"]).astype(np.int64)
    py = np.rint(intr["fy"] * xyz[:, 1] / z + intr["cy"]).astype(np.int64)
    on = ((px >= 0) & (px < intr["width"]) & (py >= 0) & (py < intr["height"]))
    return px, py, on


def _depth_image(xyz, intr):
    px, py, on = _pixel_of(xyz, intr)
    img = np.full((intr["height"], intr["width"]), np.inf)
    np.minimum.at(img, (py[on], px[on]), -xyz[on, 2])
    return img


def world_to_index(points, lo, res, dims):
    idx = np.floor((np.asarray(points, dtype=float) - lo) / res).astype(np.int64)
    inside = np.ones(len(idx), dtype=bool)
    for a in range(3):
        inside &= (idx[:, a] >= 0) & (idx[:, a] < dims[a])
    return idx, inside


def sphere_offsets(radius, res):
    r = int(np.ceil(radius / res))
    g = np.arange(-r, r + 1)
    dx, dy, dz = np.meshgrid(g, g, g, indexing="ij")
    keep = (dx * dx + dy * dy + dz * dz) * res * res <= radius * radius
    return np.stack([dx[keep], dy[keep], dz[keep]], axis=1)


def classify(xyz, res=0.005, pad=0.02, tol=None, chunk=32):
    """Label every voxel FREE, OBSTACLE or UNKNOWN. Chunked over the first axis
    because the whole grid of ray lookups at once costs gigabytes."""
    intr = _intrinsics(xyz)
    img = _depth_image(xyz, intr)
    lo = xyz.min(0) - pad
    dims = np.maximum(np.ceil((xyz.max(0) + pad - lo) / res).astype(int), 1)
    tol = res if tol is None else tol

    state = np.full(tuple(dims), UNKNOWN, dtype=np.uint8)
    jj, kk = np.meshgrid(np.arange(dims[1]), np.arange(dims[2]), indexing="ij")
    cy_ = lo[1] + (jj + 0.5) * res
    cz_ = lo[2] + (kk + 0.5) * res

    for i0 in range(0, dims[0], chunk):
        i1 = min(i0 + chunk, dims[0])
        cx_ = lo[0] + (np.arange(i0, i1) + 0.5) * res
        x = np.broadcast_to(cx_[:, None, None], (i1 - i0,) + cy_.shape)
        y = np.broadcast_to(cy_[None, :, :], x.shape)
        z = np.broadcast_to(cz_[None, :, :], x.shape)

        depth = -z
        in_front = depth > 1e-6
        safe = np.where(in_front, depth, 1.0)
        px = np.rint(intr["fx"] * x / safe + intr["cx"]).astype(np.int64)
        py = np.rint(intr["fy"] * y / safe + intr["cy"]).astype(np.int64)
        seen = (in_front & (px >= 0) & (px < intr["width"])
                & (py >= 0) & (py < intr["height"]))

        measured = np.full(x.shape, np.inf)
        measured[seen] = img[py[seen], px[seen]]
        block = np.full(x.shape, UNKNOWN, dtype=np.uint8)
        block[seen & np.isfinite(measured) & (depth < measured - tol)] = FREE
        state[i0:i1] = block

    hit, inside = world_to_index(xyz, lo, res, dims)
    hit = hit[inside]
    state[hit[:, 0], hit[:, 1], hit[:, 2]] = OBSTACLE
    return state, lo


def carve_target(state, lo, res, grasps, radius=0.005, max_radius=0.015):
    label = state.copy()
    dims = np.array(state.shape)
    radius = min(float(radius), float(max_radius))

    grasps = np.asarray(grasps, dtype=float)
    if not len(grasps):
        return label, 0

    gidx, ginside = world_to_index(grasps, lo, res, dims)
    gidx = gidx[ginside]

    off = sphere_offsets(radius, res)
    hit = (gidx[:, None, :] + off[None, :, :]).reshape(-1, 3)
    ok = np.ones(len(hit), dtype=bool)
    for a in range(3):
        ok &= (hit[:, a] >= 0) & (hit[:, a] < dims[a])
    hit = hit[ok]

    near = np.zeros(tuple(dims), dtype=bool)
    near[hit[:, 0], hit[:, 1], hit[:, 2]] = True

    is_target = near & (label == OBSTACLE)
    label[is_target] = TARGET
    return label, int(is_target.sum())
