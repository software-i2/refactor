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
HANDLE_RADIUS = 0.005    # m, the handle around a grasp, exempt from the filter
PAD = 0.02               # m, margin the grid keeps around the cloud


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


def _shift(img, dy, dx):
    out = np.full(img.shape, np.inf)
    h, w = img.shape
    ys, ye = max(0, dy), min(h, h + dy)
    xs, xe = max(0, dx), min(w, w + dx)
    out[ys:ye, xs:xe] = img[ys - dy:ye - dy, xs - dx:xe - dx]
    return out


def _window_mean(a, valid, window):
    weight = ndimage.uniform_filter(valid.astype(np.float64), window)
    total = ndimage.uniform_filter(np.where(valid, a, 0.0), window)
    return total / np.maximum(weight, 1e-9)


def _local_tilt(img, valid, window):
    """Depth gradient per pixel, least squares over the window."""
    u = np.arange(img.shape[1], dtype=np.float64)[None, :]
    v = np.arange(img.shape[0], dtype=np.float64)[:, None]
    mean_z = _window_mean(img, valid, window)
    spread = (window * window - 1) / 12.0
    return ((_window_mean(img * u, valid, window) - u * mean_z) / spread,
            (_window_mean(img * v, valid, window) - v * mean_z) / spread)


def flying_pixels(xyz, tol=SUPPORT_TOL, need=SUPPORT_MIN, window=SUPPORT_WINDOW):
    """Returns that no neighbour supports, as a mask over xyz.

    A stereo match straddling an occlusion edge lands between the near surface
    and the far one and belongs to neither, which is what this asks. The
    comparison runs against the local tilt rather than the raw depth, so a steep
    surface is not mistaken for a straddle.
    """
    intr = _intrinsics(xyz)
    img = _depth_image(xyz, intr)
    valid = np.isfinite(img)
    dzdx, dzdy = _local_tilt(img, valid, window)

    agree = np.zeros(img.shape, dtype=np.int16)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx or dy:
                off = _shift(img, dy, dx) - img - dzdx * dx - dzdy * dy
                agree += (np.abs(off) <= tol)

    px, py, on = _pixel_of(xyz, intr)
    drop = np.zeros(len(xyz), dtype=bool)
    drop[on] = ~(valid & (agree >= need))[py[on], px[on]]
    return drop


def near_grasps(xyz, grasps, res=0.005, pad=PAD, radius=HANDLE_RADIUS):
    """Returns landing in a voxel carve_target will call TARGET.

    They are exempt from the filter: a handle is thin and obliquely seen, which
    is the one case a straddle and a real surface look alike. Keeping them costs
    the jaw nothing, since field.py dilates its plane from a grid with TARGET
    taken out. Sharing _grasp_voxels with carve_target is what stops the exempt
    set and the carved set from drifting apart.
    """
    grasps = np.asarray(grasps, dtype=float)
    out = np.zeros(len(xyz), dtype=bool)
    if not len(grasps):
        return out

    lo, dims = grid_bounds(xyz, res, pad)
    near = _grasp_voxels(grasps, lo, res, dims, radius)
    idx, inside = world_to_index(xyz, lo, res, dims)
    idx = idx[inside]
    out[inside] = near[idx[:, 0], idx[:, 1], idx[:, 2]]
    return out


def grid_bounds(xyz, res, pad=PAD):
    """Origin and shape of the voxel grid holding the cloud."""
    lo = xyz.min(0) - pad
    dims = np.maximum(np.ceil((xyz.max(0) + pad - lo) / res).astype(int), 1)
    return lo, dims


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


def classify(xyz, res=0.005, pad=PAD, tol=None, chunk=32, keep=None,
             trusted=None, min_hits=CARVE_MIN_HITS):
    """Label every voxel FREE, OBSTACLE or UNKNOWN. Chunked over the first axis
    because the whole grid of ray lookups at once costs gigabytes.

    `keep` selects which returns may mark a voxel occupied; the depth image is
    built from all of them either way, so filtering never costs free space. A
    voxel the rays looked through needs min_hits returns to be called back --
    one straggler does not outvote the carve.

    `trusted` returns stand on their own. Rays passing either side of a thin
    thing carve it free, so a handle one return wide loses the vote it should
    win, and that vote is the whole reason to name an exception.
    """
    intr = _intrinsics(xyz)
    img = _depth_image(xyz, intr)
    lo, dims = grid_bounds(xyz, res, pad)
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

    idx, inside = world_to_index(xyz, lo, res, dims)

    def stamped(mask):
        take = inside if mask is None else (inside & np.asarray(mask, dtype=bool))
        grid = np.zeros(tuple(dims), dtype=np.int32)
        np.add.at(grid, tuple(idx[take].T), 1)
        return grid

    hits = stamped(keep)
    sure = np.zeros(tuple(dims), dtype=bool) if trusted is None else stamped(trusted) > 0
    state[(hits >= min_hits) | ((hits > 0) & (state != FREE)) | sure] = OBSTACLE
    return state, lo


def _grasp_voxels(grasps, lo, res, dims, radius):
    """The voxels a grasp sphere covers."""
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
    return near


def carve_target(state, lo, res, grasps, radius=HANDLE_RADIUS, max_radius=0.015):
    label = state.copy()
    dims = np.array(state.shape)
    radius = min(float(radius), float(max_radius))

    grasps = np.asarray(grasps, dtype=float)
    if not len(grasps):
        return label, 0

    near = _grasp_voxels(grasps, lo, res, dims, radius)
    is_target = near & (label == OBSTACLE)
    label[is_target] = TARGET
    return label, int(is_target.sum())
