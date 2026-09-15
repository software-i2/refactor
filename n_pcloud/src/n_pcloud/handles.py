#!/usr/bin/env python
# Copyright by BeeX [2026]

"""Handle or rope, per grasp pose: the more the pose trace curves, the more it is a handle."""

from __future__ import print_function

import numpy as np

from n_pcloud.occupancy import _intrinsics

MU = (2.2619414139736507,
      1.1477051149648436)

SD = (0.9793411571819177,
      0.9437532509915271)

WEIGHTS = (0.19815769261425545,
           0.5976326457184641)

BIAS = -0.6448103858399833
THRESHOLD = 0.65
WINDOW = 0.06
SETTINGS = (THRESHOLD, WINDOW, BIAS) + MU + SD + WEIGHTS


def _at(L, i, s):
    return int(np.clip(np.searchsorted(L, L[i] + s), 0, len(L) - 1))


def _smooth(v, L, window=WINDOW):
    out = np.full(len(v), np.nan)
    for i in range(len(v)):
        q = v[_at(L, i, -window / 2.0):_at(L, i, window / 2.0) + 1]
        q = q[np.isfinite(q)]
        if len(q):
            out[i] = np.median(q)
    return out


def _curvature(p, L, s):
    out = np.full(len(p), np.nan)
    for i in range(len(p)):
        j, k = _at(L, i, -s), _at(L, i, s)
        if L[i] - L[j] < 0.7 * s or L[k] - L[i] < 0.7 * s:
            continue
        a, b, c = p[j], p[i], p[k]
        den = np.linalg.norm(b - a) * np.linalg.norm(c - b) * np.linalg.norm(c - a)
        if den > 1e-12:
            out[i] = 2 * np.linalg.norm(np.cross(b - a, c - a)) / den
    return out


def _bend_px(uv, L, s):
    out = np.full(len(uv), np.nan)
    for i in range(len(uv)):
        j, k = _at(L, i, -s), _at(L, i, s)
        ab = uv[k] - uv[j]
        n = np.linalg.norm(ab)
        if n < 3:
            continue
        q = uv[j:k + 1] - uv[j]
        out[i] = np.abs(ab[0] * q[:, 1] - ab[1] * q[:, 0]).max() / n
    return out


def features(pos, uv):
    L = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pos, axis=0), axis=1))]
    with np.errstate(all="ignore"):
        X = np.c_[np.log1p(np.nan_to_num(_smooth(_curvature(pos, L, 0.08), L))),
                  np.log1p(np.nan_to_num(_smooth(_bend_px(uv, L, 0.05), L)))]
    return np.nan_to_num(X)


def handle_score(xyz, pos):
    pos = np.asarray(pos, dtype=float)
    if not len(pos):
        return np.zeros(0)
    intr = _intrinsics(xyz)
    z = -pos[:, 2]
    z = np.where(z > 1e-6, z, 1.0)
    uv = np.c_[intr["fx"] * pos[:, 0] / z + intr["cx"], intr["fy"] * pos[:, 1] / z + intr["cy"]]
    Z = (features(pos, uv) - np.asarray(MU)) / np.asarray(SD)
    return 1.0 / (1.0 + np.exp(-(Z.dot(np.asarray(WEIGHTS)) + BIAS)))


def handle_poses(xyz, pos, threshold=THRESHOLD):
    return handle_score(xyz, pos) >= threshold
