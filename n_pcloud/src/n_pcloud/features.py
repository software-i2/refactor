#!/usr/bin/env python
# Copyright by BeeX [2026]

"""Candidate handles extracted from a camera frame."""

from __future__ import print_function

import numpy as np

APPROACH = 0
ALONG = 1


def candidates(frame):
    point = frame.to_arm(frame.pos)
    axis = frame.directions_to_arm(frame.rot[:, :, ALONG])
    approach = frame.directions_to_arm(frame.rot[:, :, APPROACH])
    return np.stack([point, axis, approach], axis=1)


def in_reach(cand, reach_max, floor_z):
    point = cand[:, 0, :]
    return (np.linalg.norm(point, axis=1) <= reach_max) & (point[:, 2] >= floor_z)


def flatten(cand):
    return np.asarray(cand, dtype=np.float32).reshape(-1).tolist()
