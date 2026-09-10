#!/usr/bin/env python
# Copyright by BeeX [2026]

"""What the arm could take hold of, read out of one Frame.

THE PLACE TO EDIT. Everything here takes a Frame and returns candidates; it
opens no files, knows no placement, and cannot reach the obstacle field. Change
how holds are found and the field cannot break, and vice versa.

A candidate is nine numbers, which is exactly what n_task's readCandidates
parses: point, handle axis, preferred approach. The JSON already carries all
three -- its own coordinate_convention says R's columns are approach, along the
handle, and closing -- so this is a transform, not an interpretation.
"""

from __future__ import print_function

import numpy as np

# R's columns, per the file's own coordinate_convention. Column 2 is the closing
# axis, which the arm derives from the other two and so is never read here.
APPROACH = 0   # into the object
ALONG = 1      # down the handle


def candidates(frame):
    """(N, 3, 3) of point, axis, approach in arm_base, one row per pose."""
    point = frame.to_arm(frame.pos)
    axis = frame.directions_to_arm(frame.rot[:, :, ALONG])
    approach = frame.directions_to_arm(frame.rot[:, :, APPROACH])
    return np.stack([point, axis, approach], axis=1)


def in_reach(cand, reach_max, floor_z):
    """Drop what the arm plainly cannot get to, before anything expensive runs.

    Deliberately generous: this is a screen, not a verdict. n_check owns the
    real answer, and a candidate wrongly dropped here is one it never gets to
    refuse with a reason.
    """
    point = cand[:, 0, :]
    return (np.linalg.norm(point, axis=1) <= reach_max) & (point[:, 2] >= floor_z)


def flatten(cand):
    """Nine floats per candidate, in the order readCandidates expects."""
    return np.asarray(cand, dtype=np.float32).reshape(-1).tolist()
