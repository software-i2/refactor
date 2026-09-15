#!/usr/bin/env python
# Copyright by BeeX [2026]

"""One capture through n_pcloud into its frame folder."""

from __future__ import print_function

import json
import os
import shutil
import time

from n_live import store
from n_pcloud import field, frame, pipeline

FIELD = "field.bin"
CANDIDATES = "candidates.txt"


class Params(object):
    def __init__(self, at, rpy, res=0.0025, step=0.002, reach_max=0.35, floor_z=0.0, full=False):
        self.at = [float(v) for v in at]
        self.rpy = [float(v) for v in rpy]
        self.res = float(res)
        self.step = float(step)
        self.reach_max = float(reach_max)
        self.floor_z = float(floor_z)
        self.box = None if full else field.REACH


def _move(path, folder):
    dst = os.path.join(folder, os.path.basename(path))
    shutil.move(path, dst)
    return dst


def build(fid, ply, js, folder, p):
    arrived = os.path.getmtime(js)
    ply = _move(ply, folder)
    js = _move(js, folder)

    clock = pipeline.Clock()
    stamp = pipeline.stamp(ply, js, p.at, p.rpy, p.res, p.step, False, p.box)
    clock.lap("digest")
    f = frame.read(fid, ply, js, p.at, p.rpy)
    clock.lap("read")
    r = pipeline.run(f, stamp, p.res, p.step, p.reach_max, p.floor_z, False, p.box, clock)
    pipeline.save(r, os.path.join(folder, FIELD), os.path.join(folder, CANDIDATES))

    doc = {
        "id": fid,
        "at": p.at,
        "rpy": p.rpy,
        "res": p.res,
        "step": p.step,
        "reach_max": p.reach_max,
        "floor_z": p.floor_z,
        "cropped": p.box is not None,
        "digest": "%016x" % r["stamp"],
        "points": int(len(f.points)),
        "poses": int(len(f.pos)),
        "candidates": int(r["keep"].sum()),
        "carved": int(r["carved"]),
        "corridor": int(r["corridor"]),
        "grid": [int(d) for d in r["grid"]],
        "field_mb": round(r["size"] / 1e6, 2),
        "times": [[stage, round(t, 3)] for stage, t in r["times"]],
        "seconds": round(sum(t for _s, t in r["times"]), 3),
        "arrived": arrived,
        "built": time.time(),
    }
    with open(os.path.join(folder, store.MARK), "w") as fh:
        json.dump(doc, fh, indent=2, sort_keys=True)
    return doc, r
