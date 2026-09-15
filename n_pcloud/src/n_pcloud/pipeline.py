#!/usr/bin/env python
# Copyright by BeeX [2026]

"""One capture to an obstacle field and grasp candidates."""

from __future__ import print_function

import time

from n_pcloud import features, field, handles
from n_pcloud import occupancy as occ
from n_pcloud.field_format import digest


class Clock(object):
    def __init__(self):
        self.times = []
        self.mark = time.time()

    def lap(self, stage):
        now = time.time()
        self.times.append((stage, now - self.mark))
        self.mark = now


def stamp(ply_path, json_path, at, rpy, res, step, raw=False, box=None, all_poses=False):
    filt = () if raw else (occ.SUPPORT_TOL, occ.SUPPORT_MIN,
                           occ.SUPPORT_WINDOW, occ.CARVE_MIN_HITS,
                           occ.HANDLE_RADIUS, occ.BAR_GAP,
                           occ.CORRIDOR_LENGTH, occ.CORRIDOR_RADIUS)
    if not all_poses:
        filt += handles.SETTINGS
    return digest(ply_path, json_path, res, step, at, field.links_for(res), filt, rpy, box)


def run(f, stamp, res, step, reach_max, floor_z, raw=False, box=None, clock=None,
        all_poses=False):
    clock = clock or Clock()
    out = {"frame": f, "stamp": stamp, "times": clock.times,
           "dropped": None, "spared": None, "placed": None, "cropped": None, "handle": None}

    keep_mask, handle_mask, hit_threshold = None, None, 1
    if not raw:
        handle_mask = occ.near_grasps(f.points, f.pos, res=res)
        keep_mask = ~occ.flying_pixels(f.points) | handle_mask
        hit_threshold = occ.CARVE_MIN_HITS
        out["dropped"] = len(f.points) - int(keep_mask.sum())
        out["spared"] = int(handle_mask.sum())
        clock.lap("filter")

    label, lo = occ.classify(f.points, res=res, keep=keep_mask,
                             trusted=handle_mask, hit_threshold=hit_threshold)
    clock.lap("classify")

    pick = slice(None)
    if not all_poses:
        out["handle"] = handles.handle_poses(f.points, f.pos)
        pick = out["handle"]
        clock.lap("handles")

    label, out["carved"] = occ.carve_target(label, lo, res, f.pos[pick])
    label, out["corridor"] = occ.carve_corridor(label, lo, res, f.pos[pick],
                                                f.rot[pick][:, :, features.APPROACH])
    out["grid"] = tuple(label.shape)
    out["counts"] = dict((occ.NAME[v], int((label == v).sum())) for v in occ.NAME)
    out["label"], out["lo"] = label, lo
    clock.lap("carve")

    near = box
    if field.tilted(f.rpy):
        label, lo = field.place(label, lo, res, f.at, f.rpy, box)
        out["placed"] = tuple(label.shape)
        clock.lap("place")
    elif box is not None:
        near = field.box_in_camera(box, f.at)

    built = None if box is None else field.build_within(label, lo, res, step, near)
    if built is None:
        built = field.build(label, lo, res, step)
    else:
        out["cropped"] = tuple(built["packed"].shape)
    out["field"] = built
    clock.lap("build")

    out["cand"] = features.candidates(f)
    out["keep"] = features.in_reach(out["cand"], reach_max=reach_max, floor_z=floor_z)
    if out["handle"] is not None:
        out["keep"] &= out["handle"]
    clock.lap("candidates")
    return out


def save(r, field_path=None, candidates_path=None):
    t = time.time()
    if field_path:
        r["size"], r["arm_lo"], r["dims"] = field.export(field_path, r["field"], r["frame"].at,
                                                         r["stamp"], r["frame"].rpy)
    if candidates_path:
        flat = features.flatten(r["cand"][r["keep"]])
        with open(candidates_path, "w") as fh:
            fh.write("[%s]\n" % ", ".join("%.6f" % v for v in flat))
        r["floats"] = len(flat)
    r["times"].append(("write", time.time() - t))
