#!/usr/bin/env python
# Copyright by BeeX [2026]
#
# Melodic only ships numpy/scipy for python2.7, so this must NOT say python3.

"""One capture in, an obstacle field and a set of grasp candidates out.

    rosrun n_pcloud scene.py --scene 000390 --at "-0.2 0 0.3"

--at is where the camera sat in arm_base, and it is required. There is no
default and no lookup table: a guessed placement puts the whole scene somewhere
the arm never looks, every target comes back unreachable, and nothing
downstream can tell you why. The value is recorded in the field header, so the
viz reads it back rather than being told it a second time.
"""

from __future__ import print_function

import argparse
import os
import sys
import time

import numpy as np

from n_pcloud import features, field, frame
from n_pcloud import occupancy as occ
from n_pcloud.field_format import digest, read_header


def placement(text):
    parts = text.replace(",", " ").split()
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            "--at needs three numbers, \"X Y Z\" in metres, got %r" % text)
    return [float(v) for v in parts]


def scene_paths(args):
    if args.scene:
        ply = os.path.join(args.data, "ply", "%s_depth_scene.ply" % args.scene)
        js = os.path.join(args.data, "json", "%s_depth_poses.json" % args.scene)
    else:
        ply, js = args.ply, args.json
    for path in (ply, js):
        if not path or not os.path.isfile(path):
            raise IOError("no such file: %s" % path)
    return ply, js


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", help="capture id, e.g. 000390; resolves both files under --data")
    ap.add_argument("--data", default="data", help="folder holding ply/ and json/")
    ap.add_argument("--ply", help="explicit cloud, instead of --scene")
    ap.add_argument("--json", help="explicit poses, instead of --scene")
    ap.add_argument("--at", type=placement, required=True,
                    metavar='"X Y Z"', help="camera origin in arm_base, metres. Required.")
    ap.add_argument("--res", type=float, default=0.005, help="working voxel size")
    ap.add_argument("--step", type=float, default=0.002, help="axis sample spacing n_check walks")
    ap.add_argument("--reach-max", type=float, default=0.35, dest="reach_max",
                    help="generous screen on how far the arm can get; n_check owns the verdict")
    ap.add_argument("--floor-z", type=float, default=0.0, dest="floor_z",
                    help="screen out candidates below this; n_check owns the verdict")
    ap.add_argument("--raw", action="store_true",
                    help="skip all noise filtering, for comparison")
    ap.add_argument("--out", help="write the field here")
    ap.add_argument("--candidates", help="write the flat candidate list here, one per line")
    ap.add_argument("--check", help="report whether this field still matches these inputs")
    args = ap.parse_args()

    ply_path, json_path = scene_paths(args)
    name = args.scene or os.path.basename(ply_path)
    links = field.links_for(args.res)
    filt = () if args.raw else (occ.SUPPORT_TOL, occ.SUPPORT_MIN,
                                occ.SUPPORT_WINDOW, occ.CARVE_MIN_HITS,
                                occ.HANDLE_RADIUS)
    stamp = digest(ply_path, json_path, args.res, args.step, args.at, links, filt)

    if args.check:
        head = read_header(args.check)
        print("%s" % args.check)
        print("  built from  %016x" % head["digest"])
        print("  these input %016x  (placed at %s)" % (stamp, np.round(args.at, 3)))
        if head["digest"] == stamp:
            print("  FRESH")
            return 0
        print("  STALE: the arm would check a world that no longer matches the inputs.")
        print("  placed at %s when it was built" % np.round(head["xyz"], 3))
        return 3

    t0 = time.time()
    f = frame.read(name, ply_path, json_path, args.at)
    print("%s" % name)
    print("  %s" % ply_path)
    print("  %s" % json_path)
    print("  %d points, %d poses, camera at %s in arm_base"
          % (len(f.points), len(f.pos), np.round(f.at, 3)))

    keep, handle, min_hits = None, None, 1
    if not args.raw:
        handle = occ.near_grasps(f.points, f.pos, res=args.res)
        keep = ~occ.flying_pixels(f.points) | handle
        min_hits = occ.CARVE_MIN_HITS
        dropped = len(f.points) - int(keep.sum())
        print("  filter dropped %d of %d returns (%.2f%%), %d spared as handle"
              % (dropped, len(f.points), 100.0 * dropped / len(f.points),
                 int(handle.sum())))

    label, lo = occ.classify(f.points, res=args.res, keep=keep, trusted=handle,
                             min_hits=min_hits)
    label, carved = occ.carve_target(label, lo, args.res, f.pos)
    counts = dict((occ.NAME[v], int((label == v).sum())) for v in occ.NAME)
    print("  grid %s at %.0f mm: %s"
          % (tuple(label.shape), args.res * 1000,
             ", ".join("%s %d" % (k, counts[k]) for k in ("free", "obstacle", "target", "unknown"))))
    if carved == 0:
        print("  WARNING: no handle voxels carved. The poses do not land on anything the")
        print("  camera saw, which usually means --at is wrong for this capture.")

    built = field.build(label, lo, args.res, args.step)
    print("  blades sampled every %.1f mm, jaw dilated %.1f mm, arm %.1f mm"
          % (field.blade_pitch(args.res) * 1000, links[-1][1] * 1000, links[0][1] * 1000))

    cand = features.candidates(f)
    keep = features.in_reach(cand, reach_max=args.reach_max, floor_z=args.floor_z)
    print("  %d of %d candidates are within reach and above the floor"
          % (int(keep.sum()), len(cand)))

    if args.out:
        size, arm_lo, dims = field.export(args.out, built, args.at, stamp)
        hi = arm_lo + dims * built["res"]
        print("  wrote %s (%.1f MB), digest %016x" % (args.out, size / 1e6, stamp))
        print("    arm_base x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f"
              % (arm_lo[0], hi[0], arm_lo[1], hi[1], arm_lo[2], hi[2]))

    if args.candidates:
        flat = features.flatten(cand[keep])
        with open(args.candidates, "w") as fh:
            fh.write("[%s]\n" % ", ".join("%.6f" % v for v in flat))
        print("  wrote %s (%d candidates, %d floats)"
              % (args.candidates, int(keep.sum()), len(flat)))
        print("    rosservice call /task/plan \"data: $(cat %s)\"" % args.candidates)

    print("  %.2f s" % (time.time() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
