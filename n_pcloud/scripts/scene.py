#!/usr/bin/env python
# Copyright by BeeX [2026]
#
# Melodic only ships numpy/scipy for python2.7, so this must NOT say python3.

"""Build an obstacle field and export grasp candidates for one scene."""

from __future__ import print_function

import argparse
import os
import sys
import time

import numpy as np

from n_pcloud import field, frame, pipeline
from n_pcloud import occupancy as occ
from n_pcloud.field_format import read_header


def triple(flag, shape):
    def parse(text):
        parts = text.replace(",", " ").split()
        if len(parts) != 3:
            raise argparse.ArgumentTypeError(
                "%s needs three numbers, %s, got %r" % (flag, shape, text))
        return [float(v) for v in parts]
    return parse


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
    ap.add_argument("--at", type=triple("--at", '"X Y Z" in metres'), required=True,
                    metavar='"X Y Z"', help="camera origin in arm_base, metres. Required.")
    ap.add_argument("--rpy", type=triple("--rpy", '"R P Y" in degrees'), default=[0.0, 0.0, 0.0],
                    metavar='"R P Y"',
                    help="extra camera rotation about arm_base x, y, z, degrees, applied "
                         "after the fixed optical-to-arm swap and pivoting on --at")
    ap.add_argument("--res", type=float, default=0.0025, help="working voxel size")
    ap.add_argument("--step", type=float, default=0.002, help="axis sample spacing n_check walks")
    ap.add_argument("--reach-max", type=float, default=0.35, dest="reach_max",
                    help="generous screen on how far the arm can get; n_check owns the verdict")
    ap.add_argument("--floor-z", type=float, default=0.0, dest="floor_z",
                    help="screen out candidates below this; n_check owns the verdict")
    ap.add_argument("--raw", action="store_true",
                    help="skip all noise filtering, for comparison")
    ap.add_argument("--full", action="store_true",
                    help="keep the whole camera volume instead of cropping to the arm's reach")
    ap.add_argument("--out", help="write the field here")
    ap.add_argument("--candidates", help="write the flat candidate list here, one per line")
    ap.add_argument("--check", help="report whether this field still matches these inputs")
    args = ap.parse_args()

    ply_path, json_path = scene_paths(args)
    name = args.scene or os.path.basename(ply_path)
    box = None if args.full else field.REACH

    if args.check:
        stamp = pipeline.stamp(ply_path, json_path, args.at, args.rpy, args.res, args.step,
                               args.raw, box)
        head = read_header(args.check)
        print("%s" % args.check)
        print("  built from  %016x" % head["digest"])
        print("  these input %016x  (placed at %s, rpy %s)"
              % (stamp, np.round(args.at, 3), np.round(args.rpy, 2)))
        if head["digest"] == stamp:
            print("  FRESH")
            return 0
        print("  STALE: the arm would check a world that no longer matches the inputs.")
        print("  placed at %s, rpy %s when it was built"
              % (np.round(head["xyz"], 3), np.round(head["rpy"], 2)))
        return 3

    t0 = time.time()
    clock = pipeline.Clock()
    stamp = pipeline.stamp(ply_path, json_path, args.at, args.rpy, args.res, args.step,
                           args.raw, box)
    clock.lap("digest")
    f = frame.read(name, ply_path, json_path, args.at, args.rpy)
    clock.lap("read")
    r = pipeline.run(f, stamp, args.res, args.step, args.reach_max, args.floor_z, args.raw,
                     box, clock)
    pipeline.save(r, args.out, args.candidates)

    print("%s" % name)
    print("  %s" % ply_path)
    print("  %s" % json_path)
    print("  %d points, %d poses, camera at %s rpy %s in arm_base"
          % (len(f.points), len(f.pos), np.round(f.at, 3), np.round(f.rpy, 2)))

    if not args.raw:
        print("  filter dropped %d of %d returns (%.2f%%), %d spared as handle"
              % (r["dropped"], len(f.points), 100.0 * r["dropped"] / len(f.points),
                 r["spared"]))

    counts = r["counts"]
    print("  grid %s at %.0f mm: %s"
          % (r["grid"], args.res * 1000,
             ", ".join("%s %d" % (k, counts[k]) for k in ("free", "obstacle", "target", "unknown"))))
    print("  %d handle voxels carved, %d more cleared in the %.0f mm approach corridors"
          % (r["carved"], r["corridor"], occ.CORRIDOR_LENGTH * 1000))
    if r["carved"] == 0:
        print("  WARNING: no handle voxels carved. The poses do not land on anything the")
        print("  camera saw, which usually means --at is wrong for this capture.")

    if r["placed"] is not None:
        print("  resampled into arm_base for rpy %s: grid %s" % (np.round(args.rpy, 2),
                                                               r["placed"]))
    if r["cropped"] is not None:
        print("  cropped to the arm's reach: grid %s" % (r["cropped"],))

    built = r["field"]
    links = built["links"]
    print("  blades sampled every %.1f mm, jaw dilated %.1f mm, arm %.1f mm"
          % (field.blade_pitch(args.res) * 1000, links[-1][1] * 1000, links[0][1] * 1000))

    keep = r["keep"]
    print("  %d of %d candidates are within reach and above the floor"
          % (int(keep.sum()), len(r["cand"])))

    if args.out:
        hi = r["arm_lo"] + r["dims"] * built["res"]
        print("  wrote %s (%.1f MB), digest %016x" % (args.out, r["size"] / 1e6, r["stamp"]))
        print("    arm_base x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f"
              % (r["arm_lo"][0], hi[0], r["arm_lo"][1], hi[1], r["arm_lo"][2], hi[2]))

    if args.candidates:
        print("  wrote %s (%d candidates, %d floats)"
              % (args.candidates, int(keep.sum()), r["floats"]))
        print("    rosservice call /task/plan \"data: $(cat %s)\"" % args.candidates)

    print("  %s" % "  ".join("%s %.2f" % t for t in r["times"]))
    print("  %.2f s" % (time.time() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
