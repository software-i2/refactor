#!/usr/bin/env python
# Copyright by BeeX [2026]
#
# Melodic only ships numpy/scipy for python2.7, so this must NOT say python3.

"""Build an obstacle field for the newest capture in the inbox, over and over."""

from __future__ import print_function

import os
import shutil
import time

import rospy
import yaml
from std_msgs.msg import String

from n_live import build, draw, inbox, store


def camera(path):
    with open(path) as fh:
        cam = yaml.safe_load(fh)["camera"]
    at, rpy = cam["at_m"], cam["rpy_deg"]
    if len(at) != 3 or len(rpy) != 3:
        raise ValueError("%s: camera.at_m and camera.rpy_deg need three numbers each" % path)
    return [float(v) for v in at], [float(v) for v in rpy]


def main():
    rospy.init_node("n_live")
    config = rospy.get_param("~config")
    at, rpy = camera(config)
    p = build.Params(at, rpy,
                     res=rospy.get_param("~res", 0.005),
                     step=rospy.get_param("~step", 0.002),
                     reach_max=rospy.get_param("~reach_max", 0.35),
                     floor_z=rospy.get_param("~floor_z", 0.0),
                     full=rospy.get_param("~full", False))
    src = os.path.abspath(rospy.get_param("~inbox"))
    out = os.path.abspath(rospy.get_param("~out"))
    keep = max(1, int(rospy.get_param("~keep", 5)))
    board = draw.Board() if rospy.get_param("~draw", True) else None

    for path in (src, out):
        if not os.path.isdir(path):
            os.makedirs(path)
    store.clear(out)

    pub = rospy.Publisher("live/frame", String, queue_size=1, latch=True)
    rospy.loginfo("[live] watching %s, keeping the newest %d in %s", src, keep, out)
    rospy.loginfo("[live] camera at %s rpy %s from %s, %s", p.at, p.rpy, config,
                  "whole camera volume" if p.box is None else "cropped to the arm's reach")
    if not any(p.at):
        rospy.logwarn("[live] the camera is placed at 0 0 0, the arm base; set camera.at_m in %s",
                      config)

    idle = rospy.Rate(10)
    while not rospy.is_shutdown():
        found = inbox.newest(src)
        if found is None:
            idle.sleep()
            continue
        fid, ply, js, stale, skipped = found
        for path in stale:
            inbox.drop(path)

        folder = store.stage(out, fid)
        try:
            doc, r = build.build(fid, ply, js, folder, p)
        except Exception as e:
            shutil.rmtree(folder, ignore_errors=True)
            inbox.drop(ply)
            inbox.drop(js)
            rospy.logerr("[live] %s dropped: %s", fid, e)
            continue

        final = store.commit(out, fid, folder)
        store.prune(out, keep)
        pub.publish(String(data=final))
        if board is not None:
            try:
                board.show(os.path.join(final, build.FIELD), r)
            except Exception as e:
                rospy.logwarn("[live] %s built but not drawn: %s", fid, e)

        rospy.loginfo("[live] %s: %d of %d candidates in reach, field %.1f MB, ready %.1f s after "
                      "it arrived%s", fid, doc["candidates"], doc["poses"], doc["field_mb"],
                      time.time() - doc["arrived"],
                      ", %d older skipped" % skipped if skipped else "")
        rospy.loginfo("[live]   %s = %.2f s",
                      "  ".join("%s %.2f" % (s, t) for s, t in doc["times"]), doc["seconds"])


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
