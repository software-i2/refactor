#!/usr/bin/env python
# Copyright by BeeX [2026]

"""Keep a copy of every frame n_task planned from, and draw it on its own topics."""

from __future__ import print_function

import os

import rospy
from std_msgs.msg import String

from n_live import build, draw, store

PREFIX = "viz/planned/"


def show(board, folder):
    doc, r = build.rebuild(folder)
    board.show(os.path.join(folder, build.FIELD), r)
    rospy.loginfo("[planned] %s on %s*: %d candidates in reach, digest %s", folder, PREFIX,
                  int(r["keep"].sum()), doc["digest"])


def main():
    rospy.init_node("n_live_planned")
    board = draw.Board(prefix=PREFIX)

    only = rospy.get_param("~frame", "")
    if only:
        show(board, os.path.realpath(only))
        rospy.spin()
        return

    out = os.path.abspath(rospy.get_param("~out"))
    newest = max(1, int(rospy.get_param("~keep", 20)))
    if not os.path.isdir(out):
        os.makedirs(out)

    def take(msg):
        if not os.path.isdir(msg.data):
            rospy.logerr("[planned] %s was pruned before it could be kept; raise n_live's keep",
                         msg.data)
            return
        try:
            kept = store.keep(msg.data, out)
        except Exception as e:
            rospy.logerr("[planned] %s not kept: %s", msg.data, e)
            return
        store.prune(out, newest)
        try:
            show(board, kept)
        except Exception as e:
            rospy.logwarn("[planned] %s kept but not drawn: %s", kept, e)

    rospy.Subscriber("task/planned_frame", String, take, queue_size=1)
    rospy.loginfo("[planned] keeping the newest %d frames n_task planned from in %s", newest, out)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
