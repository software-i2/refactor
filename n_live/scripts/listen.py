#!/usr/bin/env python
# Copyright by BeeX [2026]
#
# Melodic only ships numpy for python2.7, so this must NOT say python3.

"""Pair clouds with grasp poses by header stamp and drop each pair into the inbox."""

from __future__ import print_function

import itertools
import os

import rospy
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2

from n_live import topics


def main():
    rospy.init_node("n_live_listen")
    folder = os.path.abspath(rospy.get_param("~inbox"))
    cloud_topic = rospy.get_param("~cloud", "/pointcloud")
    poses_topic = rospy.get_param("~poses", "/grasp_poses")
    if not os.path.isdir(folder):
        os.makedirs(folder)

    pairer = topics.Pairer()
    count = itertools.count()

    def take(pair):
        if pair is None:
            return
        cloud, poses = pair
        fid = "%05d-%06d" % (next(count), cloud.header.seq)
        try:
            n = topics.put(folder, fid, cloud, poses)
        except Exception as e:
            rospy.logerr("[listen] %s not written: %s", fid, e)
            return
        rospy.loginfo("[listen] %s: %d points, %d poses", fid, n, len(poses.poses))

    rospy.Subscriber(cloud_topic, PointCloud2, lambda m: take(pairer.add(cloud=m)),
                     queue_size=1, buff_size=1 << 25)
    rospy.Subscriber(poses_topic, PoseArray, lambda m: take(pairer.add(poses=m)),
                     queue_size=1, buff_size=1 << 22)
    rospy.loginfo("[listen] pairing %s with %s by header stamp, into %s",
                  cloud_topic, poses_topic, folder)
    rospy.spin()


if __name__ == "__main__":
    main()
