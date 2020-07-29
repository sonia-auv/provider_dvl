#!/usr/bin/env python
__author__ = "jeremie"

from sonia_common.msg import BottomTracking
import rospy


def callback(data):
    velocities = list(data.velocity)
    vx = "{:10.4f}".format(float(velocities[0]))
    vy = "{:10.4f}".format(float(data.velocity[1]))
    vz = "{:10.4f}".format(float(data.velocity[2]))
    print "Velocity:\tNorth : " + vx + "\tEast : " + vy + "\tDown : " + vz + "\r",


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("velocity_pretty_print", anonymous=True)

    rospy.Subscriber("/provider_dvl/bottom_tracking", BottomTracking, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    listener()
