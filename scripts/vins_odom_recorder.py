#!/usr/bin/env python
import rospy
import csv
import os
import time
from nav_msgs.msg import Odometry

writer = None
csvfile = None

def odom_callback(msg):
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation

    writer.writerow([
        pos.x, pos.y, pos.z,
        ori.x, ori.y, ori.z, ori.w
    ])
    csvfile.flush()

if __name__ == "__main__":
    rospy.init_node("vins_odom_recorder", anonymous=True)

    # Get bag file path from parameter
    bag_path = rospy.get_param("~bag_file", "")
    if not bag_path:
        rospy.logerr("No bag_file parameter provided!")
        rospy.signal_shutdown("Missing bag_file parameter")
        exit(1)

    # Extract bag name and add timestamp
    bag_name = os.path.basename(bag_path).replace(".bag", "")
    timestamp = int(time.time())
    csv_name = "{}_{}.csv".format(bag_name, timestamp)

    # Save CSV in same directory as bag file
    csv_file = os.path.join(os.path.dirname(bag_path), csv_name)

    # Open CSV and write header
    global csvfile, writer
    csvfile = open(csv_file, 'w')
    writer = csv.writer(csvfile)
    writer.writerow([
        'Position_x', 'Position_y', 'Position_z',
        'Orientation_x', 'Orientation_y', 'Orientation_z', 'Orientation_w'
    ])

    rospy.loginfo("Recording /vins_estimator/odometry to %s ..." % csv_file)
    rospy.Subscriber("/vins_estimator/odometry", Odometry, odom_callback)
    rospy.spin()
    csvfile.close()
