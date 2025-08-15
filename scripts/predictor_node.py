#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import joblib
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import os

# Path to models folder (update if needed)
pkg_path = os.path.dirname(os.path.realpath(__file__)) + '/../models/'

# Load pre-trained ML model and scaler
model = joblib.load(os.path.join(pkg_path, 'activity_rf_model.pkl'))
scaler = joblib.load(os.path.join(pkg_path, 'activity_scaler.pkl'))

# Define activity ID → Name mapping (adjust as per your dataset)
activity_map = {
    1: "Walking",
    2: "Running",
    3: "Jumping",
    4: "Sitting and Standing",
    5: "Climbing Stairs",
    6: "Descending Stairs",
    7: "Falling Forward",
    8: "Falling Backward"
}

# ROS Publisher for predicted activity
pred_pub = rospy.Publisher('/activity_prediction', String, queue_size=10)

def odometry_callback(msg):
    # Extract position
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y
    pos_z = msg.pose.pose.position.z

    # Extract orientation
    ori_x = msg.pose.pose.orientation.x
    ori_y = msg.pose.pose.orientation.y
    ori_z = msg.pose.pose.orientation.z
    ori_w = msg.pose.pose.orientation.w

    # Prepare feature vector
    features = np.array([[pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w]])

    # Scale features
    features_scaled = scaler.transform(features)

    # Predict activity
    activity_id = model.predict(features_scaled)[0]

    # Map ID to name (fallback to ID if name missing)
    if activity_id in activity_map:
        activity_name = activity_map[activity_id]
    else:
        activity_name = "Unknown Activity ({})".format(activity_id)

    # Publish prediction
    pred_msg = "Predicted Activity: {} (ID: {})".format(activity_name, activity_id)
    rospy.loginfo(pred_msg)
    pred_pub.publish(pred_msg)

def main():
    rospy.init_node('vio_activity_predictor', anonymous=True)
    rospy.Subscriber('/vins_estimator/odometry', Odometry, odometry_callback)
    rospy.loginfo("VIO Activity Predictor Node Started. Listening to /vins_estimator/odometry...")
    rospy.spin()

if __name__ == '__main__':
    main()