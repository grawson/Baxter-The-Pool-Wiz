#!/usr/bin/env python

import roslib
roslib.load_manifest('pool_planner')
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import time

###################################################
# centroid.py
#   This package reads in a point cloud and
#   publishes the centroid.
###################################################

POINT_CLOUD_TOPIC = "/camera/depth/points"

class Centroid:

    def __init__(self):
        self.sub = rospy.Subscriber(POINT_CLOUD_TOPIC, PointCloud2, self.get_centroid)

    def get_centroid(self, data):
        rospy.loginfo(rospy.get_caller_id() + " %s", type(data.data))
        # print('here i am!', type(data.data[0]))

        dtype_list = [(f.name, np.float32) for f in data.fields]
        cloud_arr = np.fromstring(data.data, dtype_list).astype(np.float)
        # print(cloud_arr.shape)
        # print(data.height, data.width)

        # cloud_arr = cloud_arr.astype(np.float)
        # x_mean = np.nanmean(cloud_arr)
        # y_mean = np.nanmean(cloud_arr, axis=1)
        # z_mean = np.nanmean(cloud_arr, axis=2)
        # print(x_mean)
        # print('\n\n', cloud_arr.size, cloud_arr[500:505])

        time.sleep(5)

def main():
    rospy.loginfo("Calling centroid node...")
    centroid = Centroid()
    rospy.init_node('centroid', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
