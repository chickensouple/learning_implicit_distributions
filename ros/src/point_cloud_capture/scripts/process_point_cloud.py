import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
import threading
import copy
import math
import numpy as np
import scipy.io

curr_cloud = None
curr_cloud_lock = threading.Lock()

def callback(data):
    global curr_cloud
    curr_cloud_lock.acquire()
    if curr_cloud == None:
        curr_cloud = copy.deepcopy(data)
    curr_cloud_lock.release()

if __name__ == '__main__':
    rospy.init_node('stitch')
    rospy.Subscriber('/camera/depth_registered/points', PointCloud2, callback)
    pub = rospy.Publisher('/processed_pc', PointCloud2, queue_size=1)

    rate = rospy.Rate(5)
    while True and not rospy.is_shutdown():
        curr_cloud_lock.acquire()
        if curr_cloud != None:
            curr_cloud_lock.release()
            break
        curr_cloud_lock.release()
        rate.sleep()

    # recieved a point cloud
    print("Recieved Point Cloud")

    # process pc
    new_points = []
    for point in sensor_msgs.point_cloud2.read_points(curr_cloud):
        if not math.isnan(point[0]) and not math.isnan(point[1]) and not math.isnan(point[2]): 
            xyz = np.array(point[:3])
            if np.linalg.norm(xyz) <= 1.3:
                new_points.append(point)

    pub_pointcloud = sensor_msgs.point_cloud2.create_cloud(curr_cloud.header, curr_cloud.fields, new_points)

    new_points_arr = np.array(new_points)
    scipy.io.savemat('pointcloud.mat', mdict={'points': new_points_arr})

    # publish processed pc
    while True and not rospy.is_shutdown():
        pub.publish(pub_pointcloud)
        print pub_pointcloud.header
        rate.sleep()

