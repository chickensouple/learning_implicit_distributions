#!/usr/bin/env python 
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
import msgpack
import zmq
import threading
import copy
import struct
import numpy as np
import math
import signal

curr_head_angle = None
curr_head_angle_mutex = threading.Lock()

neck_transform = np.array([[1., 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.085], [0, 0, 0, 1]])

curr_cloud = None
curr_cloud_lock = threading.Lock()

def signal_term_handler(signal, frame):
    print 'got SIGTERM'
    sys.exit(0)

def yaw_rot(yaw):
    rot = np.zeros((4, 4))

    rot[3, 3] = 1
    rot[1, 1] = 1
    rot[0, 0] = math.cos(yaw)
    rot[0, 2] = math.sin(yaw)
    rot[2, 0] = -math.sin(yaw)
    rot[2, 2] = math.cos(yaw)

    return rot

def roll_rot(roll):
    rot = np.zeros((4, 4))

    rot[3, 3] = 1
    rot[0, 0] = 1
    rot[1, 1] = math.cos(roll)
    rot[1, 2] = math.sin(roll)
    rot[2, 1] = -math.sin(roll)
    rot[2, 2] = math.cos(roll)
    return rot


def callback(data):
    global curr_cloud
    curr_cloud_lock.acquire()
    curr_cloud = copy.deepcopy(data)
    curr_cloud_lock.release()

def head_angle_reciever():
    global curr_head_angle
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect('tcp://192.168.123.146:43354')
    socket.setsockopt(zmq.SUBSCRIBE, '')
    while True:
        data = socket.recv()
        message = msgpack.unpackb(data)

        curr_head_angle_mutex.acquire()
        curr_head_angle = copy.deepcopy(message[1])
        curr_head_angle_mutex.release()
        # print('Received reply: ' + str(message))


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, signal_term_handler)
    print("Starting stitch.py")
    rospy.init_node('stitch')
    rospy.Subscriber('/camera/depth_registered/points', PointCloud2, callback)
    pub = rospy.Publisher('/all_pc', PointCloud2, queue_size=1)

    head_angle_thread = threading.Thread(target=head_angle_reciever)
    head_angle_thread.start()

    rate = rospy.Rate(5)

    all_points = []

    while True:
        curr_cloud_lock.acquire()
        curr_cloud_copy = copy.deepcopy(curr_cloud)
        curr_cloud_lock.release()

        if curr_cloud_copy == None:
            continue
        print("Adding New Points")

        points = []
        for point in sensor_msgs.point_cloud2.read_points(curr_cloud_copy):
            if not math.isnan(point[0]) and not math.isnan(point[1]) and not math.isnan(point[2]): 
                points.append(point)

        points = np.array(points)

        xyz = points[:, 0:3]
        xyz = np.concatenate((xyz, np.ones((xyz.shape[0], 1))), axis=1)

        curr_head_angle_mutex.acquire()
        curr_head_angle_copy = copy.deepcopy(curr_head_angle)
        curr_head_angle_mutex.release()

        if curr_head_angle_copy == None:
            continue

        rot_mat = roll_rot(curr_head_angle_copy)
        camera_transform = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1.]])

        transform = np.dot(rot_mat, np.dot(neck_transform, camera_transform))
        # transform = np.dot(rot_mat, camera_transform)
        # transform = camera_transform

        xyz = np.dot(transform, xyz.T).T
        points = np.concatenate((xyz[:, 0:3], np.reshape(points[:, -1], (-1, 1))), axis=1)

        all_points.extend(points.tolist())
        # all_points = points.tolist()

        pub_pc = sensor_msgs.point_cloud2.create_cloud(curr_cloud_copy.header, curr_cloud_copy.fields, all_points)

        pub.publish(pub_pc)
        rate.sleep()

    rospy.spin()


# for getting head angles
# context = zmq.Context()
# socket = context.socket(zmq.SUB)
# socket.connect('tcp://192.168.123.146:43354')
# socket.setsockopt(zmq.SUBSCRIBE, '')

# while True:
#     data = socket.recv()
#     message = msgpack.unpackb(data)
#     # message = data
#     print('Received reply: ' + str(message))



