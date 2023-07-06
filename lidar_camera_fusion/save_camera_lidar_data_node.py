def gprint(*args, **kwargs):
    print('\033[92m', *args, '\033[0m', **kwargs)
def yprint(*args, **kwargs):
    print('\033[93m', *args, '\033[0m', **kwargs)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np 
from cv_bridge import CvBridge

import matplotlib.pyplot as plt

rotation = np.array([[-0.01535590583, 0.001793002054, 0.9998804835],
                    [-0.9998095223, -0.01207531679, -0.01533316237],
                    [0.0120463812, -0.9999254832, 0.001978087955]])

translation = np.array([0.1935187898, -0.09712105286, -0.1787771099])

class PointCloud2Listener(Node):
    def __init__(self):
        super().__init__('pointcloud2_listener')
        self.camera_count = 0
        self.lidar_count = 0

        self.bridge = CvBridge()
        self.camera_subscription = self.create_subscription(
            Image,
            'camera',
            self.camera_callback,
            10)
        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            'velodyne_points',  
            self.lidar_callback,
            10)

        #cv_image
        self.cv_image = None
        self.lidar_points=None
        print("READY")

    def save_data(self):
        if self.cv_image is None or self.lidar_points is None:
            return #do nothing

        np.save(f'{filename}.npy', [self.cv_image, self.lidar_points])

    def camera_callback(self, msg):
        self.camera_count+=1
        timestamp = msg.header.stamp

        self.cv_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

    def lidar_callback(self, msg):
        self.lidar_count+=1
        timestamp = msg.header.stamp
        
        # Deserialize PointCloud2 data into xyz points
        point_gen = pc2.read_points(
            msg, field_names=(
                "x", "y", "z"), skip_nans=True)
        # points = np.array(list(point_gen))
        points = [[x, y, z] for x, y, z in point_gen]
        self.lidar_points = np.array(points)
        self.save_data()
        return


        return 


        points = np.dot(points, rotation.T) + translation
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(points[:, 0], points[:, 1], points[:, 2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # Save the plot as an image
        plt.savefig(f'{filename}.jpg')


rclpy.init()
def main(args=None):
    pointcloud2_listener = PointCloud2Listener()
    rclpy.spin(pointcloud2_listener)
    pointcloud2_listener.destroy_node()
    rclpy.shutdown()


import subprocess, threading, os
def run_shell_command(command):
    with open(os.devnull, 'w') as nullfile:
        process = subprocess.Popen(command, shell=True, stdout=nullfile, stderr=subprocess.STDOUT)
        process.communicate()


command1 = "ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/camera/compressed --remap out:=/camera"
command2 = "ros2 bag play bag0"

if __name__ == '__main__':

    # Create threads for each shell command and main function
    thread1 = threading.Thread(target=run_shell_command, args=(command1,))
    thread2 = threading.Thread(target=run_shell_command, args=(command2,))
    thread_main = threading.Thread(target=main)

    thread1.start()
    thread2.start()
    thread_main.start()

    # Wait for the threads to finish
    thread1.join()
    thread2.join()
    thread_main.join()