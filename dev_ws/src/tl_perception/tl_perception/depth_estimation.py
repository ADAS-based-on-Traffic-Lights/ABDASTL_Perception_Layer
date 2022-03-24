import rclpy
import sys
import os
import numpy as np
import cv2

from cv_bridge import CvBridge
import message_filters

from rclpy.node import Node

from sensor_msgs.msg import Image
from tl_interfaces.msg import TLPredictions
from tl_interfaces.msg import ClassDistanceTL

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('depth_estimation')
        # Instance of Bridge for img to message conversion and visa versa
        self.bridge = CvBridge()
        # Merging the subscribers into a single callback
        self.prediction_sub = message_filters.Subscriber(self, TLPredictions, "/detection/model/predictions")
        self.depth_sub = message_filters.Subscriber(self, Image, "/zed/zed_node/depth/depth_registered")
        self.ts = message_filters.TimeSynchronizer([self.prediction_sub, self.depth_sub], 20)
        self.ts.registerCallback(self.listener_callback)
        # Publisher
        self.publisher_ = self.create_publisher(ClassDistanceTL,"estimation/classes_distances",20)
        self.get_logger().info('Depth Estimation System loaded')

    def listener_callback(self, prediction_msg, depth_msg):
        # Retrieve TLPredicton Message
        header = prediction_msg.header
        classes = prediction_msg.classes
        bbs = prediction_msg.boundingboxes

        #Create a empty distance array
        depth_estimation = []

        #Convert the depth_msg to depth map in meters
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        depth_array = np.array(depth_img, dtype=np.float32)

        #Draw a bounding box to the traffic light detections
        for i,tlclass in enumerate(classes):
          # Calculate the centroid of the TL
          ###x = int((bbs[i*4+3] - bbs[i*4+1])/2 + bbs[i*4+1])
          ###y = int((bbs[i*4+2] - bbs[i*4])/2 + bbs[i*4])
          ###depth_estimation.append(depth_array[y,x])

          # Calculate the median of all the values of the TL area
          bbs_values = np.copy(depth_array[bbs[i*4]:bbs[i*4+2],bbs[i*4+1]:bbs[i*4+3]])
          print(bbs_values.shape)
          bbs_values = bbs_values[~np.isinf(bbs_values)]
          bbs_values = bbs_values[~np.isnan(bbs_values)]
          print(bbs_values)
          median = np.median(bbs_values)
          depth_estimation.append(median)


        # Convert it to a np array
        depth_estimation = np.array(depth_estimation)
        classes = np.array(classes)

        # Sort the tl distance based on the distance
        indexes = np.argsort(depth_estimation)
        depth_estimation = depth_estimation[indexes]
        classes = classes[indexes]

        # Create a ClassDistanceTL Message
        new_CDTL = ClassDistanceTL()
        new_CDTL.header = header
        new_CDTL.classes = classes.tolist()
        new_CDTL.distances = depth_estimation.tolist()
        # Publish the ClassDistanceTL Message
        self.publisher_.publish(new_CDTL)

def main(args=None):
    # ROS
    rclpy.init(args=args)
    model_inference = MinimalSubscriber()

    rclpy.spin(model_inference)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    model_inference.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
