import rclpy
import sys
import os
import numpy as np
import cv2

from cv_bridge import CvBridge

from rclpy.node import Node

from sensor_msgs.msg import Image
from tl_interfaces.msg import TLPredictions

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('depth_estimation')
        self.bridge = CvBridge()
        # Subscribers
        ## Prediction Image subscription
        #self.subscription = self.create_subscription(
        #    Image,
        #    '/detections/image',
        #    self.listener_callback,
        #    10)
        #self.subscription  # prevent unused variable warning
        ## Depth Map Image subscription
        self.depth_sub = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_estimation_callback,
            10)
        self.depth_sub
        ## Predictions BBs and classes subscription
        #self.prediction_sub = self.create_subscription(
        #    TLPredictions,
        #    'detection/model/predictions',
        #    self.listener_callback,
        #    10)
        #self.prediction_sub

    #def listener_callback(self, msg):
        #print(msg)
        #img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #print(img)
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        #height, width, channels = img.shape
        #print(height,width)
        #cv2.imshow("Traffic Light Detections", cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        #cv2.waitKey(3)

    def depth_estimation_callback(self, msg):
        # depth map dimension is 360,640
        img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        cv2.imshow("Depth Map Estimation", img)
        cv2.waitKey(3)

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

