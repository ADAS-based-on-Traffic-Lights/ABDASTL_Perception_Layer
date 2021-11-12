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

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('depth_estimation')
        self.bridge = CvBridge()
        # Mergin the subscribers into a single callback
        #self.prediction_sub = message_filters.Subscriber(self, Image, "/zed/zed_node/left/image_rect_color")
        self.prediction_sub = message_filters.Subscriber(self, TLPredictions, "/detection/model/predictions")
        self.depth_sub = message_filters.Subscriber(self, Image, "/zed/zed_node/depth/depth_registered")
        self.ts = message_filters.TimeSynchronizer([self.prediction_sub, self.depth_sub], 20)
        self.ts.registerCallback(self.listener_callback)

    def listener_callback(self, prediction_msg, depth_msg):
        #Predicton Message
        header = prediction_msg.header
        classes = prediction_msg.classes
        bbs = prediction_msg.boundingboxes
       # print(header)
       # print(classes)
       # print(bbs)
        #Create a empty distance array
        depth_estimation = []
        #Depth map dimension is 360,640
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        depth_array = np.array(depth_img, dtype=np.float32)
        #Copy the depth map image and convert it to rgb to verify that the detections are well detected
        copy = depth_img.copy()
        copy = cv2.cvtColor(copy, cv2.COLOR_GRAY2RGB)
        # Display the depth image with a point in the center
        print("The distance of the center is :", depth_array[180,320])
        cv2.circle(copy, (320,180), 5, (0,0,255), -1)
        cv2.imshow("Depth Map Estimation", copy)
        cv2.waitKey(3)

        #Draw a bounding box to the traffic light detections
        ###for i,tlclass in enumerate(classes): # 1 0 3 2
        ###  cv2.rectangle(copy, (bbs[i*4+1],bbs[i*4]), (bbs[i*4+3], bbs[i*4+2]), (0,255,0), 2)
          ## Calculate the centroid of the TL
        ###  x = int((bbs[i*4+3] - bbs[i*4+1])/2 + bbs[i*4+1])
        ###  y = int((bbs[i*4+2] - bbs[i*4])/2 + bbs[i*4])
        ###  cv2.circle(copy, (x,y), 5, (0,0,255), -1)
        ###  print(depth_array[x,y])
          #depth_estimation.append(depth_array[x,y])
        #print(depth_estimation)
        #print(depth_array.shape)
        #print("---------------------------------")
        ##cv2.imshow("Depth Map Estimation", copy)
        ##cv2.waitKey(3)

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
