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
        # Mergin the subscribers into a single callback
        #self.prediction_sub = message_filters.Subscriber(self, Image, "/zed/zed_node/left/image_rect_color")
        self.prediction_sub = message_filters.Subscriber(self, TLPredictions, "/detection/model/predictions")
        self.depth_sub = message_filters.Subscriber(self, Image, "/zed/zed_node/depth/depth_registered")
        self.ts = message_filters.TimeSynchronizer([self.prediction_sub, self.depth_sub], 20)
        self.ts.registerCallback(self.listener_callback)
        # Publisher
        # self.publisher_= self.create_publisher(Image,'detections/image/depth_map',10)
        self.publisher_ = self.create_publisher(ClassDistanceTL,"estimation/classes_distances",20)

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

        # Test 1: Display the depth image with a point in the center
        #print("The distance of the center is :", depth_array[180,320])
        #cv2.circle(copy, (320,180), 5, (0,0,255), -1)
        #cv2.imshow("Depth Map Estimation", copy)
        #cv2.waitKey(3)

        # Test 2: Copy the depth map image and convert it to rgb to verify that the detections are well detected
        #depth_img_rgb = depth_img.copy()
        #depth_img_rgb = cv2.cvtColor(depth_img_rgb, cv2.COLOR_GRAY2RGB)
        #norm = np.linalg.norm(depth_img_rgb)
        #depth_img_rgb = (depth_img_rgb/norm)*255.0
        #height, width, channels = depth_img_rgb.shape

        #Draw a bounding box to the traffic light detections
        for i,tlclass in enumerate(classes): # 1 0 3 2
          #cv2.rectangle(depth_img_rgb, (bbs[i*4+1],bbs[i*4]), (bbs[i*4+3], bbs[i*4+2]), (0,255,0), 2)
          # Calculate the centroid of the TL
          x = int((bbs[i*4+3] - bbs[i*4+1])/2 + bbs[i*4+1])
          y = int((bbs[i*4+2] - bbs[i*4])/2 + bbs[i*4])
          #cv2.circle(depth_img_rgb, (x,y), 5, (0,0,255), -1)
          #print(depth_array[y,x])
          depth_estimation.append(depth_array[y,x])
        # Convert it to a np array
        depth_estimation = np.array(depth_estimation)
        classes = np.array(classes)
        # Create the Image Message
        #new_image = self.bridge.cv2_to_imgmsg(cv2.cvtColor(np.array(depth_img_rgb, dtype=np.uint8), cv2.COLOR_RGB2GRAY), "mono8")
        #new_image.header = prediction_msg.header
        #new_image.height = height
        #new_image.width = width
        #new_image.encoding = "mono8"
        #new_image.is_bigendian = 0
        #new_image.step = 0
        # Publish the Image Message
        #self.publisher_.publish(new_image)

        # Create a ClassDistanceTL Message
        print("Before")
        print(classes.tolist())
        print(depth_estimation.tolist())
        print("-----")
        indexes = np.argsort(depth_estimation)
        depth_estimation = depth_estimation[indexes]
        classes = classes[indexes]
        print("After")
        print(classes.tolist())
        print(depth_estimation.tolist())
        print("-----")
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
