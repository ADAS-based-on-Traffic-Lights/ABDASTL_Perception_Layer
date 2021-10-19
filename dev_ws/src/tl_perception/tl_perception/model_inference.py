import rclpy
import sys
import os
import numpy as np
import cv2
from cv_bridge import CvBridge

import tensorflow as tf
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils

from rclpy.node import Node

from sensor_msgs.msg import Image

## Message type: sensor_msgs/msg/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image

""" std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image
                             # If the frame_id here and the frame_id of the CameraInfo
                             # message associated with the image conflict
                             # the behavior is undefined

uint32 height                # image height, that is, number of rows
uint32 width                 # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.ros.org and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.hpp

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
 """


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('model_inference')
        PATH_TO_SAVED_MODEL = "./src/tl_perception/models/EfficienDet512_Augmeted/saved_model"
        PATH_TO_LABELS = "./src/tl_perception/label_maps/bstld_label_map.pbtxt"

        # Load saved model and build the detection function
        self.detect_fn = tf.saved_model.load(PATH_TO_SAVED_MODEL)
        self.get_logger().info('The model was succesfully loaded')
        # Load the labels
        self.category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS,use_display_name=True)
        self.get_logger().info('The label maps was succesfully loaded')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/left/image_rect_color',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        #self.publisher_ = self.create_publisher(Image,'detections/image',10)

    def listener_callback(self, msg):
        # Extract only the information of the image in the ros message
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Convert the domain from BGR to RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # Convert to a numpy array
        img_np = np.array(img)
        #height, width, channels = img.shape
        input_tensor = tf.convert_to_tensor(img_np)
        input_tensor = input_tensor[tf.newaxis, ...]
        detections = self.detect_fn(input_tensor)
        num_detections = int(detections.pop('num_detections'))
        detections = {key: value[0, :num_detections].numpy()
                   for key, value in detections.items()}
        detections['num_detections'] = num_detections
        # detection_classes should be ints.
        detections['detection_classes'] = detections['detection_classes'].astype(np.int64)
        image_np_with_detections = img_np.copy()
        viz_utils.visualize_boxes_and_labels_on_image_array(
              image_np_with_detections,
              detections['detection_boxes'],
              detections['detection_classes'],
              detections['detection_scores'],
              self.category_index,
              use_normalized_coordinates=True,
              max_boxes_to_draw=200,
              min_score_thresh=.20,
              agnostic_mode=False)
        # Retrieve all the information from the confidences, BBs, and classes that are greater than the threshold
        scores = detections['detection_scores'][detections['detection_scores'] > 0.20]
        bb = detections['detection_boxes'][0:scores.size]
        classes = detections['detection_classes'][0:scores.size] 
        # Plot the TL detections found in the image 
        cv2.imshow("Traffic Light Detections", cv2.cvtColor(image_np_with_detections, cv2.COLOR_BGR2RGB)
        cv2.waitKey(3)
##        new_msg = self.bridge.cv2_to_imgmsg(np.array(img),encoding = "passthrough")
##        new_msg.header.frame_id = 'Inference'
##        new_msg.header.stamp.sec =  msg.header.stamp.sec
##        new_msg.header.stamp.nanosec = msg.header.stamp.nanosec
##        new_msg.step = msg.step
##        new_msg.encoding = 'bgra8'
##        self.publisher_.publish(new_msg)
#        self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(self.image), "rgb8")) 
        #msg.data=self.bridge.cv2_to_imgmsg(np.array(img), "bgr8")
        #self.publisher_.publish(msg)
        #self.getself.get_logger().info('The number of detections %d' %(scores.size))_logger().info('height  %d, width %d, channels %d' %(height,width,channels))
  #      self.get_logger().info('The number of detections %d' %(scores.size))
        #self.publisher_.publish(msg)

def main(args=None):

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
