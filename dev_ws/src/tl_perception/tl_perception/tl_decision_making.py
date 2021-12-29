import rclpy

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

from rclpy.node import Node

from tl_interfaces.msg import ClassDistanceTL

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('tl_decision_making')
        
        # New Antecedent/Consequent objects hold universe variables and membership
        ## Inputs
        ### Traffic Light State
        self.TL_State = ctrl.Antecedent(np.arange(0, 5, 0.1), 'TL_State')
        self.TL_State['Off'] = fuzz.trapmf(self.TL_State.universe, [0.75, 0.75, 1.25, 1.25])
        self.TL_State['Green'] = fuzz.trapmf(self.TL_State.universe, [1.75, 1.75, 2.25, 2.25])
        self.TL_State['Yellow'] = fuzz.trapmf(self.TL_State.universe, [2.75, 2.75, 3.25, 3.25])
        self.TL_State['Red'] = fuzz.trapmf(self.TL_State.universe, [3.75, 3.75, 4.25, 4.25])

        ### Distance to the Traffic Light
        self.Distance = ctrl.Antecedent(np.arange(0, 16, 1), 'Distance')
        self.Distance['Close'] = fuzz.trapmf(self.Distance.universe, [0, 0, 5, 8])
        self.Distance['Normal'] = fuzz.trapmf(self.Distance.universe, [5, 8, 11, 13])
        self.Distance['Far'] = fuzz.trapmf(self.Distance.universe, [11, 13, 15, 15])

        ## Ouput
        self.Brake_Signal = ctrl.Consequent(np.arange(0, 1.1, 0.1), 'Brake_Signal')
        self.Brake_Signal['No_Brake'] = fuzz.trapmf(self.Brake_Signal.universe, [0, 0, 0.1, 0.2])
        self.Brake_Signal['Moderate_Brake'] = fuzz.trimf(self.Brake_Signal.universe, [0.2, 0.3, 0.5])
        self.Brake_Signal['Full_Brake'] = fuzz.trapmf(self.Brake_Signal.universe, [0.5, 0.8, 1, 1])
        
        # Information Message (Membership functions loaded)
        self.get_logger().info('The membership functions have been succesfuly created')

        # Subscribe to the tl classes and distances provided by the depth_estimation node
        self.cd_tl_sub = self.create_subscription(ClassDistanceTL, 'estimation/classes_distances', self.listener_callback, 10)
        self.cd_tl_sub

        # Publisher
        # self.publisher_ = self.create_publisher(ClassDistanceTL,"estimation/classes_distances",20)

    def listener_callback(self, msg):
        # Retrieve ClassDistanceTL Message
        header = msg.header
        classes = np.array(msg.classes)
        distances = np.array(msg.distances)

        print(classes)
        print(distances)
        print("------------")

def main(args=None):
    # ROS
    rclpy.init(args=args)
    tl_decision_making = MinimalSubscriber()

    rclpy.spin(tl_decision_making)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tl_decision_making.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


