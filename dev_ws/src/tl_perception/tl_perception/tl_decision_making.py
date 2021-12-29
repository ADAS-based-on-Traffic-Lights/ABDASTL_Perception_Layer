import rclpy

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

from rclpy.node import Node

from tl_interfaces.msg import ClassDistanceTL

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('tl_decision_making')
        
        # Define the Membership Functions 
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
        
        # Define the Fuzzy Logic Rules
        rule1 = ctrl.Rule(self.TL_State['Green'] & self.Distance['Close'], self.Brake_Signal['No_Brake'])
        rule2 = ctrl.Rule(self.TL_State['Green'] & self.Distance['Normal'], self.Brake_Signal['No_Brake'])
        rule3 = ctrl.Rule(self.TL_State['Green'] & self.Distance['Far'], self.Brake_Signal['No_Brake'])
        rule4 = ctrl.Rule(self.TL_State['Off'] & self.Distance['Far'], self.Brake_Signal['No_Brake'])
        rule5 = ctrl.Rule(self.TL_State['Yellow'] & self.Distance['Far'], self.Brake_Signal['No_Brake'])
        rule6 = ctrl.Rule(self.TL_State['Red'] & self.Distance['Far'], self.Brake_Signal['No_Brake'])
        rule7 = ctrl.Rule(self.TL_State['Off'] & self.Distance['Normal'], self.Brake_Signal['Moderate_Brake'])
        rule8 = ctrl.Rule(self.TL_State['Yellow'] & self.Distance['Normal'], self.Brake_Signal['Moderate_Brake'])
        rule9 = ctrl.Rule(self.TL_State['Red'] & self.Distance['Normal'], self.Brake_Signal['Moderate_Brake'])
        rule10 = ctrl.Rule(self.TL_State['Off'] & self.Distance['Close'], self.Brake_Signal['Full_Brake'])
        rule11 = ctrl.Rule(self.TL_State['Yellow'] & self.Distance['Close'], self.Brake_Signal['Full_Brake'])
        rule12 = ctrl.Rule(self.TL_State['Red'] & self.Distance['Close'], self.Brake_Signal['Full_Brake'])
        
        # Create the control system with the defined Fuzzy Rules
        self.TL_Decision_Making = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6,
                                                 rule7, rule8, rule9, rule10, rule11, rule12])
        
        # In order to simulate this control system, we will create a ``ControlSystemSimulation`` which represent the fuzzy logic system
        self.inference = ctrl.ControlSystemSimulation(self.TL_Decision_Making)

        # Information Message (Membership functions loaded)
        self.get_logger().info('The Fuzzy Logic System loaded')

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
        
        # Check that we have at least one detection
        if len(classes) != 0:
           ## Provide the Distance and TL_State of the closest TL
           self.inference.input['TL_State'] = classes[0]
           self.inference.input['Distance'] = distances[0]
           ## Compute the output with the given values
           self.inference.compute()
           ## Result of the Fuzzy Logic Systen
           print (self.inference.output['Brake_Signal'])
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


