#interpreteur line pour utiliser python3
#!/usr/bin/env python3  

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtleControllerNode(Node):
    
    def __init__(self):
        super().__init__("turtle_controller")
        self._parameter_suscriber_ = self.create_subscription (
            Pose,"/turtle1/pose", self.pose_callback, 10) #
        self.get_logger().info("Turtle controller has been started")


    def pose_callback (self, pose: Pose):
        pass

def main(args=None):  # Initialiser le client ROS 2
    rclpy.init(args=args)  # Initialiser le client ROS 2
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()    
