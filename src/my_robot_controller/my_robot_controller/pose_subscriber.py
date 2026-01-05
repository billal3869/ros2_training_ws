# This is a Python script for subscribing to pose messages in a ROS2 environment.

#interpreteur line pour utiliser python3
#!/usr/bin/env python3  

# Importer les bibliothèques nécessaires
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose 


class PoseSubscriberNode(Node):

    def __init__(self):
        super().__init__('pose_subscriber') # Initialize the node with a name
        self.pose_subscriber_= self.create_subscription( Pose,'/turtle1/pose', self.pose_callback, 10) # Create a subscription to the 'Pose' topic


# Callback function to handle incoming pose messages

    def pose_callback(self, msg: Pose): 
        self.get_logger().info("(" + str(msg.x) + "," + str(msg.y) + ")")
                               
        
def main(args=None):  # Initialiser le client ROS 2
    rclpy.init(args=args)  # Initialiser le client ROS 2
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()    

