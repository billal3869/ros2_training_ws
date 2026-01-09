#interpreteur line pour utiliser python3
#!/usr/bin/env python3  

import cmd
import rclpy
from rclpy.node import Node # Import the Node class from rclpy
from turtlesim.msg import Pose # Import the Pose message type
from geometry_msgs.msg import Twist # Import the Twist message type
from turtlesim.srv import SetPen # Import the SetPen service type
from functools import partial

class TurtleControllerNode(Node):
    
    def __init__(self):
        super().__init__("turtle_controller") # Initialize the node with a name
        self.previous_x = 0.0 # Variable to store the previous x position
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)  # Publisher for command velocity
        self.pose_subscriber_= self.create_subscription( 
            Pose,'/turtle1/pose', self.pose_callback, 10) # Create a subscription to the 'Pose' topic

        self.get_logger().info("Turtle controller has been started") # Log message indicating the node has started


    def pose_callback(self, pose: Pose):
        cmd = Twist()  # Create a new Twist message
        if pose.x >= 9.0 or pose.x <= 2.0 or pose.y >= 9.0 or pose.y <= 2.0:  # Check if the turtle's x or y position is greater than or equal to 9.0   
            cmd.linear.x = 1.0  # Set linear velocity to 1.0 
            cmd.angular.z = 0.9  # Set angular velocity to  0.9 
        else:  
            cmd.linear.x = 5.0  # Set linear velocity
            cmd.angular.z = 0.0  # Set angular velocity
        self.cmd_vel_publisher_.publish(cmd)  # Publish the command velocity

        #condition pour changer la couleur du stylo lorsque la tortue atteint une certaine position, rouge a partir de x=5.5
        if pose.x > 5.5 and self.previous_x <= 5.5: # Si la position x actuelle est supérieure à 5.5 et la position précédente était inférieure ou égale à 5.5
            self.previous_x = pose.x # Mettre à jour la position précédente
            self.get_logger().info("Changing pen color to red")
            self.call_set_pen_service(255, 0, 0, 3, 0)  # Appeler le service SetPen pour changer la couleur du stylo en rouge
        elif pose.x <= 5.5 and self.previous_x > 5.5: # Si la position x actuelle est inférieure ou égale à 5.5 et la position précédente était supérieure à 5.5
            self.previous_x = pose.x
            self.get_logger().info("Changing pen color to GREEN")
            self.call_set_pen_service(0, 255, 0, 3, 0)  # Appeler le service SetPen pour changer la couleur du stylo en noir et désactiver le stylo

    def call_set_pen_service(self, r, g, b, width, off): # Méthode pour appeler le service SetPen
        client = self.create_client(SetPen, '/turtle1/set_pen') # Créer un client de service pour SetPen
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')
        request = SetPen.Request() # Créer une requête de service
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        
        future = client.call_async(request) # Appeler le service de manière asynchrone
        future.add_done_callback(partial(self.calback_set_pen)) # Ajouter une méthode de rappel pour gérer la réponse

    def calback_set_pen(self, future): # Méthode de rappel pour le service SetPen
        try:
            response = future.result() # Obtenir la réponse du service
            self.get_logger().info('SetPen service called successfully') # Log message indicating success
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,)) # Log message indicating failure

def main(args=None):  # Initialiser le client ROS 2
    rclpy.init(args=args)  # Initialiser le client ROS 2
    node = TurtleControllerNode() # Créer une instance du nœud
    rclpy.spin(node) # Garder le nœud en fonctionnement jusqu'à interruption
    rclpy.shutdown() # Arrêter proprement le client ROS 2
