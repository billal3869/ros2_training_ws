#interpreteur line pour utiliser python3
#!/usr/bin/env python3  

# Importer les bibliothèques nécessaires
import rclpy

#importer la classe Node de rclpy
from rclpy.node import Node

# Importer le message Twist pour contrôler le robot
from geometry_msgs.msg import Twist

# Définir la classe Draw_circle qui hérite de Node
class Draw_circle(Node):
    def __init__(self): # Initialiser le nœud avec un nom
        super().__init__('draw_circle')  # Initialiser le nœud avec un nom
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  # Créer un éditeur pour le topic 'cmd_vel'
        self.timer = self.create_timer(0.5, self.send_velocity_command)  # Créer un timer pour envoyer des commandes de vitesse toutes les 0.5 secondes
        self.get_logger().info('Draw Circle Node has been started.')  # Log message when node starts
   
# Créer un timer qui appelle la fonction timer_callback toutes les 0.5 secondes
    def send_velocity_command(self):
        msg = Twist ()  # Créer un message Twist
        msg.linear.x = 2.0  # Vitesse linéaire en x     
        msg.angular.z = 1.0  # Vitesse angulaire en z
        self.cmd_vel_pub_.publish(msg)  # Publier le message sur le topic 'cmd_vel'

def main(args=None):  # Initialiser le client ROS 2
        rclpy.init(args=args)  # Initialiser le client ROS 2
        node = Draw_circle()  # Créer une instance de Draw_circle
        rclpy.spin(node)  # Garder le nœud en fonctionnement pour traiter les événements
        rclpy.shutdown()  # Arrêter le client ROS 2
    
if __name__ == '__main__':  # Point d'entrée du script
    main()
