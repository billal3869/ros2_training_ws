#interpreteur line pour utiliser python3
#!/usr/bin/env python3  
# Importer les bibliothèques nécessaires
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('first_node')  # Initialiser le nœud avec un nom
        self.count_publisher_ = 0  # Initialiser un compteur
        # Créer un timer qui appelle la fonction timer_callback toutes les secondes
        self.timer = self.create_timer(1.0, self.timer_callback)

    #creation de la fonction timer  
    def timer_callback(self):
        self.get_logger().info('Hello ' + str(self.count_publisher_))  # Afficher un message à chaque appel du timer
        self.count_publisher_ += 1  # Incrémenter le compteur


def main(args=None):
    rclpy.init(args=args)  # Initialiser le client ROS 2
    node = MyNode()  # Créer une instance de MyNode
    rclpy.spin(node)  # Garder le nœud en fonctionnement pour traiter les événements
    rclpy.shutdown()  # Arrêter le client ROS 2


if __name__ == '__main__':
    main()

        