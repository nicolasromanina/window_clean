#!/usr/bin/env python3
"""
Noeud ROS2 pour interface des capteurs ultrason
Agrège les données des 7 capteurs HC-SR04 et publie des informations utiles
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import math


class UltrasonicSensorNode(Node):
    """Noeud pour gérer les capteurs ultrason du robot"""
    
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        
        # Subscriptions pour chaque capteur
        self.subscriptions = {}
        self.ranges = {}
        
        # Liste des capteurs
        sensors = [
            'ultrasonic_front',
            'ultrasonic_back',
            'ultrasonic_left',
            'ultrasonic_right',
            'ultrasonic_thickness_1',
            'ultrasonic_thickness_2',
            'ultrasonic_obstacle'
        ]
        
        for sensor in sensors:
            topic = f'/windowclean_pro/{sensor}/range'
            self.subscriptions[sensor] = self.create_subscription(
                Range,
                topic,
                lambda msg, s=sensor: self.range_callback(s, msg),
                10
            )
            self.ranges[sensor] = 4.0  # Valeur par défaut (max range)
        
        # Publisher pour commande d'évitement
        self.safety_cmd_pub = self.create_publisher(
            Twist,
            '/windowclean_pro/safety/cmd_vel',
            10
        )
        
        # Paramètres de sécurité
        self.min_safe_distance = 0.05  # 5 cm
        self.edge_detection_threshold = 0.15  # 15 cm pour détection de bord
        
        self.get_logger().info('UltrasonicSensorNode démarré')
    
    def range_callback(self, sensor_name, msg):
        """Callback pour mise à jour des distances"""
        self.ranges[sensor_name] = msg.range
        
        # Vérification de sécurité
        self.check_safety()
    
    def check_safety(self):
        """Vérifie les conditions de sécurité et publie commande si nécessaire"""
        # Vérification des capteurs de bord
        edge_sensors = ['ultrasonic_front', 'ultrasonic_back', 
                       'ultrasonic_left', 'ultrasonic_right']
        
        for sensor in edge_sensors:
            if self.ranges[sensor] < self.edge_detection_threshold:
                self.get_logger().warn(
                    f'⚠️ BORD DÉTECTÉ par {sensor}: {self.ranges[sensor]:.3f}m'
                )
                # Commande d'arrêt d'urgence
                cmd = Twist()
                self.safety_cmd_pub.publish(cmd)
                return
        
        # Vérification obstacle
        if self.ranges['ultrasonic_obstacle'] < self.min_safe_distance:
            self.get_logger().warn(
                f'⚠️ OBSTACLE PROCHE: {self.ranges["ultrasonic_obstacle"]:.3f}m'
            )
            cmd = Twist()
            self.safety_cmd_pub.publish(cmd)
    
    def get_edge_distances(self):
        """Retourne les distances aux bords"""
        return {
            'front': self.ranges['ultrasonic_front'],
            'back': self.ranges['ultrasonic_back'],
            'left': self.ranges['ultrasonic_left'],
            'right': self.ranges['ultrasonic_right']
        }
    
    def get_glass_thickness(self):
        """Estime l'épaisseur de la vitre"""
        # Utilise les deux capteurs d'épaisseur
        t1 = self.ranges['ultrasonic_thickness_1']
        t2 = self.ranges['ultrasonic_thickness_2']
        return (t1 + t2) / 2.0


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

