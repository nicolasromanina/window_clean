#!/usr/bin/env python3
"""
Noeud ROS2 pour contrôle de navigation autonome
Implémente le pattern de nettoyage "boustrophédon"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import math


class NavigationControllerNode(Node):
    """Contrôleur de navigation autonome avec pattern boustrophédon"""
    
    def __init__(self):
        super().__init__('navigation_controller_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/windowclean_pro/cmd_vel',
            10
        )
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            '/windowclean_pro/odom',
            self.odom_callback,
            10
        )
        
        # État
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Paramètres de navigation
        self.line_spacing = 0.25  # Espacement entre les lignes (m)
        self.line_length = 0.9  # Longueur des lignes (m)
        self.current_line = 0
        self.moving_forward = True
        
        # État machine
        self.state = 'IDLE'  # IDLE, FORWARD, TURN, REVERSE
        
        # Timer pour contrôle
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('NavigationControllerNode démarré')
    
    def odom_callback(self, msg):
        """Callback odométrie"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Orientation (quaternion -> euler)
        q = msg.pose.pose.orientation
        self.theta = math.atan2(2*(q.w*q.z + q.x*q.y), 
                               1 - 2*(q.y*q.y + q.z*q.z))
    
    def control_loop(self):
        """Boucle de contrôle principale"""
        cmd = Twist()
        
        if self.state == 'FORWARD':
            cmd.linear.x = 0.2
            # Vérifier si on a atteint la fin de la ligne
            if self.x > self.line_length / 2:
                self.state = 'TURN'
        
        elif self.state == 'TURN':
            cmd.angular.z = 0.5
            # Tourner de 90 degrés
            if abs(self.theta - math.pi/2) < 0.1:
                self.state = 'REVERSE'
                self.current_line += 1
        
        elif self.state == 'REVERSE':
            cmd.linear.x = -0.2
            # Reculer de l'espacement
            if abs(self.y - self.current_line * self.line_spacing) < 0.05:
                self.state = 'TURN'
        
        elif self.state == 'IDLE':
            self.state = 'FORWARD'
            self.get_logger().info('🧹 Démarrage pattern boustrophédon')
        
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

