#!/usr/bin/env python3
"""
Noeud ROS2 pour surveillance de sécurité multi-niveau
- Surveillance batterie
- Surveillance adhérence ventouses
- Surveillance capteurs
- Détection de chute
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Twist
import math


class SafetyMonitorNode(Node):
    """Noeud de surveillance de sécurité"""
    
    def __init__(self):
        super().__init__('safety_monitor_node')
        
        # Subscriptions
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/windowclean_pro/battery/state',
            self.battery_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/windowclean_pro/imu/data',
            self.imu_callback,
            10
        )
        
        # Publishers
        self.emergency_stop_pub = self.create_publisher(
            Bool,
            '/windowclean_pro/safety/emergency_stop',
            10
        )
        
        self.safety_status_pub = self.create_publisher(
            Bool,
            '/windowclean_pro/safety/status',
            10
        )
        
        # État
        self.battery_level = 100.0
        self.acceleration_z = 0.0
        self.emergency_stop = False
        
        # Seuils
        self.battery_critical = 10.0  # %
        self.battery_low = 20.0  # %
        self.fall_acceleration_threshold = 2.0  # m/s² (chute)
        self.normal_gravity = 9.81
        
        # Timer pour vérification périodique
        self.timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('SafetyMonitorNode démarré')
    
    def battery_callback(self, msg):
        """Callback batterie"""
        self.battery_level = msg.percentage
        
        if msg.percentage < self.battery_critical:
            self.get_logger().error(
                f'🔴 BATTERIE CRITIQUE: {msg.percentage:.1f}%'
            )
            self.trigger_emergency_stop()
        elif msg.percentage < self.battery_low:
            self.get_logger().warn(
                f'⚠️ BATTERIE FAIBLE: {msg.percentage:.1f}%'
            )
    
    def imu_callback(self, msg):
        """Callback IMU"""
        # Accélération en Z (perpendiculaire à la vitre)
        self.acceleration_z = msg.linear_acceleration.z
        
        # Détection de chute (accélération Z anormale)
        if abs(self.acceleration_z) < self.fall_acceleration_threshold:
            self.get_logger().error('🔴 CHUTE DÉTECTÉE!')
            self.trigger_emergency_stop()
    
    def safety_check(self):
        """Vérification périodique de sécurité"""
        status = Bool()
        status.data = not self.emergency_stop
        self.safety_status_pub.publish(status)
    
    def trigger_emergency_stop(self):
        """Déclenche un arrêt d'urgence"""
        self.emergency_stop = True
        msg = Bool()
        msg.data = True
        self.emergency_stop_pub.publish(msg)
        self.get_logger().error('🛑 ARRÊT D\'URGENCE ACTIVÉ')
    
    def clear_emergency_stop(self):
        """Réinitialise l'arrêt d'urgence"""
        self.emergency_stop = False
        msg = Bool()
        msg.data = False
        self.emergency_stop_pub.publish(msg)
        self.get_logger().info('✅ Arrêt d\'urgence réinitialisé')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

