#!/usr/bin/env python3
"""
Noeud ROS2 pour contrôle des micropompes péristaltiques
Gère l'activation/désactivation et le débit des pompes de nettoyage
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from std_srvs.srv import SetBool


class PumpControlNode(Node):
    """Noeud pour contrôler les micropompes péristaltiques"""
    
    def __init__(self):
        super().__init__('pump_control_node')
        
        # État des pompes
        self.pump1_enabled = False
        self.pump2_enabled = False
        self.pump1_flow_rate = 0.0  # ml/min
        self.pump2_flow_rate = 0.0
        
        # Publishers pour commande
        self.pump1_enable_pub = self.create_publisher(Bool, '/windowclean_pro/pump1/enable', 10)
        self.pump2_enable_pub = self.create_publisher(Bool, '/windowclean_pro/pump2/enable', 10)
        self.pump1_flow_pub = self.create_publisher(Float64, '/windowclean_pro/pump1/flow_rate', 10)
        self.pump2_flow_pub = self.create_publisher(Float64, '/windowclean_pro/pump2/flow_rate', 10)
        
        # Service pour activation/désactivation
        self.enable_service = self.create_service(
            SetBool,
            '/windowclean_pro/pumps/enable',
            self.enable_callback
        )
        
        # Paramètres
        self.max_flow_rate = 100.0  # ml/min max
        self.default_flow_rate = 50.0  # ml/min par défaut
        
        self.get_logger().info('PumpControlNode démarré')
    
    def enable_callback(self, request, response):
        """Service callback pour activer/désactiver les pompes"""
        self.pump1_enabled = request.data
        self.pump2_enabled = request.data
        
        # Publication
        msg = Bool()
        msg.data = request.data
        self.pump1_enable_pub.publish(msg)
        self.pump2_enable_pub.publish(msg)
        
        if request.data:
            # Activation avec débit par défaut
            self.set_flow_rate(self.default_flow_rate, self.default_flow_rate)
            self.get_logger().info('✅ Pompes activées')
        else:
            self.set_flow_rate(0.0, 0.0)
            self.get_logger().info('⏹️  Pompes désactivées')
        
        response.success = True
        return response
    
    def set_flow_rate(self, rate1, rate2):
        """Définit le débit des pompes"""
        self.pump1_flow_rate = min(max(0.0, rate1), self.max_flow_rate)
        self.pump2_flow_rate = min(max(0.0, rate2), self.max_flow_rate)
        
        msg1 = Float64()
        msg1.data = self.pump1_flow_rate
        self.pump1_flow_pub.publish(msg1)
        
        msg2 = Float64()
        msg2.data = self.pump2_flow_rate
        self.pump2_flow_pub.publish(msg2)
    
    def start_cleaning_cycle(self):
        """Démarre un cycle de nettoyage"""
        self.get_logger().info('🧹 Démarrage cycle de nettoyage')
        # Activation pompe solution nettoyante
        self.set_flow_rate(self.default_flow_rate, 0.0)
        # Puis rinçage (pompe eau)
        # Implémenter logique de cycle


def main(args=None):
    rclpy.init(args=args)
    node = PumpControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

