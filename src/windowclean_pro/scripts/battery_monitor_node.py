#!/usr/bin/env python3
"""
Noeud ROS2 pour surveillance de la batterie
Affiche l'état de la batterie et alerte en cas de problème
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BatteryMonitorNode(Node):
    """Noeud de surveillance de la batterie"""
    
    def __init__(self):
        super().__init__('battery_monitor_node')
        
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/windowclean_pro/battery/state',
            self.battery_callback,
            10
        )
        
        self.last_log_time = 0.0
        
        self.get_logger().info('BatteryMonitorNode démarré')
    
    def battery_callback(self, msg):
        """Callback batterie avec logging périodique"""
        import time
        current_time = time.time()
        
        # Log toutes les 10 secondes
        if current_time - self.last_log_time > 10.0:
            self.get_logger().info(
                f'🔋 Batterie: {msg.percentage:.1f}% | '
                f'Voltage: {msg.voltage:.2f}V | '
                f'Courant: {msg.current:.2f}A'
            )
            self.last_log_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

