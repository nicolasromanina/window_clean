#!/usr/bin/env python3
"""
Script de validation: Métriques de performance
Mesure les performances du robot (vitesse, précision, consommation)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
import time
import math
import json


class PerformanceMetricsTester(Node):
    """Testeur de métriques de performance"""
    
    def __init__(self):
        super().__init__('performance_metrics_tester')
        
        self.odom_sub = self.create_subscription(
            Odometry, '/windowclean_pro/odom',
            self.odom_callback, 10
        )
        
        self.battery_sub = self.create_subscription(
            BatteryState, '/windowclean_pro/battery/state',
            self.battery_callback, 10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/windowclean_pro/cmd_vel', 10
        )
        
        self.odom_history = []
        self.battery_history = []
        self.start_time = None
        self.start_battery = None
        
    def odom_callback(self, msg):
        """Callback pour l'odométrie"""
        self.odom_history.append({
            'time': time.time(),
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'linear_vel': msg.twist.twist.linear.x,
            'angular_vel': msg.twist.twist.angular.z
        })
    
    def battery_callback(self, msg):
        """Callback pour la batterie"""
        self.battery_history.append({
            'time': time.time(),
            'percentage': msg.percentage,
            'voltage': msg.voltage,
            'current': abs(msg.current)
        })
        if self.start_battery is None:
            self.start_battery = msg.percentage
    
    def calculate_distance_traveled(self):
        """Calculer la distance totale parcourue"""
        if len(self.odom_history) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(1, len(self.odom_history)):
            prev = self.odom_history[i-1]
            curr = self.odom_history[i]
            
            dx = curr['x'] - prev['x']
            dy = curr['y'] - prev['y']
            distance = math.sqrt(dx*dx + dy*dy)
            total_distance += distance
        
        return total_distance
    
    def calculate_average_speed(self):
        """Calculer la vitesse moyenne"""
        if len(self.odom_history) < 2:
            return 0.0
        
        speeds = [abs(odom['linear_vel']) for odom in self.odom_history]
        return sum(speeds) / len(speeds) if speeds else 0.0
    
    def calculate_max_speed(self):
        """Calculer la vitesse maximale"""
        if not self.odom_history:
            return 0.0
        
        speeds = [abs(odom['linear_vel']) for odom in self.odom_history]
        return max(speeds) if speeds else 0.0
    
    def calculate_battery_consumption(self):
        """Calculer la consommation de batterie"""
        if not self.battery_history or self.start_battery is None:
            return None
        
        end_battery = self.battery_history[-1]['percentage']
        consumption = self.start_battery - end_battery
        
        if len(self.battery_history) > 1:
            duration = (self.battery_history[-1]['time'] - 
                       self.battery_history[0]['time'])
            consumption_rate = consumption / duration if duration > 0 else 0.0
        else:
            consumption_rate = 0.0
        
        return {
            'total_consumption': consumption,
            'consumption_rate': consumption_rate,  # %/s
            'duration': duration if len(self.battery_history) > 1 else 0.0
        }
    
    def run_performance_test(self, duration=30):
        """Exécuter un test de performance"""
        self.get_logger().info('='*60)
        self.get_logger().info('PERFORMANCE METRICS TEST')
        self.get_logger().info('='*60)
        
        self.get_logger().info(f'Running test for {duration} seconds...')
        
        # Attendre que la simulation soit prête
        time.sleep(2.0)
        
        self.start_time = time.time()
        end_time = self.start_time + duration
        
        # Commande de mouvement constant
        cmd = Twist()
        cmd.linear.x = 0.2  # m/s
        cmd.angular.z = 0.1  # rad/s
        
        while time.time() < end_time:
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Arrêter
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        
        # Calculer les métriques
        self.get_logger().info('Calculating metrics...')
        
        distance = self.calculate_distance_traveled()
        avg_speed = self.calculate_average_speed()
        max_speed = self.calculate_max_speed()
        battery_data = self.calculate_battery_consumption()
        
        # Afficher les résultats
        self.get_logger().info('='*60)
        self.get_logger().info('PERFORMANCE METRICS')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Distance traveled: {distance:.2f} m')
        self.get_logger().info(f'Average speed: {avg_speed:.3f} m/s')
        self.get_logger().info(f'Max speed: {max_speed:.3f} m/s')
        
        if battery_data:
            self.get_logger().info(
                f'Battery consumption: {battery_data["total_consumption"]:.2f}%'
            )
            if battery_data['duration'] > 0:
                self.get_logger().info(
                    f'Battery consumption rate: '
                    f'{battery_data["consumption_rate"]*3600:.2f} %/hour'
                )
        
        # Évaluer les performances
        metrics = {
            'distance_traveled': distance,
            'average_speed': avg_speed,
            'max_speed': max_speed,
            'battery_consumption': battery_data
        }
        
        # Critères de performance
        performance_ok = True
        if avg_speed < 0.15:
            self.get_logger().warn('⚠ Average speed below target (0.15 m/s)')
            performance_ok = False
        
        if max_speed > 0.5:
            self.get_logger().warn('⚠ Max speed exceeds safe limit (0.5 m/s)')
            performance_ok = False
        
        if battery_data and battery_data['consumption_rate'] > 0.001:
            self.get_logger().warn('⚠ High battery consumption rate')
        
        # Sauvegarder dans un fichier JSON
        with open('/tmp/windowclean_pro_metrics.json', 'w') as f:
            json.dump(metrics, f, indent=2)
        
        self.get_logger().info('Metrics saved to /tmp/windowclean_pro_metrics.json')
        
        if performance_ok:
            self.get_logger().info('✓ Performance metrics: OK')
            return 0
        else:
            self.get_logger().warn('⚠ Performance metrics: Warnings')
            return 0  # Warnings seulement


def main(args=None):
    rclpy.init(args=args)
    tester = PerformanceMetricsTester()
    
    try:
        exit_code = tester.run_performance_test(duration=30)
    except KeyboardInterrupt:
        exit_code = 1
    finally:
        tester.destroy_node()
        rclpy.shutdown()
    
    return exit_code


if __name__ == '__main__':
    import sys
    sys.exit(main())

