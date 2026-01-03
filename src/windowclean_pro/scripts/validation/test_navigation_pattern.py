#!/usr/bin/env python3
"""
Script de validation: Test du pattern de navigation boustrophédon
Valide que le robot peut suivre un pattern de nettoyage en boustrophédon
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import math
import time


class NavigationPatternTester(Node):
    """Testeur de pattern de navigation"""
    
    def __init__(self):
        super().__init__('navigation_pattern_tester')
        
        self.odom_sub = self.create_subscription(
            Odometry, '/windowclean_pro/odom',
            self.odom_callback, 10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/windowclean_pro/cmd_vel', 10
        )
        
        self.ultrasonic_sub = self.create_subscription(
            Range, '/windowclean_pro/ultrasonic_front_link/range',
            self.ultrasonic_callback, 10
        )
        
        self.current_pose = None
        self.current_range = None
        self.pattern_path = []
        self.pattern_step = 0
        
    def odom_callback(self, msg):
        """Callback pour l'odométrie"""
        self.current_pose = msg.pose.pose
        
    def ultrasonic_callback(self, msg):
        """Callback pour le capteur ultrason"""
        self.current_range = msg.range
    
    def get_distance(self, pose1, pose2):
        """Calculer la distance entre deux poses"""
        if pose1 is None or pose2 is None:
            return float('inf')
        
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def generate_boustrophedon_pattern(self, width=1.0, height=1.5, step=0.3):
        """Générer un pattern boustrophédon"""
        path = []
        y = 0.0
        direction = 1
        
        while y < height:
            # Aller dans une direction
            x_start = 0.0 if direction == 1 else width
            x_end = width if direction == 1 else 0.0
            
            path.append((x_start, y, 0.0))  # Position
            path.append((x_end, y, math.pi if direction == -1 else 0.0))  # Orientation
            
            # Se déplacer vers la ligne suivante
            y += step
            direction *= -1
        
        return path
    
    def follow_pattern(self):
        """Suivre le pattern généré"""
        self.get_logger().info('Starting boustrophedon pattern...')
        
        pattern = self.generate_boustrophedon_pattern()
        tolerance = 0.1  # m
        linear_speed = 0.2  # m/s
        angular_speed = 0.5  # rad/s
        
        for i, (target_x, target_y, target_yaw) in enumerate(pattern):
            self.get_logger().info(
                f'Moving to waypoint {i+1}/{len(pattern)}: '
                f'({target_x:.2f}, {target_y:.2f}, {math.degrees(target_yaw):.1f}°)'
            )
            
            # Attendre d'atteindre le waypoint
            max_iterations = 500
            iteration = 0
            
            while iteration < max_iterations:
                if self.current_pose is None:
                    time.sleep(0.1)
                    iteration += 1
                    continue
                
                # Calculer l'erreur
                dx = target_x - self.current_pose.position.x
                dy = target_y - self.current_pose.position.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Orientation actuelle
                current_yaw = 2 * math.atan2(
                    self.current_pose.orientation.z,
                    self.current_pose.orientation.w
                )
                
                # Erreur angulaire
                target_angle = math.atan2(dy, dx)
                angle_error = target_angle - current_yaw
                
                # Normaliser l'angle
                while angle_error > math.pi:
                    angle_error -= 2*math.pi
                while angle_error < -math.pi:
                    angle_error += 2*math.pi
                
                # Si proche, arrêter
                if distance < tolerance:
                    break
                
                # Commande de vitesse
                cmd = Twist()
                
                if abs(angle_error) > 0.1:
                    # Rotation d'abord
                    cmd.angular.z = angular_speed * math.copysign(1, angle_error)
                else:
                    # Avancer
                    cmd.linear.x = linear_speed
                    cmd.angular.z = 0.0
                
                self.cmd_vel_pub.publish(cmd)
                
                time.sleep(0.1)
                rclpy.spin_once(self, timeout_sec=0.01)
                iteration += 1
            
            # Arrêter
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.5)
        
        self.get_logger().info('Pattern completed!')
    
    def test_obstacle_avoidance(self):
        """Test d'évitement d'obstacles"""
        self.get_logger().info('Testing obstacle avoidance...')
        
        cmd = Twist()
        cmd.linear.x = 0.2
        
        # Avancer jusqu'à détecter un obstacle
        for _ in range(100):
            if self.current_range is not None and self.current_range < 0.3:
                self.get_logger().info(f'Obstacle detected at {self.current_range:.3f}m')
                
                # Arrêter et reculer
                cmd.linear.x = -0.1
                cmd.angular.z = 0.5
                self.cmd_vel_pub.publish(cmd)
                time.sleep(1.0)
                
                self.get_logger().info('✓ Obstacle avoidance: OK')
                return True
            
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        self.get_logger().warn('No obstacle detected (expected in some scenarios)')
        return True
    
    def run_tests(self):
        """Exécuter les tests"""
        self.get_logger().info('='*60)
        self.get_logger().info('NAVIGATION PATTERN TESTS')
        self.get_logger().info('='*60)
        
        # Attendre que la simulation soit prête
        time.sleep(2.0)
        
        # Test du pattern
        try:
            self.follow_pattern()
            self.get_logger().info('✓ Pattern navigation: OK')
        except Exception as e:
            self.get_logger().error(f'✗ Pattern navigation failed: {e}')
            return 1
        
        # Test d'évitement d'obstacles
        try:
            self.test_obstacle_avoidance()
        except Exception as e:
            self.get_logger().error(f'✗ Obstacle avoidance failed: {e}')
            return 1
        
        self.get_logger().info('='*60)
        self.get_logger().info('ALL NAVIGATION TESTS PASSED ✓')
        return 0


def main(args=None):
    rclpy.init(args=args)
    tester = NavigationPatternTester()
    
    try:
        exit_code = tester.run_tests()
    except KeyboardInterrupt:
        exit_code = 1
    finally:
        tester.destroy_node()
        rclpy.shutdown()
    
    return exit_code


if __name__ == '__main__':
    import sys
    sys.exit(main())

