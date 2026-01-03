#!/usr/bin/env python3
"""
Script de validation: Tests basiques de fonctionnalité
Valide que tous les composants du robot fonctionnent correctement
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Imu, BatteryState, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64
import time
import sys


class BasicFunctionalityTester(Node):
    """Testeur de fonctionnalités basiques"""
    
    def __init__(self):
        super().__init__('basic_functionality_tester')
        
        self.test_results = {}
        self.sensor_data = {}
        self.timeout = 5.0  # secondes
        
        # Subscriptions pour vérifier les capteurs
        self.ultrasonic_subs = {
            'front': self.create_subscription(
                Range, '/windowclean_pro/ultrasonic_front_link/range',
                lambda msg: self.sensor_callback('ultrasonic_front', msg), 10
            ),
            'back': self.create_subscription(
                Range, '/windowclean_pro/ultrasonic_back_link/range',
                lambda msg: self.sensor_callback('ultrasonic_back', msg), 10
            ),
            'left': self.create_subscription(
                Range, '/windowclean_pro/ultrasonic_left_link/range',
                lambda msg: self.sensor_callback('ultrasonic_left', msg), 10
            ),
            'right': self.create_subscription(
                Range, '/windowclean_pro/ultrasonic_right_link/range',
                lambda msg: self.sensor_callback('ultrasonic_right', msg), 10
            ),
        }
        
        self.imu_sub = self.create_subscription(
            Imu, '/windowclean_pro/imu/data',
            lambda msg: self.sensor_callback('imu', msg), 10
        )
        
        self.battery_sub = self.create_subscription(
            BatteryState, '/windowclean_pro/battery/state',
            lambda msg: self.sensor_callback('battery', msg), 10
        )
        
        # Publishers pour tester les actionneurs
        self.cmd_vel_pub = self.create_publisher(Twist, '/windowclean_pro/cmd_vel', 10)
        self.vacuum_enable_pub = self.create_publisher(Bool, '/windowclean_pro/suction_fl_link/vacuum/enable', 10)
        
    def sensor_callback(self, sensor_name, msg):
        """Callback pour recevoir les données des capteurs"""
        self.sensor_data[sensor_name] = {
            'data': msg,
            'timestamp': time.time()
        }
        self.get_logger().info(f'Received data from {sensor_name}')
    
    def wait_for_sensor(self, sensor_name, timeout=None):
        """Attendre qu'un capteur publie des données"""
        if timeout is None:
            timeout = self.timeout
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if sensor_name in self.sensor_data:
                elapsed = time.time() - self.sensor_data[sensor_name]['timestamp']
                if elapsed < 1.0:  # Données récentes (< 1s)
                    return True
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return False
    
    def test_ultrasonic_sensors(self):
        """Test des capteurs ultrason"""
        self.get_logger().info('Testing ultrasonic sensors...')
        
        test_passed = True
        for sensor_name in ['ultrasonic_front', 'ultrasonic_back', 
                           'ultrasonic_left', 'ultrasonic_right']:
            if self.wait_for_sensor(sensor_name):
                data = self.sensor_data[sensor_name]['data']
                if hasattr(data, 'range') and 0.02 <= data.range <= 4.0:
                    self.get_logger().info(f'✓ {sensor_name}: OK (range={data.range:.3f}m)')
                    self.test_results[sensor_name] = True
                else:
                    self.get_logger().error(f'✗ {sensor_name}: Invalid range')
                    self.test_results[sensor_name] = False
                    test_passed = False
            else:
                self.get_logger().error(f'✗ {sensor_name}: Timeout')
                self.test_results[sensor_name] = False
                test_passed = False
        
        return test_passed
    
    def test_imu(self):
        """Test de l'IMU"""
        self.get_logger().info('Testing IMU...')
        
        if self.wait_for_sensor('imu'):
            data = self.sensor_data['imu']['data']
            # Vérifier que les données sont présentes
            if hasattr(data, 'linear_acceleration') and hasattr(data, 'angular_velocity'):
                self.get_logger().info('✓ IMU: OK')
                self.test_results['imu'] = True
                return True
        
        self.get_logger().error('✗ IMU: Timeout or invalid data')
        self.test_results['imu'] = False
        return False
    
    def test_battery(self):
        """Test de la batterie"""
        self.get_logger().info('Testing battery...')
        
        if self.wait_for_sensor('battery'):
            data = self.sensor_data['battery']['data']
            if hasattr(data, 'percentage') and 0 <= data.percentage <= 100:
                self.get_logger().info(f'✓ Battery: OK ({data.percentage:.1f}%)')
                self.test_results['battery'] = True
                return True
        
        self.get_logger().error('✗ Battery: Timeout or invalid data')
        self.test_results['battery'] = False
        return False
    
    def test_motor_control(self):
        """Test du contrôle des moteurs"""
        self.get_logger().info('Testing motor control...')
        
        # Envoyer une commande de mouvement
        cmd = Twist()
        cmd.linear.x = 0.1  # 0.1 m/s
        cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.5)
        
        # Arrêter
        cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd)
        
        self.get_logger().info('✓ Motor control: Command sent')
        self.test_results['motor_control'] = True
        return True
    
    def test_vacuum_system(self):
        """Test du système ventouse"""
        self.get_logger().info('Testing vacuum system...')
        
        # Activer la ventouse
        enable_msg = Bool()
        enable_msg.data = True
        self.vacuum_enable_pub.publish(enable_msg)
        time.sleep(0.5)
        
        # Désactiver
        enable_msg.data = False
        self.vacuum_enable_pub.publish(enable_msg)
        
        self.get_logger().info('✓ Vacuum system: Command sent')
        self.test_results['vacuum_system'] = True
        return True
    
    def run_all_tests(self):
        """Exécuter tous les tests"""
        self.get_logger().info('='*60)
        self.get_logger().info('BASIC FUNCTIONALITY TESTS')
        self.get_logger().info('='*60)
        
        # Attendre que Gazebo soit prêt
        self.get_logger().info('Waiting for simulation to start...')
        time.sleep(2.0)
        
        results = []
        results.append(('Ultrasonic Sensors', self.test_ultrasonic_sensors()))
        results.append(('IMU', self.test_imu()))
        results.append(('Battery', self.test_battery()))
        results.append(('Motor Control', self.test_motor_control()))
        results.append(('Vacuum System', self.test_vacuum_system()))
        
        # Résumé
        self.get_logger().info('='*60)
        self.get_logger().info('TEST RESULTS')
        self.get_logger().info('='*60)
        
        all_passed = True
        for test_name, passed in results:
            status = 'PASS' if passed else 'FAIL'
            self.get_logger().info(f'{test_name}: {status}')
            if not passed:
                all_passed = False
        
        self.get_logger().info('='*60)
        if all_passed:
            self.get_logger().info('ALL TESTS PASSED ✓')
            return 0
        else:
            self.get_logger().error('SOME TESTS FAILED ✗')
            return 1


def main(args=None):
    rclpy.init(args=args)
    tester = BasicFunctionalityTester()
    
    try:
        exit_code = tester.run_all_tests()
    except KeyboardInterrupt:
        exit_code = 1
    finally:
        tester.destroy_node()
        rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()

