# API Documentation - WindowClean Pro Plugins

Documentation complète de l'API des plugins Gazebo pour WindowClean Pro.

## Table des Matières

- [UltrasonicSensorPlugin](#ultrasonicsensorplugin)
- [VacuumSuctionPlugin](#vacuumsuctionplugin)
- [ImuMpu6050Plugin](#imumpu6050plugin)
- [BatteryPlugin](#batteryplugin)
- [EncoderPlugin](#encoderplugin)
- [DifferentialDriveControllerPlugin](#differentialdrivecontrollerplugin)

---

## UltrasonicSensorPlugin

### Description

Plugin pour simuler un capteur ultrason HC-SR04 avec caractéristiques réalistes:
- Portée: 2-400 cm (configurable)
- Angle de détection: ~15 degrés
- Bruit gaussien simulé
- Fréquence: 10 Hz (typique HC-SR04)

### Paramètres SDF

```xml
<plugin name="ultrasonic_plugin" filename="libultrasonic_sensor_plugin.so">
  <ros>
    <namespace>/windowclean_pro</namespace>
  </ros>
  <topic_name>ultrasonic/range</topic_name>
  <frame_name>ultrasonic_link</frame_name>
  <min_range>0.02</min_range>          <!-- m -->
  <max_range>4.0</max_range>            <!-- m -->
  <noise_mean>0.0</noise_mean>          <!-- m -->
  <noise_stddev>0.01</noise_stddev>     <!-- m -->
  <update_rate>10.0</update_rate>       <!-- Hz -->
</plugin>
```

### Messages ROS2

#### Publication

**Topic:** `{topic_name}`  
**Type:** `sensor_msgs/msg/Range`

```cpp
std_msgs/Header header
  uint32 seq
  builtin_interfaces/Time stamp
  string frame_id
uint8 ULTRASOUND=0
uint8 radiation_type
float32 field_of_view        # ~15 degrés (0.261799 rad)
float32 min_range            # 0.02 m
float32 max_range            # 4.0 m
float32 range                # Distance mesurée (m)
```

### Exemple d'utilisation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UltrasonicSubscriber(Node):
    def __init__(self):
        super().__init__('ultrasonic_subscriber')
        self.subscription = self.create_subscription(
            Range,
            '/windowclean_pro/ultrasonic_front_link/range',
            self.listener_callback,
            10
        )
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Range: {msg.range:.3f} m')

rclpy.init()
node = UltrasonicSubscriber()
rclpy.spin(node)
```

---

## VacuumSuctionPlugin

### Description

Simule un système de ventouse à vide régulé avec:
- Force d'adhérence variable
- Détection de contact avec surface
- Simulation physique réaliste des forces
- Contrôle via ROS2

### Paramètres SDF

```xml
<plugin name="vacuum_plugin" filename="libvacuum_suction_plugin.so">
  <ros>
    <namespace>/windowclean_pro</namespace>
  </ros>
  <topic_name>suction/vacuum</topic_name>
  <link_name>suction_fl_link</link_name>
  <max_suction_force>100.0</max_suction_force>  <!-- N -->
  <suction_area>0.002827</suction_area>          <!-- m² (π * 0.03²) -->
  <update_rate>50.0</update_rate>                <!-- Hz -->
</plugin>
```

### Messages ROS2

#### Publication

**Topics:**
- `{topic_name}/state` → `std_msgs/msg/Bool`: État actif/inactif
- `{topic_name}/force` → `std_msgs/msg/Float64`: Force actuelle (N)

#### Abonnement

**Topics:**
- `{topic_name}/enable` → `std_msgs/msg/Bool`: Activer/désactiver
- `{topic_name}/force_target` → `std_msgs/msg/Float64`: Force cible (N)

### Exemple d'utilisation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

class VacuumController(Node):
    def __init__(self):
        super().__init__('vacuum_controller')
        self.enable_pub = self.create_publisher(
            Bool, 
            '/windowclean_pro/suction_fl_link/vacuum/enable', 
            10
        )
        self.force_pub = self.create_publisher(
            Float64,
            '/windowclean_pro/suction_fl_link/vacuum/force_target',
            10
        )
    
    def enable_vacuum(self):
        msg = Bool()
        msg.data = True
        self.enable_pub.publish(msg)
    
    def set_force_target(self, force):
        msg = Float64()
        msg.data = force  # N
        self.force_pub.publish(msg)
```

---

## ImuMpu6050Plugin

### Description

Simule un IMU MPU6050 (6 axes) avec:
- Gyroscope 3 axes
- Accéléromètre 3 axes
- Dérive (drift) simulée
- Bruit gaussien réaliste

### Paramètres SDF

```xml
<plugin name="imu_plugin" filename="libimu_mpu6050_plugin.so">
  <ros>
    <namespace>/windowclean_pro</namespace>
  </ros>
  <topic_name>imu/data</topic_name>
  <frame_name>base_link</frame_name>
  <update_rate>100.0</update_rate>              <!-- Hz -->
  <gyro_noise_stddev>0.00017</gyro_noise_stddev>  <!-- rad/s -->
  <accel_noise_stddev>0.00196</accel_noise_stddev> <!-- m/s² -->
</plugin>
```

### Messages ROS2

#### Publication

**Topic:** `{topic_name}`  
**Type:** `sensor_msgs/msg/Imu`

```cpp
std_msgs/Header header
geometry_msgs/Quaternion orientation       # Orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity     # rad/s
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration  # m/s²
float64[9] linear_acceleration_covariance
```

### Exemple d'utilisation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/windowclean_pro/imu/data',
            self.listener_callback,
            10
        )
    
    def listener_callback(self, msg):
        accel = msg.linear_acceleration
        gyro = msg.angular_velocity
        
        self.get_logger().info(
            f'Accel: ({accel.x:.2f}, {accel.y:.2f}, {accel.z:.2f}) m/s²\n'
            f'Gyro: ({math.degrees(gyro.x):.2f}, '
            f'{math.degrees(gyro.y):.2f}, '
            f'{math.degrees(gyro.z):.2f}) °/s'
        )
```

---

## BatteryPlugin

### Description

Simule une batterie Li-ion avec:
- Décharge selon courant consommé
- Modèle tension-charge réaliste
- Mise à jour périodique de l'état

### Paramètres SDF

```xml
<plugin name="battery" filename="libbattery_plugin.so">
  <ros>
    <namespace>/windowclean_pro</namespace>
  </ros>
  <topic_name>battery/state</topic_name>
  <initial_charge>100.0</initial_charge>  <!-- % -->
  <capacity>7.4</capacity>                 <!-- Ah -->
  <voltage>14.8</voltage>                  <!-- V -->
  <current_draw>2.5</current_draw>         <!-- A -->
  <update_rate>10.0</update_rate>          <!-- Hz -->
</plugin>
```

### Messages ROS2

#### Publication

**Topic:** `{topic_name}`  
**Type:** `sensor_msgs/msg/BatteryState`

```cpp
std_msgs/Header header
float32 voltage              # V
float32 temperature          # °C (non utilisé)
float32 current              # A (négatif pour décharge)
float32 charge               # Ah (non utilisé)
float32 capacity             # Ah
float32 design_capacity      # Ah
float32 percentage           # % (0-100)
uint8 POWER_SUPPLY_STATUS_UNKNOWN=0
uint8 POWER_SUPPLY_STATUS_CHARGING=1
uint8 POWER_SUPPLY_STATUS_DISCHARGING=2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING=3
uint8 POWER_SUPPLY_STATUS_FULL=4
uint8 power_supply_status
uint8 POWER_SUPPLY_HEALTH_UNKNOWN=0
uint8 power_supply_health
uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN=0
uint8 POWER_SUPPLY_TECHNOLOGY_LION=1
uint8 power_supply_technology
bool present
```

### Exemple d'utilisation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.subscription = self.create_subscription(
            BatteryState,
            '/windowclean_pro/battery/state',
            self.listener_callback,
            10
        )
    
    def listener_callback(self, msg):
        if msg.percentage < 20.0:
            self.get_logger().warn(f'Low battery: {msg.percentage:.1f}%')
        else:
            self.get_logger().info(
                f'Battery: {msg.percentage:.1f}% '
                f'({msg.voltage:.2f}V, {abs(msg.current):.2f}A)'
            )
```

---

## EncoderPlugin

### Description

Simule un encodeur incrémental avec résolution configurable.

### Paramètres SDF

```xml
<plugin name="encoder" filename="libencoder_plugin.so">
  <ros>
    <namespace>/windowclean_pro</namespace>
  </ros>
  <joint_name>left_wheel_joint</joint_name>
  <topic_name>left_wheel/encoder</topic_name>
  <pulses_per_revolution>600</pulses_per_revolution>
  <update_rate>100.0</update_rate>  <!-- Hz -->
</plugin>
```

### Messages ROS2

#### Publication

**Topic:** `{topic_name}`  
**Type:** `sensor_msgs/msg/JointState`

```cpp
std_msgs/Header header
string[] name                    # Nom du joint
float64[] position               # Position (rad)
float64[] velocity               # Vitesse (rad/s)
float64[] effort                 # Effort/couple (N⋅m) + pulses dans effort[0]
```

### Exemple d'utilisation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class EncoderSubscriber(Node):
    def __init__(self):
        super().__init__('encoder_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/windowclean_pro/left_wheel/encoder',
            self.listener_callback,
            10
        )
    
    def listener_callback(self, msg):
        if msg.position:
            position_rad = msg.position[0]
            pulses = int(msg.effort[0]) if msg.effort else 0
            self.get_logger().info(
                f'Position: {position_rad:.3f} rad '
                f'({pulses} pulses)'
            )
```

---

## DifferentialDriveControllerPlugin

### Description

Contrôleur différentiel avec odométrie pour robot à deux roues.

### Paramètres SDF

```xml
<plugin name="differential_drive_controller" 
        filename="libdifferential_drive_controller_plugin.so">
  <ros>
    <namespace>/windowclean_pro</namespace>
  </ros>
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>0.26</wheel_separation>      <!-- m -->
  <wheel_diameter>0.1</wheel_diameter>            <!-- m -->
  <max_wheel_torque>50</max_wheel_torque>         <!-- N⋅m -->
  <command_topic>cmd_vel</command_topic>
  <odometry_topic>odom</odometry_topic>
  <odometry_frame>odom</odometry_frame>
  <robot_base_frame>base_link</robot_base_frame>
  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
  <update_rate>50.0</update_rate>                 <!-- Hz -->
</plugin>
```

### Messages ROS2

#### Abonnement

**Topic:** `{command_topic}`  
**Type:** `geometry_msgs/msg/Twist`

```cpp
geometry_msgs/Vector3 linear
  float64 x    # Vitesse linéaire (m/s)
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z    # Vitesse angulaire (rad/s)
```

#### Publication

**Topic:** `{odometry_topic}`  
**Type:** `nav_msgs/msg/Odometry`

```cpp
std_msgs/Header header
  string frame_id          # "odom"
string child_frame_id      # "base_link"
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    Point position
    Quaternion orientation
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    Vector3 linear
    Vector3 angular
  float64[36] covariance
```

### Exemple d'utilisation

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_pub = self.create_publisher(
            Twist,
            '/windowclean_pro/cmd_vel',
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/windowclean_pro/odom',
            self.odom_callback,
            10
        )
    
    def move_forward(self, speed=0.2):
        cmd = Twist()
        cmd.linear.x = speed
        self.cmd_pub.publish(cmd)
    
    def rotate(self, angular_vel=0.5):
        cmd = Twist()
        cmd.angular.z = angular_vel
        self.cmd_pub.publish(cmd)
    
    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
    
    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.get_logger().info(
            f'Position: ({pose.position.x:.2f}, '
            f'{pose.position.y:.2f})'
        )
```

---

## Qualités de Service (QoS)

Pour une communication fiable en simulation, utilisez les profils QoS suivants:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Pour les capteurs (best effort)
sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT
)

# Pour les commandes (reliable)
command_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE
)
```

---

## Notes d'implémentation

1. **Thread-safety**: Les plugins sont thread-safe et peuvent être utilisés dans des environnements multi-threaded.

2. **Performance**: Les plugins sont optimisés pour une fréquence de mise à jour élevée (50-100 Hz).

3. **Bruit et erreurs**: Tous les capteurs incluent des modèles de bruit réalistes basés sur les spécifications matérielles réelles.

4. **Débogage**: Activez les logs Gazebo avec `--verbose` pour voir les messages détaillés des plugins.

---

## Support

Pour toute question sur l'API, consulter:
- Le code source dans `plugins/src/`
- Les exemples dans `scripts/`
- Les tests dans `scripts/validation/`

