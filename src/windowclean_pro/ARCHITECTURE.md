# Architecture du Projet WindowClean Pro

Documentation technique de l'architecture du projet.

## Vue d'Ensemble

Le projet WindowClean Pro est organisé selon une architecture modulaire permettant la simulation complète d'un robot nettoyeur de vitres dans Gazebo avec ROS2.

## Structure des Répertoires

```
windowclean_pro/
├── urdf/                          # Modèles URDF/Xacro
│   └── windowclean_pro.urdf.xacro  # Modèle principal (modulaire)
│
├── plugins/                       # Plugins Gazebo C++
│   ├── src/                       # Sources C++17
│   │   ├── ultrasonic_sensor_plugin.cpp
│   │   ├── vacuum_suction_plugin.cpp
│   │   ├── imv_mpu6050_plugin.cpp
│   │   ├── battery_plugin.cpp
│   │   ├── encoder_plugin.cpp
│   │   └── differential_drive_controller_plugin.cpp
│   └── include/                   # Headers
│       ├── ultrasonic_sensor_plugin.h
│       ├── vacuum_suction_plugin.h
│       └── imv_mpu6050_plugin.h
│
├── launch/                        # Launch files ROS2
│   ├── simple_window.launch.py    # Vitre simple 1×2m
│   ├── apartment.launch.py        # Appartement 5 vitres
│   ├── office.launch.py           # Bureau open-space
│   └── complex.launch.py          # Environnement complexe
│
├── worlds/                        # Environnements Gazebo
│   ├── simple_window.world
│   ├── apartment.world
│   ├── office.world
│   └── complex.world
│
├── scripts/                       # Scripts Python ROS2
│   ├── validation/                # Tests automatisés
│   │   ├── test_basic_functionality.py
│   │   ├── test_navigation_pattern.py
│   │   └── test_performance_metrics.py
│   ├── ultrasonic_sensor_node.py
│   ├── pump_control_node.py
│   ├── cleaning_arm_node.py
│   ├── safety_monitor_node.py
│   ├── navigation_controller_node.py
│   └── battery_monitor_node.py
│
├── config/                        # Fichiers de configuration
├── meshes/                        # Modèles 3D (optionnel)
├── docs/                          # Documentation
│   └── API.md                     # Documentation API complète
│
├── CMakeLists.txt                 # Configuration de build
├── package.xml                    # Métadonnées ROS2
├── README.md                      # Documentation principale
├── QUICKSTART.md                  # Guide de démarrage rapide
└── ARCHITECTURE.md                # Ce fichier
```

## Architecture des Plugins

### 1. UltrasonicSensorPlugin

**Type**: SensorPlugin  
**Objectif**: Simuler un capteur ultrason HC-SR04

**Architecture**:
```
RaySensor (Gazebo)
    ↓
UltrasonicSensorPlugin
    ↓
Ajout bruit + limitations HC-SR04
    ↓
sensor_msgs/Range (ROS2)
```

**Caractéristiques**:
- Bruit gaussien avec stddev configurable
- Limitations réalistes (min/max range)
- Résolution limitée (~3mm) près des limites

### 2. VacuumSuctionPlugin

**Type**: ModelPlugin  
**Objectif**: Simuler système ventouse avec forces physiques

**Architecture**:
```
Link (Ventouse)
    ↓
VacuumSuctionPlugin
    ↓
Calcul force = (P_atm - P_vide) × Surface × Efficacité
    ↓
Application force + friction
    ↓
Link::AddForce()
```

**Caractéristiques**:
- Force d'adhérence variable
- Détection de contact (simplifiée)
- Friction élevée simulée
- Contrôle via ROS2 topics

### 3. ImuMpu6050Plugin

**Type**: SensorPlugin  
**Objectif**: Simuler IMU MPU6050 avec dérive

**Architecture**:
```
ImuSensor (Gazebo)
    ↓
ImuMpu6050Plugin
    ↓
Ajout bruit + dérive (drift)
    ↓
sensor_msgs/Imu (ROS2)
```

**Caractéristiques**:
- Bruit gaussien réaliste
- Dérive gyroscopique (random walk)
- Bias qui évolue dans le temps

### 4. BatteryPlugin

**Type**: ModelPlugin  
**Objectif**: Simuler batterie Li-ion

**Architecture**:
```
World Update (chaque frame)
    ↓
BatteryPlugin::OnUpdate()
    ↓
Calcul décharge = I × V × dt / (Capacité × V)
    ↓
Mise à jour %charge + tension
    ↓
sensor_msgs/BatteryState (ROS2)
```

**Caractéristiques**:
- Modèle de décharge réaliste
- Tension dépend de charge
- Courant configurable

### 5. EncoderPlugin

**Type**: ModelPlugin  
**Objectif**: Simuler encodeur incrémental

**Architecture**:
```
Joint (Roue)
    ↓
EncoderPlugin
    ↓
Suivi position angulaire
    ↓
Conversion angle → pulses (600 PPR)
    ↓
sensor_msgs/JointState (ROS2)
```

### 6. DifferentialDriveControllerPlugin

**Type**: ModelPlugin  
**Objectif**: Contrôle différentiel + odométrie

**Architecture**:
```
geometry_msgs/Twist (cmd_vel)
    ↓
DifferentialDriveControllerPlugin
    ↓
Calcul vitesses roues:
  v_left = v_linear - (ω × wheel_sep/2)
  v_right = v_linear + (ω × wheel_sep/2)
    ↓
SetVelocity() sur joints
    ↓
Calcul odométrie (intégration)
    ↓
nav_msgs/Odometry (ROS2)
```

## Architecture ROS2

### Namespace Principal

Tous les topics sont sous `/windowclean_pro/`

### Topics Principaux

```
/windowclean_pro/
├── cmd_vel              # geometry_msgs/Twist
├── odom                 # nav_msgs/Odometry
│
├── imu/
│   └── data            # sensor_msgs/Imu
│
├── battery/
│   └── state           # sensor_msgs/BatteryState
│
├── ultrasonic_*_link/
│   └── range           # sensor_msgs/Range
│
├── suction_*_link/
│   └── vacuum/
│       ├── enable      # std_msgs/Bool
│       ├── state       # std_msgs/Bool
│       ├── force       # std_msgs/Float64
│       └── force_target # std_msgs/Float64
│
└── */encoder            # sensor_msgs/JointState
```

### Qualités de Service (QoS)

- **Capteurs**: BEST_EFFORT (données peuvent être perdues)
- **Commandes**: RELIABLE (garantie de livraison)

## Flux de Données

### Boucle de Navigation

```
Capteurs (Ultrasonic, IMU, Encoders)
    ↓
Noeuds ROS2 (navigation_controller_node.py)
    ↓
Planification trajectoire
    ↓
cmd_vel (geometry_msgs/Twist)
    ↓
DifferentialDriveControllerPlugin
    ↓
Moteurs (Joints)
    ↓
Odométrie
    ↓
Feedback (loop)
```

### Boucle de Sécurité

```
Capteurs (Ultrasonic bord, Pression)
    ↓
safety_monitor_node.py
    ↓
Détection anomalie
    ↓
Commande arrêt d'urgence
    ↓
cmd_vel = 0
```

## Physique Simulation

### Modèle de Ventouse

Force d'adhérence calculée selon:

```
F = (P_atmosphérique - P_vide) × Surface × Efficacité

Où:
- P_atmosphérique = 101325 Pa
- P_vide = négatif (configurable)
- Surface = π × r² (0.002827 m² pour r=3cm)
- Efficacité = 0.85 (pertes système)
```

### Modèle de Friction

Sur surface vitrée:
- Coefficient friction statique: μ = 1.2
- Friction dynamique: similaire
- Force friction = Force_adhérence × μ

### Bruit Capteurs

- **HC-SR04**: Gaussien, stddev = 1cm
- **MPU6050**: 
  - Gyro: stddev = 0.00017 rad/s (~0.01°/s)
  - Accel: stddev = 0.00196 m/s² (~0.12 m/s²)

## Performance

### Fréquences de Mise à Jour

- Capteurs ultrason: 10 Hz
- IMU: 100 Hz
- Batterie: 10 Hz
- Encodeurs: 100 Hz
- Contrôleur différentiel: 50 Hz
- Système ventouse: 50 Hz

### Optimisations

1. **Plugins C++**: Performance native
2. **Mise à jour conditionnelle**: Vérification dt avant update
3. **Allocation mémoire minimale**: Réutilisation de messages
4. **Spin non-bloquant**: `rclcpp::spin_some()` dans callbacks

## Extensibilité

### Ajouter un Nouveau Capteur

1. Créer plugin C++ dans `plugins/src/`
2. Ajouter macro xacro dans `urdf/windowclean_pro.urdf.xacro`
3. Configurer dans URDF
4. Installer dans CMakeLists.txt

### Ajouter un Nouvel Environnement

1. Créer `.world` dans `worlds/`
2. Créer launch file dans `launch/`
3. Optionnel: Ajouter dans documentation

## Dépendances

### ROS2 Packages
- `rclcpp`, `rclpy`
- `sensor_msgs`, `geometry_msgs`, `nav_msgs`
- `tf2_ros`, `robot_state_publisher`
- `gazebo_ros`, `xacro`

### Gazebo
- Gazebo Garden/Fortress
- ignition-math6
- sdformat13

### Système
- CMake >= 3.8
- C++17 compiler
- Python 3.8+

## Points Critiques

1. **Thread-safety**: Plugins thread-safe pour multi-threading
2. **Real-time**: Optimisé pour temps réel simulé
3. **Modularité**: Chaque plugin indépendant
4. **Réalisme**: Modèles physiques réalistes
5. **Débogage**: Logs détaillés intégrés

## Évolutions Futures

- Support multi-robots
- Modèles de saleté dynamiques
- Pattern de nettoyage optimisé (ML)
- Intégration avec SLAM
- Support caméra pour vision

