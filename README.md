# WindowClean Pro - Robot Nettoyeur de Vitres Autonome

Simulation complète du robot nettoyeur de vitres autonome WindowClean Pro pour Gazebo avec ROS2.

## 📋 Table des Matières

- [Description](#description)
- [Architecture](#architecture)
- [Installation](#installation)
- [Utilisation](#utilisation)
- [Plugins Gazebo](#plugins-gazebo)
- [Environnements de Test](#environnements-de-test)
- [Validation](#validation)
- [Documentation API](#documentation-api)

## 🚀 Description

WindowClean Pro est un robot autonome conçu pour nettoyer des surfaces vitrées verticales. Cette simulation complète inclut :

- **Modèle physique détaillé** avec URDF/Xacro
- **Plugins Gazebo personnalisés** pour tous les capteurs et actionneurs
- **Environnements de test** progressifs
- **Interfaces ROS2** complètes
- **Scripts de validation** automatisés

### Caractéristiques Principales

- **Dimensions**: 30×30×8 cm, masse 2.5 kg
- **MCU**: ESP32-S3 (simulé via ROS2)
- **Capteurs**: 
  - 7× HC-SR04 (ultrasons)
  - MPU6050 (IMU 6 axes)
  - Capteur pression différentielle
  - Encodeurs incrémentaux 600 PPR
- **Actionneurs**:
  - 2× moteurs DC brushless avec réducteur 100:1
  - 2× micropompes péristaltiques
  - 1× moteur pas-à-pas NEMA11 (bras nettoyeur)
  - Système ventouses à vide régulé (4 ventouses)

## 🏗️ Architecture

```
windowclean_pro/
├── urdf/                  # Modèles URDF/Xacro
│   └── windowclean_pro.urdf.xacro
├── plugins/               # Plugins Gazebo C++
│   ├── src/              # Sources C++
│   └── include/          # Headers
├── launch/               # Launch files ROS2
│   ├── simple_window.launch.py
│   ├── apartment.launch.py
│   ├── office.launch.py
│   └── complex.launch.py
├── worlds/               # Environnements Gazebo
│   ├── simple_window.world
│   ├── apartment.world
│   ├── office.world
│   └── complex.world
├── scripts/              # Scripts Python ROS2
│   ├── validation/       # Scripts de test
│   └── ...
├── config/               # Fichiers de configuration
└── docs/                 # Documentation API
```

## 📦 Installation

### Prérequis

- **ROS2 Humble** ou **Iron**
- **Gazebo Garden** ou **Fortress**
- **CMake** >= 3.8
- **C++17** compatible compiler
- Packages ROS2:
  ```bash
  sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs \
                   ros-$ROS_DISTRO-robot-state-publisher \
                   ros-$ROS_DISTRO-joint-state-publisher \
                   ros-$ROS_DISTRO-xacro
  ```

### Compilation

```bash
cd windowclean_pro_ws
colcon build --symlink-install
source install/setup.bash
```

## 🎮 Utilisation

### Lancement Simple

```bash
ros2 launch windowclean_pro simple_window.launch.py
```

### Scénarios Disponibles

1. **Vitre simple** (1×2m)
   ```bash
   ros2 launch windowclean_pro simple_window.launch.py
   ```

2. **Appartement** (5 vitres différentes)
   ```bash
   ros2 launch windowclean_pro apartment.launch.py
   ```

3. **Bureau open-space** (grandes baies)
   ```bash
   ros2 launch windowclean_pro office.launch.py
   ```

4. **Environnement complexe** (avec obstacles)
   ```bash
   ros2 launch windowclean_pro complex.launch.py
   ```

### Contrôle du Robot

#### Commandes de base

```bash
# Mouvement
ros2 topic pub /windowclean_pro/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"

# Activer les ventouses
ros2 topic pub /windowclean_pro/suction_fl_link/vacuum/enable std_msgs/msg/Bool \
  "{data: true}"

# Désactiver les ventouses
ros2 topic pub /windowclean_pro/suction_fl_link/vacuum/enable std_msgs/msg/Bool \
  "{data: false}"
```

#### Visualisation des données

```bash
# Odométrie
ros2 topic echo /windowclean_pro/odom

# Capteurs ultrason
ros2 topic echo /windowclean_pro/ultrasonic_front_link/range

# IMU
ros2 topic echo /windowclean_pro/imu/data

# Batterie
ros2 topic echo /windowclean_pro/battery/state
```

## 🔌 Plugins Gazebo

### 1. UltrasonicSensorPlugin

Simule un capteur ultrason HC-SR04 avec bruit réaliste.

**Paramètres SDF:**
- `topic_name`: Topic ROS2 pour publier les données
- `frame_name`: Frame TF du capteur
- `min_range`: Distance minimale (m)
- `max_range`: Distance maximale (m)
- `noise_mean`: Moyenne du bruit
- `noise_stddev`: Écart-type du bruit
- `update_rate`: Fréquence de mise à jour (Hz)

**Topic publié:**
- `sensor_msgs/Range`: Données de distance

### 2. VacuumSuctionPlugin

Simule un système de ventouse à vide avec force d'adhérence variable.

**Paramètres SDF:**
- `link_name`: Nom du lien sur lequel appliquer la force
- `topic_name`: Namespace pour les topics ROS2
- `max_suction_force`: Force maximale (N)
- `suction_area`: Surface de la ventouse (m²)
- `update_rate`: Fréquence de mise à jour (Hz)

**Topics ROS2:**
- `{topic_name}/state` (std_msgs/Bool): État actif/inactif
- `{topic_name}/force` (std_msgs/Float64): Force actuelle
- `{topic_name}/enable` (std_msgs/Bool): Commande d'activation
- `{topic_name}/force_target` (std_msgs/Float64): Force cible

### 3. ImuMpu6050Plugin

Simule un IMU MPU6050 avec dérive et bruit.

**Paramètres SDF:**
- `topic_name`: Topic ROS2
- `frame_name`: Frame TF
- `update_rate`: Fréquence (Hz)
- `gyro_noise_stddev`: Bruit gyroscope
- `accel_noise_stddev`: Bruit accéléromètre

**Topic publié:**
- `sensor_msgs/Imu`: Données IMU (accélération, vitesse angulaire, orientation)

### 4. BatteryPlugin

Simule une batterie Li-ion avec décharge réaliste.

**Paramètres SDF:**
- `topic_name`: Topic ROS2
- `initial_charge`: Charge initiale (%)
- `capacity`: Capacité (Ah)
- `voltage`: Tension nominale (V)
- `current_draw`: Courant moyen (A)
- `update_rate`: Fréquence (Hz)

**Topic publié:**
- `sensor_msgs/BatteryState`: État de la batterie

### 5. EncoderPlugin

Simule un encodeur incrémental.

**Paramètres SDF:**
- `joint_name`: Nom du joint à surveiller
- `topic_name`: Topic ROS2
- `pulses_per_revolution`: Résolution (PPR)
- `update_rate`: Fréquence (Hz)

**Topic publié:**
- `sensor_msgs/JointState`: Position, vitesse, pulses

### 6. DifferentialDriveControllerPlugin

Contrôleur différentiel avec odométrie.

**Paramètres SDF:**
- `left_joint`: Joint de la roue gauche
- `right_joint`: Joint de la roue droite
- `wheel_separation`: Distance entre les roues (m)
- `wheel_diameter`: Diamètre des roues (m)
- `max_wheel_torque`: Couple maximal (N⋅m)
- `command_topic`: Topic pour cmd_vel
- `odometry_topic`: Topic pour l'odométrie
- `update_rate`: Fréquence (Hz)

**Topics ROS2:**
- S'abonne à: `geometry_msgs/Twist` (cmd_vel)
- Publie: `nav_msgs/Odometry` (odom)

## 🌍 Environnements de Test

### 1. Simple Window (simple_window.world)

Environnement minimal avec une vitre unique 1×2m pour tests basiques.

**Utilisation:**
- Tests de navigation de base
- Validation des capteurs
- Calibration des contrôleurs

### 2. Apartment (apartment.world)

Appartement avec 5 vitres de tailles différentes:
- Fenêtre salon (2×1.5m)
- Fenêtre chambre 1 (1.5×1.2m)
- Fenêtre chambre 2 (1.5×1.2m)
- Baie vitrée cuisine (3×2m)
- Fenêtre salle de bain (0.8×0.6m)

**Utilisation:**
- Tests de navigation multi-fenêtres
- Pattern boustrophédon
- Gestion des transitions entre fenêtres

### 3. Office (office.world)

Bureau open-space avec grandes baies vitrées.

**Utilisation:**
- Tests sur grandes surfaces
- Optimisation de trajectoires
- Tests d'endurance

### 4. Complex (complex.world)

Environnement avec obstacles imprévus et défis.

**Utilisation:**
- Tests d'évitement d'obstacles
- Gestion d'erreurs
- Tests de robustesse

## ✅ Validation

### Scripts de Test Automatisés

Les scripts de validation se trouvent dans `scripts/validation/`:

#### 1. Test Basique de Fonctionnalité

```bash
ros2 run windowclean_pro test_basic_functionality.py
```

Valide que tous les composants fonctionnent:
- Capteurs ultrason
- IMU
- Batterie
- Contrôle des moteurs
- Système ventouse

#### 2. Test de Navigation

```bash
ros2 run windowclean_pro test_navigation_pattern.py
```

Teste:
- Pattern boustrophédon
- Suivi de trajectoire
- Évitement d'obstacles

#### 3. Métriques de Performance

```bash
ros2 run windowclean_pro test_performance_metrics.py
```

Mesure:
- Distance parcourue
- Vitesse moyenne/maximale
- Consommation de batterie

### Exécution de Tous les Tests

```bash
cd windowclean_pro_ws
source install/setup.bash

# Dans un terminal: Lancer la simulation
ros2 launch windowclean_pro simple_window.launch.py

# Dans un autre terminal: Lancer les tests
ros2 run windowclean_pro test_basic_functionality.py
```

## 📚 Documentation API

La documentation complète de l'API des plugins est disponible dans `docs/API.md`.

### Structure des Topics ROS2

```
/windowclean_pro/
├── cmd_vel                    # geometry_msgs/Twist
├── odom                       # nav_msgs/Odometry
├── imu/
│   └── data                  # sensor_msgs/Imu
├── battery/
│   └── state                 # sensor_msgs/BatteryState
├── ultrasonic_*_link/
│   └── range                 # sensor_msgs/Range
├── suction_*_link/
│   ├── vacuum/
│   │   ├── enable            # std_msgs/Bool
│   │   ├── state             # std_msgs/Bool
│   │   ├── force             # std_msgs/Float64
│   │   └── force_target      # std_msgs/Float64
└── */encoder                  # sensor_msgs/JointState
```

## 🐛 Débogage

### Visualisation TF

```bash
ros2 run tf2_tools view_frames
```

### Inspection des Topics

```bash
ros2 topic list
ros2 topic info /windowclean_pro/odom
ros2 topic hz /windowclean_pro/imu/data
```

### Logs Gazebo

Les logs Gazebo incluent des messages détaillés des plugins:
- Activation/désactivation des composants
- Erreurs de configuration
- Avertissements de performance

## 🤝 Contribution

Pour contribuer au projet:
1. Fork le repository
2. Créer une branche pour votre fonctionnalité
3. Faire vos modifications
4. Tester avec les scripts de validation
5. Soumettre une pull request

## 📄 Licence

MIT License - Voir LICENSE pour plus de détails.

## 👤 Auteur

Développé pour la simulation de robot nettoyeur de vitres autonome.

## 📞 Support

Pour toute question ou problème:
- Ouvrir une issue sur le repository
- Consulter la documentation dans `docs/`

