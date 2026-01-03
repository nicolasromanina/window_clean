# Guide de Démarrage Rapide - WindowClean Pro

Guide rapide pour démarrer avec la simulation WindowClean Pro.

## 🚀 Installation Rapide

```bash
cd windowclean_pro_ws
colcon build --symlink-install
source install/setup.bash
```

## 🎮 Lancement de Base

### 1. Lancer la simulation simple

```bash
ros2 launch windowclean_pro simple_window.launch.py
```

### 2. Dans un autre terminal, contrôler le robot

```bash
# Activer les ventouses
ros2 topic pub /windowclean_pro/suction_fl_link/vacuum/enable std_msgs/msg/Bool "{data: true}"

# Faire avancer le robot
ros2 topic pub /windowclean_pro/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Arrêter
ros2 topic pub /windowclean_pro/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 📊 Visualisation des Données

```bash
# Odométrie
ros2 topic echo /windowclean_pro/odom

# Capteur ultrason avant
ros2 topic echo /windowclean_pro/ultrasonic_front_link/range

# IMU
ros2 topic echo /windowclean_pro/imu/data

# Batterie
ros2 topic echo /windowclean_pro/battery/state
```

## ✅ Tests de Validation

```bash
# Test basique
ros2 run windowclean_pro test_basic_functionality.py

# Test de navigation
ros2 run windowclean_pro test_navigation_pattern.py

# Métriques de performance
ros2 run windowclean_pro test_performance_metrics.py
```

## 📚 Documentation Complète

- [README.md](README.md) - Documentation complète
- [docs/API.md](docs/API.md) - Documentation API des plugins

## 🆘 Problèmes Courants

### Gazebo ne démarre pas

Vérifier que Gazebo est installé:
```bash
gz sim --version
```

### Plugins non trouvés

Recompiler et sourcer:
```bash
cd windowclean_pro_ws
colcon build
source install/setup.bash
```

### Topics non disponibles

Vérifier que la simulation est lancée et que le robot est spawné:
```bash
ros2 topic list | grep windowclean_pro
```

