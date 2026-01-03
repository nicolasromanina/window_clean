# Installation Rapide - WindowClean Pro

## Problème : `colcon: command not found`

Si vous voyez cette erreur, c'est que ROS2 n'est pas encore installé sur votre système.

## Solution Rapide

### Option 1 : Script d'installation automatique (Recommandé)

```bash
cd ~/.gazebo/models/robot_autonome/windowclean_pro_ws
./install_dependencies.sh
source ~/.bashrc
```

### Option 2 : Installation manuelle

#### 1. Installer ROS2 Humble

```bash
# Configuration du dépôt
sudo apt update
sudo apt install software-properties-common
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Installation
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install python3-colcon-common-extensions -y
```

#### 2. Packages ROS2 pour Gazebo

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher ros-humble-xacro -y
```

#### 3. Configuration de l'environnement

Ajoutez à votre `~/.bashrc` :

```bash
source /opt/ros/humble/setup.bash
```

Puis rechargez :
```bash
source ~/.bashrc
```

#### 4. Compiler le projet

```bash
cd ~/.gazebo/models/robot_autonome/windowclean_pro_ws
colcon build --symlink-install
source install/setup.bash
```

## Vérification

Vérifiez que tout est installé :

```bash
ros2 --version      # Devrait afficher la version de ROS2
colcon --version    # Devrait afficher la version de colcon
xacro --version     # Devrait afficher la version de xacro
gz sim --version    # Devrait afficher la version de Gazebo
```

## Documentation complète

Pour plus de détails, consultez `INSTALL.md`

## Prochaines étapes

Une fois l'installation terminée :

```bash
# Lancer la simulation
ros2 launch windowclean_pro simple_window.launch.py

# Dans un autre terminal, tester
ros2 run windowclean_pro test_basic_functionality.py
```

