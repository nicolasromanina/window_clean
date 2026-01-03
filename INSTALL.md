# Guide d'Installation - WindowClean Pro

## Prérequis

- **Ubuntu 22.04** (Jammy) - Pour ROS2 Humble
- **Gazebo Garden/Fortress** - Version moderne recommandée
- Accès Internet pour télécharger les packages

## Installation de ROS2 Humble

### 1. Configuration du dépôt

```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

### 2. Installation des packages ROS2

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

### 3. Installation de colcon et dépendances

```bash
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep python3-vcstool -y
```

### 4. Configuration de l'environnement ROS2

Ajoutez à votre `~/.bashrc` :

```bash
# ROS2 Humble
source /opt/ros/humble/setup.bash

# Colcon bash completion
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```

Puis rechargez :
```bash
source ~/.bashrc
```

## Installation de Gazebo Garden/Fortress

### Option 1: Gazebo Garden (recommandé)

```bash
sudo apt install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install gz-garden -y
```

### Option 2: Utiliser Gazebo 11 existant (si vous voulez continuer avec)

Note: Gazebo 11 est plus ancien mais fonctionne. Pour les meilleures performances, utilisez Gazebo Garden.

## Installation des packages ROS2 pour Gazebo

```bash
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-robot-state-publisher -y
sudo apt install ros-humble-joint-state-publisher -y
sudo apt install ros-humble-xacro -y
sudo apt install ros-humble-tf2-tools -y
```

## Installation des dépendances de build

```bash
sudo apt install build-essential cmake -y
sudo apt install libignition-math6-dev libsdformat13-dev -y
```

## Vérification de l'installation

```bash
# Vérifier ROS2
ros2 --version

# Vérifier colcon
colcon --version

# Vérifier Gazebo
gz sim --version

# Vérifier xacro
xacro --version
```

## Compilation du projet WindowClean Pro

Une fois toutes les dépendances installées :

```bash
cd ~/.gazebo/models/robot_autonome/windowclean_pro_ws
colcon build --symlink-install
source install/setup.bash
```

## Dépannage

### Colcon non trouvé

Si `colcon` n'est pas trouvé après installation :

```bash
sudo apt install python3-colcon-common-extensions --reinstall
hash -r  # Recharger les commandes
```

### Gazebo plugins non trouvés

Assurez-vous que le chemin des plugins est correct :

```bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(pwd)/install/lib
```

### Problèmes de compilation C++

Vérifier que les headers Gazebo sont disponibles :

```bash
pkg-config --cflags gazebo
```

Si cela échoue, installer les dev packages :

```bash
sudo apt install libgazebo-dev libgazebo11-dev -y
```

## Installation rapide (script)

Vous pouvez exécuter le script d'installation automatique :

```bash
chmod +x install_dependencies.sh
./install_dependencies.sh
```

