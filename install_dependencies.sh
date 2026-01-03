#!/bin/bash
# Script d'installation automatique des dépendances pour WindowClean Pro

set -e  # Arrêter en cas d'erreur

echo "========================================="
echo "Installation des dépendances WindowClean Pro"
echo "========================================="

# Couleurs pour l'affichage
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Vérifier que nous sommes sur Ubuntu
if ! grep -q "Ubuntu" /etc/os-release; then
    echo -e "${RED}Erreur: Ce script est conçu pour Ubuntu${NC}"
    exit 1
fi

echo -e "${YELLOW}[1/8] Mise à jour des packages...${NC}"
sudo apt update

echo -e "${YELLOW}[2/8] Installation des outils de base...${NC}"
sudo apt install -y curl software-properties-common lsb-release wget gnupg

# Installation de ROS2 Humble
echo -e "${YELLOW}[3/8] Configuration du dépôt ROS2...${NC}"
if ! grep -q "ros2" /etc/apt/sources.list.d/ros2-latest.list 2>/dev/null; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
fi

echo -e "${YELLOW}[4/8] Installation de ROS2 Humble...${NC}"
sudo apt update
sudo apt install -y ros-humble-desktop

echo -e "${YELLOW}[5/8] Installation de colcon et outils de build...${NC}"
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo apt install -y build-essential cmake

# Initialisation de rosdep
echo -e "${YELLOW}[5.5/8] Initialisation de rosdep...${NC}"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
fi
rosdep update || true

# Packages ROS2 pour Gazebo
echo -e "${YELLOW}[6/8] Installation des packages ROS2 pour Gazebo...${NC}"
sudo apt install -y ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-robot-state-publisher
sudo apt install -y ros-humble-joint-state-publisher
sudo apt install -y ros-humble-xacro
sudo apt install -y ros-humble-tf2-tools

# Installation de Gazebo Garden (si pas déjà installé)
echo -e "${YELLOW}[7/8] Vérification de Gazebo...${NC}"
if ! command -v gz &> /dev/null || ! gz sim --version &> /dev/null; then
    echo "Installation de Gazebo Garden..."
    if [ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then
        sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
        sudo apt update
    fi
    sudo apt install -y gz-garden
fi

# Dépendances de développement
echo -e "${YELLOW}[8/8] Installation des dépendances de développement...${NC}"
sudo apt install -y libignition-math6-dev libsdformat13-dev
sudo apt install -y libgazebo-dev || sudo apt install -y libgazebo11-dev || true

echo ""
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}Installation terminée!${NC}"
echo -e "${GREEN}=========================================${NC}"
echo ""

# Configuration du .bashrc
echo -e "${YELLOW}Configuration de l'environnement...${NC}"
BASHRC="$HOME/.bashrc"

if ! grep -q "source /opt/ros/humble/setup.bash" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# ROS2 Humble" >> "$BASHRC"
    echo "source /opt/ros/humble/setup.bash" >> "$BASHRC"
    echo "" >> "$BASHRC"
    echo "# Colcon bash completion" >> "$BASHRC"
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> "$BASHRC"
    echo -e "${GREEN}Configuration ajoutée à ~/.bashrc${NC}"
fi

echo ""
echo -e "${YELLOW}Prochaines étapes:${NC}"
echo "1. Rechargez votre shell: source ~/.bashrc"
echo "2. Compilez le projet: cd windowclean_pro_ws && colcon build --symlink-install"
echo "3. Sourcez l'installation: source install/setup.bash"
echo "4. Lancez la simulation: ros2 launch windowclean_pro simple_window.launch.py"
echo ""

