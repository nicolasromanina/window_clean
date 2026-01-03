#!/bin/bash
# Script pour résoudre le conflit Gazebo Classic / Garden

set -e

echo "========================================="
echo "Résolution du conflit Gazebo"
echo "========================================="
echo ""

echo "Option 1 : Utiliser Gazebo Classic (recommandé)"
echo "  - Désinstalle Gazebo Garden"
echo "  - Installe Gazebo Classic complet"
echo "  - Installe gazebo_ros pour ROS2"
echo ""
read -p "Continuer avec l'option 1? (o/N): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Oo]$ ]]; then
    echo "Désinstallation de Gazebo Garden..."
    sudo apt remove -y gz-garden gz-tools2 || true
    
    echo "Installation de Gazebo Classic..."
    sudo apt install -y gazebo libgazebo11-dev
    
    echo "Installation de gazebo_ros..."
    sudo apt install -y ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs
    
    echo ""
    echo "✓ Installation terminée!"
    echo ""
    echo "Vous pouvez maintenant compiler:"
    echo "  cd windowclean_pro_ws"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  colcon build --symlink-install"
else
    echo "Opération annulée"
    exit 0
fi

