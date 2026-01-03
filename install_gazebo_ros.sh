#!/bin/bash
# Script pour installer gazebo_ros

echo "Installation de gazebo_ros pour ROS2 Humble..."

sudo apt update
sudo apt install -y ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Installation réussie!"
    echo ""
    echo "Vous pouvez maintenant compiler le projet:"
    echo "  cd windowclean_pro_ws"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  colcon build --symlink-install"
else
    echo ""
    echo "✗ Erreur lors de l'installation"
    exit 1
fi

