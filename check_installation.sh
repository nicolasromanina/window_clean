#!/bin/bash
# Script de vérification de l'installation

echo "========================================="
echo "Vérification de l'installation"
echo "========================================="
echo ""

# Source ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS2 Humble installé"
else
    echo "✗ ROS2 Humble NON installé"
    echo "  Exécutez: ./install_dependencies.sh"
    exit 1
fi

# Vérifier ros2
if command -v ros2 &> /dev/null; then
    echo "✓ ros2 commande disponible"
else
    echo "✗ ros2 commande NON disponible"
    exit 1
fi

# Vérifier colcon
if command -v colcon &> /dev/null; then
    echo "✓ colcon installé"
    colcon version | head -1
else
    echo "✗ colcon NON installé"
    echo "  Exécutez: sudo apt install python3-colcon-common-extensions"
    exit 1
fi

# Vérifier xacro
if command -v xacro &> /dev/null; then
    echo "✓ xacro installé"
    xacro --version
else
    echo "✗ xacro NON installé"
    echo "  Exécutez: sudo apt install ros-humble-xacro"
fi

# Vérifier Gazebo
if command -v gz &> /dev/null; then
    echo "✓ Gazebo installé"
    gz sim --version 2>/dev/null || echo "  (Gazebo Classic détecté)"
else
    echo "✗ Gazebo NON installé"
fi

echo ""
echo "========================================="
echo "Pour compiler le projet:"
echo "  cd windowclean_pro_ws"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
echo "========================================="

