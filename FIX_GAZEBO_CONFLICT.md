# Résolution du Conflit Gazebo Classic / Gazebo Garden

## Problème

Il y a un conflit entre :
- Gazebo Classic (11.10.2) - installé via packages Ubuntu
- Gazebo Garden (7.9.0) - installé via OSRF
- `ros-humble-gazebo-ros` nécessite Gazebo Classic mais entre en conflit avec `gz-tools2`

## Solutions

### Solution 1 : Utiliser Gazebo Classic uniquement (Recommandé pour ce projet)

Les plugins actuels sont écrits pour l'API Gazebo Classic. Pour utiliser cette solution :

```bash
# Désinstaller Gazebo Garden
sudo apt remove gz-garden gz-tools2 gz-sim7-cli

# Installer Gazebo Classic complet
sudo apt install gazebo libgazebo11-dev

# Installer gazebo_ros
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs
```

**Note**: Vous perdrez `gz sim` mais pourrez utiliser `gazebo` classique.

### Solution 2 : Adapter pour Gazebo Garden (Complexe)

Les plugins doivent être réécrits pour utiliser l'API `gz::sim` au lieu de `gazebo::`. Cela nécessite :
- Réécrire tous les plugins C++
- Utiliser `gz_ros2_control` au lieu de `gazebo_ros`
- Changer tous les includes et namespaces

### Solution 3 : Compilation directe sans gazebo_ros (Temporaire)

On peut compiler les plugins directement avec les bibliothèques Gazebo Classic sans passer par gazebo_ros, mais cela limite l'intégration ROS2.

## Recommandation

Pour ce projet, **Solution 1** est recommandée car :
1. Les plugins sont déjà écrits pour Gazebo Classic
2. `gazebo_ros` fournit une intégration ROS2 complète
3. Gazebo Classic est stable et bien supporté

Après avoir suivi la Solution 1, vous pourrez compiler avec :

```bash
cd windowclean_pro_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

