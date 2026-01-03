#ifndef ULTRASONIC_SENSOR_PLUGIN_H
#define ULTRASONIC_SENSOR_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <random>

namespace gazebo
{
  /**
   * @brief Plugin Gazebo pour simuler un capteur ultrason HC-SR04
   * 
   * Caractéristiques:
   * - Portée: 2-400 cm (configurable)
   * - Angle de détection: ~15 degrés
   * - Bruit gaussien simulé
   * - Fréquence de mise à jour: 10 Hz (HC-SR04 typique)
   * 
   * Publie sur ROS2:
   * - sensor_msgs/Range sur le topic configuré
   */
  class UltrasonicSensorPlugin : public SensorPlugin
  {
    public:
      UltrasonicSensorPlugin();
      virtual ~UltrasonicSensorPlugin();
      
      // Hérité de SensorPlugin
      virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
      virtual void OnUpdate();
      
    private:
      // Sensor
      sensors::RaySensorPtr ray_sensor_;
      
      // ROS2
      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
      sensor_msgs::msg::Range range_msg_;
      
      // Paramètres SDF
      std::string topic_name_;
      std::string frame_name_;
      double min_range_;
      double max_range_;
      double noise_mean_;
      double noise_stddev_;
      
      // Génération de bruit
      std::default_random_engine generator_;
      std::normal_distribution<double> noise_distribution_;
      
      // Callback de mise à jour
      event::ConnectionPtr update_connection_;
      
      // Métriques
      double last_update_time_;
      double update_rate_;
      
      // Fonctions utilitaires
      double AddNoise(double distance);
      double ClampRange(double distance);
      void SetupROS();
      void PublishRange(double distance);
  };
  
  // Enregistrement du plugin
  GZ_REGISTER_SENSOR_PLUGIN(UltrasonicSensorPlugin)
}

#endif // ULTRASONIC_SENSOR_PLUGIN_H

