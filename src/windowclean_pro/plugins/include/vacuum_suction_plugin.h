#ifndef VACUUM_SUCTION_PLUGIN_H
#define VACUUM_SUCTION_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ignition/math/Vector3.hh>
#include <memory>

namespace gazebo
{
  /**
   * @brief Plugin Gazebo pour simuler un système de ventouse à vide régulé
   * 
   * Caractéristiques:
   * - Force d'adhérence variable selon la pression différentielle
   * - Simulation réaliste des forces de friction et adhésion
   * - Contrôle via ROS2 (activation/désactivation, force cible)
   * 
   * Publie sur ROS2:
   * - std_msgs/Bool: État de la ventouse (active/inactive)
   * - std_msgs/Float64: Force d'adhérence actuelle
   * 
   * S'abonne à:
   * - std_msgs/Bool: Commande d'activation
   * - std_msgs/Float64: Force cible (optionnel)
   */
  class VacuumSuctionPlugin : public ModelPlugin
  {
    public:
      VacuumSuctionPlugin();
      virtual ~VacuumSuctionPlugin();
      
      // Hérité de ModelPlugin
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      virtual void OnUpdate();
      
    private:
      // Modèle physique
      physics::ModelPtr model_;
      physics::LinkPtr link_;
      physics::WorldPtr world_;
      
      // ROS2
      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_pub_;
      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
      rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr force_target_sub_;
      
      // Paramètres SDF
      std::string topic_name_;
      std::string link_name_;
      double max_suction_force_;
      double suction_area_;
      double update_rate_;
      
      // État
      bool enabled_;
      double current_force_;
      double target_force_;
      double vacuum_pressure_;  // Pression négative en Pa
      
      // Constantes physiques
      static constexpr double ATMOSPHERIC_PRESSURE = 101325.0;  // Pa
      static constexpr double VACUUM_EFFICIENCY = 0.85;  // Efficacité du système
      
      // Callback
      event::ConnectionPtr update_connection_;
      
      // Fonctions
      void SetupROS();
      void OnEnableCommand(const std_msgs::msg::Bool::SharedPtr msg);
      void OnForceTarget(const std_msgs::msg::Float64::SharedPtr msg);
      void ApplySuctionForce();
      double CalculateSuctionForce();
      ignition::math::Vector3d GetSurfaceNormal();
      bool IsInContactWithSurface();
  };
  
  // Enregistrement du plugin
  GZ_REGISTER_MODEL_PLUGIN(VacuumSuctionPlugin)
}

#endif // VACUUM_SUCTION_PLUGIN_H

