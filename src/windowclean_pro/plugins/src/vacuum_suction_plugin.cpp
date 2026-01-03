#include "vacuum_suction_plugin.h"
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector3.hh>
#include <memory>

using namespace gazebo;

// ==========================================
// CONSTRUCTEUR / DESTRUCTEUR
// ==========================================

VacuumSuctionPlugin::VacuumSuctionPlugin()
  : max_suction_force_(100.0)
  , suction_area_(0.002827)  // π * 0.03^2 m²
  , update_rate_(50.0)
  , enabled_(false)
  , current_force_(0.0)
  , target_force_(100.0)
  , vacuum_pressure_(0.0)
{
}

VacuumSuctionPlugin::~VacuumSuctionPlugin()
{
  if (ros_node_)
  {
    ros_node_->~Node();
  }
}

// ==========================================
// CHARGEMENT DU PLUGIN
// ==========================================

void VacuumSuctionPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Vérifications
  if (!_model)
  {
    gzerr << "VacuumSuctionPlugin: Model pointer is NULL\n";
    return;
  }
  
  this->model_ = _model;
  this->world_ = _model->GetWorld();
  
  // Lecture des paramètres SDF
  if (_sdf->HasElement("topic_name"))
  {
    this->topic_name_ = _sdf->Get<std::string>("topic_name");
  }
  else
  {
    this->topic_name_ = "vacuum";
  }
  
  if (_sdf->HasElement("link_name"))
  {
    this->link_name_ = _sdf->Get<std::string>("link_name");
    this->link_ = this->model_->GetLink(this->link_name_);
  }
  else
  {
    // Utiliser le premier lien du modèle
    this->link_ = this->model_->GetLink();
    this->link_name_ = this->link_->GetName();
  }
  
  if (!this->link_)
  {
    gzerr << "VacuumSuctionPlugin: Link '" << this->link_name_ << "' not found\n";
    return;
  }
  
  if (_sdf->HasElement("max_suction_force"))
  {
    this->max_suction_force_ = _sdf->Get<double>("max_suction_force");
  }
  
  if (_sdf->HasElement("suction_area"))
  {
    this->suction_area_ = _sdf->Get<double>("suction_area");
  }
  
  if (_sdf->HasElement("update_rate"))
  {
    this->update_rate_ = _sdf->Get<double>("update_rate");
  }
  
  // Initialisation ROS2
  this->SetupROS();
  
  // Connexion au callback de mise à jour
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&VacuumSuctionPlugin::OnUpdate, this)
  );
  
  gzmsg << "VacuumSuctionPlugin loaded: "
        << "link=" << this->link_name_
        << ", max_force=" << this->max_suction_force_ << " N"
        << ", area=" << this->suction_area_ << " m²"
        << ", rate=" << this->update_rate_ << " Hz\n";
}

// ==========================================
// SETUP ROS2
// ==========================================

void VacuumSuctionPlugin::SetupROS()
{
  // Initialisation ROS2 si nécessaire
  if (!rclcpp::ok())
  {
    int argc = 0;
    char** argv = NULL;
    rclcpp::init(argc, argv);
  }
  
  // Création du noeud ROS2
  this->ros_node_ = rclcpp::Node::make_shared("vacuum_suction_plugin");
  
  // Publishers
  this->state_pub_ = this->ros_node_->create_publisher<std_msgs::msg::Bool>(
    this->topic_name_ + "/state", 
    10
  );
  
  this->force_pub_ = this->ros_node_->create_publisher<std_msgs::msg::Float64>(
    this->topic_name_ + "/force", 
    10
  );
  
  // Subscribers
  this->enable_sub_ = this->ros_node_->create_subscription<std_msgs::msg::Bool>(
    this->topic_name_ + "/enable",
    10,
    std::bind(&VacuumSuctionPlugin::OnEnableCommand, this, std::placeholders::_1)
  );
  
  this->force_target_sub_ = this->ros_node_->create_subscription<std_msgs::msg::Float64>(
    this->topic_name_ + "/force_target",
    10,
    std::bind(&VacuumSuctionPlugin::OnForceTarget, this, std::placeholders::_1)
  );
  
  gzmsg << "VacuumSuctionPlugin ROS2 setup complete\n";
}

// ==========================================
// CALLBACK DE MISE À JOUR
// ==========================================

void VacuumSuctionPlugin::OnUpdate()
{
  // Traitement des messages ROS2
  rclcpp::spin_some(this->ros_node_);
  
  // Application de la force de succion si activé
  if (this->enabled_)
  {
    this->ApplySuctionForce();
    
    // Publication de l'état
    std_msgs::msg::Bool state_msg;
    state_msg.data = this->enabled_;
    this->state_pub_->publish(state_msg);
    
    std_msgs::msg::Float64 force_msg;
    force_msg.data = this->current_force_;
    this->force_pub_->publish(force_msg);
  }
}

// ==========================================
// CALLBACKS ROS2
// ==========================================

void VacuumSuctionPlugin::OnEnableCommand(const std_msgs::msg::Bool::SharedPtr msg)
{
  this->enabled_ = msg->data;
  
  if (this->enabled_)
  {
    gzmsg << "VacuumSuctionPlugin: Suction ENABLED\n";
    // Calcul de la pression de vide nécessaire
    this->vacuum_pressure_ = -(this->target_force_ / this->suction_area_) / VACUUM_EFFICIENCY;
  }
  else
  {
    gzmsg << "VacuumSuctionPlugin: Suction DISABLED\n";
    this->vacuum_pressure_ = 0.0;
    this->current_force_ = 0.0;
  }
}

void VacuumSuctionPlugin::OnForceTarget(const std_msgs::msg::Float64::SharedPtr msg)
{
  this->target_force_ = std::max(0.0, std::min(msg->data, this->max_suction_force_));
  
  // Recalcul de la pression de vide
  if (this->enabled_)
  {
    this->vacuum_pressure_ = -(this->target_force_ / this->suction_area_) / VACUUM_EFFICIENCY;
  }
}

// ==========================================
// APPLICATION DE LA FORCE
// ==========================================

void VacuumSuctionPlugin::ApplySuctionForce()
{
  // Vérifier si en contact avec une surface
  if (!this->IsInContactWithSurface())
  {
    this->current_force_ = 0.0;
    return;
  }
  
  // Calcul de la force normale (perpendiculaire à la surface)
  ignition::math::Vector3d normal = this->GetSurfaceNormal();
  
  // Calcul de la force de succion
  double calculated_force = this->CalculateSuctionForce();
  this->current_force_ = calculated_force;
  
  // Application de la force perpendiculaire à la surface
  ignition::math::Vector3d force_vector = normal * calculated_force;
  
  // Application au centre de la ventouse
  ignition::math::Pose3d link_pose = this->link_->WorldPose();
  ignition::math::Vector3d force_position = link_pose.Pos();
  
  // Application de la force
  this->link_->AddForce(force_vector);
  
  // Ajout d'une force de friction supplémentaire pour simuler l'adhérence
  // Coefficient de friction élevé avec surface vitrée propre
  double friction_coefficient = 1.2;  // Friction statique élevée
  ignition::math::Vector3d linear_vel = this->link_->WorldLinearVel();
  
  // Force de friction opposée au mouvement
  if (linear_vel.Length() > 0.001)
  {
    ignition::math::Vector3d friction_direction = -linear_vel.Normalize();
    double friction_magnitude = calculated_force * friction_coefficient;
    ignition::math::Vector3d friction_force = friction_direction * std::min(
      friction_magnitude, 
      this->model_->GetLink("base_link")->WorldLinearVel().Length() * 10.0
    );
    this->link_->AddForce(friction_force);
  }
}

// ==========================================
// CALCUL DE LA FORCE
// ==========================================

double VacuumSuctionPlugin::CalculateSuctionForce()
{
  // Force = (P_atmosphérique - P_vide) × Surface × Efficacité
  // P_vide est négative, donc la force est positive (vers la surface)
  
  double pressure_differential = ATMOSPHERIC_PRESSURE + this->vacuum_pressure_;
  double force = pressure_differential * this->suction_area_ * VACUUM_EFFICIENCY;
  
  // Limiter à la force maximale
  return std::min(force, this->target_force_);
}

// ==========================================
// DÉTECTION DE SURFACE
// ==========================================

bool VacuumSuctionPlugin::IsInContactWithSurface()
{
  // Vérifier les collisions du lien
  physics::ContactManager* contact_manager = this->world_->Physics()->GetContactManager();
  
  // Pour simplifier, on considère qu'on est toujours en contact si la ventouse est activée
  // Dans une version plus avancée, on pourrait utiliser un ray sensor ou détecter les collisions
  return true;  // Simplification pour la simulation
}

// ==========================================
// NORMALE DE SURFACE
// ==========================================

ignition::math::Vector3d VacuumSuctionPlugin::GetSurfaceNormal()
{
  // Pour une surface vitrée, la normale est perpendiculaire à la surface
  // On assume que la ventouse est orientée vers la surface (axe Z négatif dans le repère du robot)
  
  ignition::math::Pose3d link_pose = this->link_->WorldPose();
  ignition::math::Vector3d normal(0, 0, -1);  // Vers la vitre (vers le bas dans le repère du robot)
  
  // Rotation selon l'orientation du lien
  return link_pose.Rot().RotateVector(normal);
}

