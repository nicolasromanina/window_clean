#include "ultrasonic_sensor_plugin.h"
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector3.hh>
#include <memory>

using namespace gazebo;

// ==========================================
// CONSTRUCTEUR / DESTRUCTEUR
// ==========================================

UltrasonicSensorPlugin::UltrasonicSensorPlugin()
  : min_range_(0.02)
  , max_range_(4.0)
  , noise_mean_(0.0)
  , noise_stddev_(0.01)
  , last_update_time_(0.0)
  , update_rate_(10.0)
{
}

UltrasonicSensorPlugin::~UltrasonicSensorPlugin()
{
  if (ros_node_)
  {
    ros_node_->~Node();
  }
}

// ==========================================
// CHARGEMENT DU PLUGIN
// ==========================================

void UltrasonicSensorPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Vérifications
  if (!_sensor)
  {
    gzerr << "UltrasonicSensorPlugin: Sensor pointer is NULL\n";
    return;
  }
  
  // Cast vers RaySensor
  this->ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
  if (!this->ray_sensor_)
  {
    gzerr << "UltrasonicSensorPlugin: Sensor is not a RaySensor\n";
    return;
  }
  
  // Lecture des paramètres SDF
  if (_sdf->HasElement("topic_name"))
  {
    this->topic_name_ = _sdf->Get<std::string>("topic_name");
  }
  else
  {
    this->topic_name_ = "ultrasonic/range";
    gzwarn << "UltrasonicSensorPlugin: topic_name not specified, using default: " 
           << this->topic_name_ << "\n";
  }
  
  if (_sdf->HasElement("frame_name"))
  {
    this->frame_name_ = _sdf->Get<std::string>("frame_name");
  }
  else
  {
    this->frame_name_ = this->ray_sensor_->ParentName();
  }
  
  if (_sdf->HasElement("min_range"))
  {
    this->min_range_ = _sdf->Get<double>("min_range");
  }
  
  if (_sdf->HasElement("max_range"))
  {
    this->max_range_ = _sdf->Get<double>("max_range");
  }
  
  if (_sdf->HasElement("noise_mean"))
  {
    this->noise_mean_ = _sdf->Get<double>("noise_mean");
  }
  
  if (_sdf->HasElement("noise_stddev"))
  {
    this->noise_stddev_ = _sdf->Get<double>("noise_stddev");
  }
  
  if (_sdf->HasElement("update_rate"))
  {
    this->update_rate_ = _sdf->Get<double>("update_rate");
  }
  
  // Initialisation du générateur de bruit
  std::random_device rd;
  this->generator_ = std::default_random_engine(rd());
  this->noise_distribution_ = std::normal_distribution<double>(
    this->noise_mean_, 
    this->noise_stddev_
  );
  
  // Configuration du capteur
  this->ray_sensor_->SetActive(true);
  
  // Initialisation ROS2
  this->SetupROS();
  
  // Connexion au callback de mise à jour
  this->update_connection_ = this->ray_sensor_->ConnectUpdated(
    std::bind(&UltrasonicSensorPlugin::OnUpdate, this)
  );
  
  gzmsg << "UltrasonicSensorPlugin loaded: " 
        << "topic=" << this->topic_name_
        << ", frame=" << this->frame_name_
        << ", range=[" << this->min_range_ << ", " << this->max_range_ << "]"
        << ", rate=" << this->update_rate_ << " Hz\n";
}

// ==========================================
// SETUP ROS2
// ==========================================

void UltrasonicSensorPlugin::SetupROS()
{
  // Initialisation ROS2 si nécessaire
  if (!rclcpp::ok())
  {
    int argc = 0;
    char** argv = NULL;
    rclcpp::init(argc, argv);
  }
  
  // Création du noeud ROS2
  this->ros_node_ = rclcpp::Node::make_shared("ultrasonic_sensor_plugin");
  
  // Création du publisher
  this->range_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::Range>(
    this->topic_name_, 
    10
  );
  
  // Configuration du message Range
  this->range_msg_.header.frame_id = this->frame_name_;
  this->range_msg_.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  this->range_msg_.field_of_view = 0.261799;  // ~15 degrés en radians
  this->range_msg_.min_range = this->min_range_;
  this->range_msg_.max_range = this->max_range_;
  
  gzmsg << "UltrasonicSensorPlugin ROS2 setup complete\n";
}

// ==========================================
// CALLBACK DE MISE À JOUR
// ==========================================

void UltrasonicSensorPlugin::OnUpdate()
{
  // Contrôle de la fréquence de mise à jour
  double current_time = this->ray_sensor_->LastUpdateTime().Double();
  double dt = current_time - this->last_update_time_;
  if (dt < 1.0 / this->update_rate_)
  {
    return;
  }
  this->last_update_time_ = current_time;
  
  // Récupération de la distance minimale mesurée
  double min_distance = this->max_range_;
  
  // Parcours de tous les rayons du capteur
  for (unsigned int i = 0; i < this->ray_sensor_->RayCount(); ++i)
  {
    for (unsigned int j = 0; j < this->ray_sensor_->RangeCount(); ++j)
    {
      double range = this->ray_sensor_->Range(i, j);
      
      // Ignorer les valeurs invalides
      if (std::isnan(range) || std::isinf(range))
      {
        continue;
      }
      
      // Prendre la distance minimale (premier obstacle)
      if (range < min_distance && range >= this->min_range_)
      {
        min_distance = range;
      }
    }
  }
  
  // Si aucune détection valide, utiliser max_range
  if (min_distance >= this->max_range_)
  {
    min_distance = this->max_range_;
  }
  
  // Ajout du bruit
  min_distance = this->AddNoise(min_distance);
  
  // Clamp dans la plage valide
  min_distance = this->ClampRange(min_distance);
  
  // Publication
  this->PublishRange(min_distance);
  
  // Traitement des callbacks ROS2
  rclcpp::spin_some(this->ros_node_);
}

// ==========================================
// AJOUT DE BRUIT
// ==========================================

double UltrasonicSensorPlugin::AddNoise(double distance)
{
  // Bruit gaussien additif
  double noise = this->noise_distribution_(this->generator_);
  double noisy_distance = distance + noise;
  
  // Simulation des limitations du HC-SR04:
  // - Résolution limitée à ~3mm
  // - Erreur systématique près des limites
  if (distance < 0.05)
  {
    noisy_distance += 0.003;  // Offset proche de 5cm
  }
  
  return noisy_distance;
}

// ==========================================
// CLAMP RANGE
// ==========================================

double UltrasonicSensorPlugin::ClampRange(double distance)
{
  if (distance < this->min_range_)
  {
    return this->min_range_;
  }
  if (distance > this->max_range_)
  {
    return this->max_range_;
  }
  return distance;
}

// ==========================================
// PUBLICATION ROS2
// ==========================================

void UltrasonicSensorPlugin::PublishRange(double distance)
{
  // Mise à jour du header
  this->range_msg_.header.stamp = rclcpp::Clock().now();
  
  // Mise à jour de la distance
  this->range_msg_.range = distance;
  
  // Publication
  this->range_pub_->publish(this->range_msg_);
}

