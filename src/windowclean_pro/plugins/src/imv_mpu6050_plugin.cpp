#include "imv_mpu6050_plugin.h"
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

using namespace gazebo;

ImuMpu6050Plugin::ImuMpu6050Plugin()
  : update_rate_(100.0)
  , gyro_noise_mean_(0.0)
  , gyro_noise_stddev_(0.00017)  // ~0.01 °/s
  , accel_noise_mean_(0.0)
  , accel_noise_stddev_(0.00196)  // ~0.12 m/s²
  , last_update_time_(0.0)
{
  gyro_bias_ = ignition::math::Vector3d::Zero;
}

ImuMpu6050Plugin::~ImuMpu6050Plugin()
{
}

void ImuMpu6050Plugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  this->imu_sensor_ = std::dynamic_pointer_cast<sensors::ImuSensor>(_sensor);
  if (!this->imu_sensor_)
  {
    gzerr << "ImuMpu6050Plugin: Sensor is not an ImuSensor\n";
    return;
  }
  
  if (_sdf->HasElement("topic_name"))
    this->topic_name_ = _sdf->Get<std::string>("topic_name");
  else
    this->topic_name_ = "imu/data";
  
  if (_sdf->HasElement("frame_name"))
    this->frame_name_ = _sdf->Get<std::string>("frame_name");
  else
    this->frame_name_ = this->imu_sensor_->ParentName();
  
  if (_sdf->HasElement("update_rate"))
    this->update_rate_ = _sdf->Get<double>("update_rate");
  
  if (_sdf->HasElement("gyro_noise_stddev"))
    this->gyro_noise_stddev_ = _sdf->Get<double>("gyro_noise_stddev");
  
  if (_sdf->HasElement("accel_noise_stddev"))
    this->accel_noise_stddev_ = _sdf->Get<double>("accel_noise_stddev");
  
  std::random_device rd;
  this->generator_ = std::default_random_engine(rd());
  this->gyro_noise_ = std::normal_distribution<double>(gyro_noise_mean_, gyro_noise_stddev_);
  this->accel_noise_ = std::normal_distribution<double>(accel_noise_mean_, accel_noise_stddev_);
  
  this->imu_sensor_->SetActive(true);
  this->SetupROS();
  
  this->update_connection_ = this->imu_sensor_->ConnectUpdated(
    std::bind(&ImuMpu6050Plugin::OnUpdate, this)
  );
}

void ImuMpu6050Plugin::SetupROS()
{
  if (!rclcpp::ok())
  {
    int argc = 0;
    char** argv = NULL;
    rclcpp::init(argc, argv);
  }
  
  this->ros_node_ = rclcpp::Node::make_shared("imu_mpu6050_plugin");
  this->imu_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::Imu>(this->topic_name_, 10);
  
  this->imu_msg_.header.frame_id = this->frame_name_;
}

void ImuMpu6050Plugin::OnUpdate()
{
  double current_time = this->imu_sensor_->LastUpdateTime().Double();
  double dt = current_time - this->last_update_time_;
  
  if (dt < 1.0 / this->update_rate_)
    return;
  
  this->last_update_time_ = current_time;
  
  // Mise à jour du biais gyroscopique (dérive)
  this->UpdateGyroBias(dt);
  
  // Récupération des données brutes
  ignition::math::Vector3d angular_vel = this->imu_sensor_->AngularVelocity();
  ignition::math::Vector3d linear_accel = this->imu_sensor_->LinearAcceleration();
  ignition::math::Quaterniond orientation = this->imu_sensor_->Orientation();
  
  // Ajout du bruit et du biais
  angular_vel = this->AddNoise(angular_vel, this->gyro_noise_);
  angular_vel += this->gyro_bias_;
  
  linear_accel = this->AddNoise(linear_accel, this->accel_noise_);
  
  // Publication
  this->imu_msg_.header.stamp = rclcpp::Clock().now();
  
  this->imu_msg_.orientation.x = orientation.X();
  this->imu_msg_.orientation.y = orientation.Y();
  this->imu_msg_.orientation.z = orientation.Z();
  this->imu_msg_.orientation.w = orientation.W();
  
  this->imu_msg_.angular_velocity.x = angular_vel.X();
  this->imu_msg_.angular_velocity.y = angular_vel.Y();
  this->imu_msg_.angular_velocity.z = angular_vel.Z();
  
  this->imu_msg_.linear_acceleration.x = linear_accel.X();
  this->imu_msg_.linear_acceleration.y = linear_accel.Y();
  this->imu_msg_.linear_acceleration.z = linear_accel.Z();
  
  this->imu_pub_->publish(this->imu_msg_);
  
  rclcpp::spin_some(this->ros_node_);
}

ignition::math::Vector3d ImuMpu6050Plugin::AddNoise(
  ignition::math::Vector3d value,
  std::normal_distribution<double>& dist)
{
  return ignition::math::Vector3d(
    value.X() + dist(this->generator_),
    value.Y() + dist(this->generator_),
    value.Z() + dist(this->generator_)
  );
}

void ImuMpu6050Plugin::UpdateGyroBias(double dt)
{
  // Dérive lente du gyroscope (random walk)
  double drift_rate = 0.00001;  // rad/s²
  std::normal_distribution<double> drift(drift_rate, drift_rate * 0.1);
  
  this->gyro_bias_.X() += drift(this->generator_) * dt;
  this->gyro_bias_.Y() += drift(this->generator_) * dt;
  this->gyro_bias_.Z() += drift(this->generator_) * dt;
  
  // Limitation du biais
  double max_bias = 0.001;  // rad/s max
  if (this->gyro_bias_.Length() > max_bias)
  {
    this->gyro_bias_ = this->gyro_bias_.Normalize() * max_bias;
  }
}

