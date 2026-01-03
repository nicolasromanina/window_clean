#ifndef IMU_MPU6050_PLUGIN_H
#define IMU_MPU6050_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <random>

namespace gazebo
{
  /**
   * @brief Plugin Gazebo pour simuler un IMU MPU6050 (6 axes)
   * 
   * Caractéristiques:
   * - Gyroscope 3 axes (±250/500/1000/2000 °/s)
   * - Accéléromètre 3 axes (±2/4/8/16 g)
   * - Dérive simulée (drift gyroscopique)
   * - Bruit gaussien réaliste
   */
  class ImuMpu6050Plugin : public SensorPlugin
  {
    public:
      ImuMpu6050Plugin();
      virtual ~ImuMpu6050Plugin();
      virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
      virtual void OnUpdate();
      
    private:
      sensors::ImuSensorPtr imu_sensor_;
      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
      sensor_msgs::msg::Imu imu_msg_;
      
      std::string topic_name_;
      std::string frame_name_;
      double update_rate_;
      double gyro_noise_mean_;
      double gyro_noise_stddev_;
      double accel_noise_mean_;
      double accel_noise_stddev_;
      
      std::default_random_engine generator_;
      std::normal_distribution<double> gyro_noise_;
      std::normal_distribution<double> accel_noise_;
      
      // Dérive gyroscopique (bias qui évolue lentement)
      ignition::math::Vector3d gyro_bias_;
      double last_update_time_;
      event::ConnectionPtr update_connection_;
      
      void SetupROS();
      ignition::math::Vector3d AddNoise(ignition::math::Vector3d value, 
                                       std::normal_distribution<double>& dist);
      void UpdateGyroBias(double dt);
  };
  
  GZ_REGISTER_SENSOR_PLUGIN(ImuMpu6050Plugin)
}

#endif

