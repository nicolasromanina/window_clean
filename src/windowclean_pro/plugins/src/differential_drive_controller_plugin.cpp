#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace gazebo
{
  class DifferentialDriveControllerPlugin : public ModelPlugin
  {
    public:
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
      {
        model_ = _model;
        world_ = _model->GetWorld();
        
        // Lecture des paramètres
        std::string left_joint_name = _sdf->Get<std::string>("left_joint");
        std::string right_joint_name = _sdf->Get<std::string>("right_joint");
        wheel_separation_ = _sdf->Get<double>("wheel_separation");
        wheel_diameter_ = _sdf->Get<double>("wheel_diameter");
        max_wheel_torque_ = _sdf->Get<double>("max_wheel_torque", 50.0).first;
        
        left_joint_ = model_->GetJoint(left_joint_name);
        right_joint_ = model_->GetJoint(right_joint_name);
        
        if (!left_joint_ || !right_joint_)
        {
          gzerr << "DifferentialDriveControllerPlugin: Joints not found\n";
          return;
        }
        
        std::string cmd_topic = _sdf->Get<std::string>("command_topic", "cmd_vel").first;
        std::string odom_topic = _sdf->Get<std::string>("odometry_topic", "odom").first;
        std::string odom_frame = _sdf->Get<std::string>("odometry_frame", "odom").first;
        std::string base_frame = _sdf->Get<std::string>("robot_base_frame", "base_link").first;
        double update_rate = _sdf->Get<double>("update_rate", 50.0).first;
        
        if (!rclcpp::ok())
          rclcpp::init(0, nullptr);
        
        ros_node_ = rclcpp::Node::make_shared("differential_drive_controller");
        cmd_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
          cmd_topic, 10,
          [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            cmd_vel_ = *msg;
          }
        );
        
        odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);
        
        odom_frame_id_ = odom_frame;
        base_frame_id_ = base_frame;
        
        // Initialisation odométrie
        x_ = y_ = theta_ = 0.0;
        last_update_time_ = 0.0;
        
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
          [this, update_rate](const common::UpdateInfo& _info) {
            double current_time = _info.simTime.Double();
            double dt = current_time - last_update_time_;
            
            if (dt < 1.0 / update_rate)
              return;
            
            last_update_time_ = current_time;
            
            // Calcul des vitesses de roues
            double linear = cmd_vel_.linear.x;
            double angular = cmd_vel_.angular.z;
            
            double left_vel = linear - (angular * wheel_separation_ / 2.0);
            double right_vel = linear + (angular * wheel_separation_ / 2.0);
            
            // Conversion en vitesse angulaire
            double left_angular_vel = left_vel / (wheel_diameter_ / 2.0);
            double right_angular_vel = right_vel / (wheel_diameter_ / 2.0);
            
            // Application des couples
            left_joint_->SetVelocity(0, left_angular_vel);
            right_joint_->SetVelocity(0, right_angular_vel);
            
            // Mise à jour odométrie
            double v = (left_vel + right_vel) / 2.0;
            double w = (right_vel - left_vel) / wheel_separation_;
            
            x_ += v * cos(theta_) * dt;
            y_ += v * sin(theta_) * dt;
            theta_ += w * dt;
            
            // Normalisation de theta
            while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
            while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
            
            // Publication odométrie
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = rclcpp::Clock().now();
            odom.header.frame_id = odom_frame_id_;
            odom.child_frame_id = base_frame_id_;
            
            odom.pose.pose.position.x = x_;
            odom.pose.pose.position.y = y_;
            odom.pose.pose.orientation.z = sin(theta_ / 2.0);
            odom.pose.pose.orientation.w = cos(theta_ / 2.0);
            
            odom.twist.twist.linear.x = v;
            odom.twist.twist.angular.z = w;
            
            odom_pub_->publish(odom);
            
            rclcpp::spin_some(ros_node_);
          }
        );
      }
      
    private:
      physics::ModelPtr model_;
      physics::WorldPtr world_;
      physics::JointPtr left_joint_, right_joint_;
      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
      std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
      geometry_msgs::msg::Twist cmd_vel_;
      
      double wheel_separation_, wheel_diameter_, max_wheel_torque_;
      std::string odom_frame_id_, base_frame_id_;
      double x_, y_, theta_, last_update_time_;
      event::ConnectionPtr update_connection_;
  };
  
  GZ_REGISTER_MODEL_PLUGIN(DifferentialDriveControllerPlugin)
}

