#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace gazebo
{
  class EncoderPlugin : public ModelPlugin
  {
    public:
      EncoderPlugin() : pulses_per_rev_(600), last_angle_(0.0), 
                       pulses_(0), update_rate_(100.0), last_update_(0.0) {}
      
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
      {
        model_ = _model;
        
        std::string joint_name;
        if (_sdf->HasElement("joint_name"))
          joint_name = _sdf->Get<std::string>("joint_name");
        else
        {
          gzerr << "EncoderPlugin: joint_name required\n";
          return;
        }
        
        joint_ = model_->GetJoint(joint_name);
        if (!joint_)
        {
          gzerr << "EncoderPlugin: Joint '" << joint_name << "' not found\n";
          return;
        }
        
        if (_sdf->HasElement("pulses_per_revolution"))
          pulses_per_rev_ = _sdf->Get<int>("pulses_per_revolution");
        if (_sdf->HasElement("update_rate"))
          update_rate_ = _sdf->Get<double>("update_rate");
        
        std::string topic = joint_name + "/encoder";
        if (_sdf->HasElement("topic_name"))
          topic = _sdf->Get<std::string>("topic_name");
        
        if (!rclcpp::ok())
          rclcpp::init(0, nullptr);
        
        ros_node_ = rclcpp::Node::make_shared("encoder_plugin");
        encoder_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>(topic, 10);
        
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
          [this](const common::UpdateInfo& _info) {
            double current_time = _info.simTime.Double();
            if (current_time - last_update_ < 1.0 / update_rate_)
              return;
            last_update_ = current_time;
            
            double current_angle = joint_->Position(0);
            double delta_angle = current_angle - last_angle_;
            last_angle_ = current_angle;
            
            // Conversion angle -> pulses
            int delta_pulses = static_cast<int>(delta_angle / (2.0 * M_PI) * pulses_per_rev_);
            pulses_ += delta_pulses;
            
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = rclcpp::Clock().now();
            msg.name.push_back(joint_->GetName());
            msg.position.push_back(current_angle);
            msg.velocity.push_back(joint_->GetVelocity(0));
            
            // Données encodeur
            msg.effort.push_back(static_cast<double>(pulses_));  // Position en pulses
            
            encoder_pub_->publish(msg);
            rclcpp::spin_some(ros_node_);
          }
        );
      }
      
    private:
      physics::ModelPtr model_;
      physics::JointPtr joint_;
      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoder_pub_;
      event::ConnectionPtr update_connection_;
      int pulses_per_rev_, pulses_;
      double last_angle_, update_rate_, last_update_;
  };
  
  GZ_REGISTER_MODEL_PLUGIN(EncoderPlugin)
}

