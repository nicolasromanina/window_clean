#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace gazebo
{
  class BatteryPlugin : public ModelPlugin
  {
    public:
      BatteryPlugin() : charge_(100.0), capacity_(7.4), voltage_(14.8), 
                       current_draw_(2.5), update_rate_(10.0), last_update_(0.0) {}
      
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
      {
        model_ = _model;
        
        if (_sdf->HasElement("initial_charge"))
          charge_ = _sdf->Get<double>("initial_charge");
        if (_sdf->HasElement("capacity"))
          capacity_ = _sdf->Get<double>("capacity");
        if (_sdf->HasElement("voltage"))
          voltage_ = _sdf->Get<double>("voltage");
        if (_sdf->HasElement("current_draw"))
          current_draw_ = _sdf->Get<double>("current_draw");
        if (_sdf->HasElement("update_rate"))
          update_rate_ = _sdf->Get<double>("update_rate");
        
        std::string topic = "battery/state";
        if (_sdf->HasElement("topic_name"))
          topic = _sdf->Get<std::string>("topic_name");
        
        if (!rclcpp::ok())
          rclcpp::init(0, nullptr);
        
        ros_node_ = rclcpp::Node::make_shared("battery_plugin");
        battery_pub_ = ros_node_->create_publisher<sensor_msgs::msg::BatteryState>(topic, 10);
        
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
          [this](const common::UpdateInfo& _info) {
            double current_time = _info.simTime.Double();
            if (current_time - last_update_ < 1.0 / update_rate_)
              return;
            last_update_ = current_time;
            
            // Décharge selon le courant consommé
            double dt = 1.0 / update_rate_;
            double energy_consumed = current_draw_ * voltage_ * dt / 3600.0;  // Wh
            charge_ -= (energy_consumed / (capacity_ * voltage_)) * 100.0;
            charge_ = std::max(0.0, charge_);
            
            // Dépendance tension-charge
            double charge_ratio = charge_ / 100.0;
            double actual_voltage = voltage_ * (0.9 + 0.1 * charge_ratio);
            
            sensor_msgs::msg::BatteryState msg;
            msg.header.stamp = rclcpp::Clock().now();
            msg.voltage = actual_voltage;
            msg.percentage = charge_;
            msg.current = -current_draw_;
            msg.present = charge_ > 5.0;
            
            battery_pub_->publish(msg);
            rclcpp::spin_some(ros_node_);
          }
        );
      }
      
    private:
      physics::ModelPtr model_;
      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
      event::ConnectionPtr update_connection_;
      double charge_, capacity_, voltage_, current_draw_, update_rate_, last_update_;
  };
  
  GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin)
}

