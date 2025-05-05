#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

class ReinitControlNode : public rclcpp::Node
{
public:
  ReinitControlNode() : Node("reinit_control")
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&ReinitControlNode::joy_callback, this, std::placeholders::_1)
    );

    reinit_client_ = this->create_client<std_srvs::srv::Trigger>("/odrive/reinit");
    estop_client_ = this->create_client<std_srvs::srv::Trigger>("/odrive/estop");

    RCLCPP_INFO(this->get_logger(), "ReinitControlNode initialized");
  }

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reinit_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr estop_client_;

  bool reinitPress = false;
  bool estopPress = false;

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!msg->buttons.empty())
    {
      // Reinit Odrive ros2 control
      if (msg->buttons[0])
      {
        // Rising Edge
        if (!reinitPress)
        {
          RCLCPP_INFO(this->get_logger(), "Reinit button pressed");
          
          if (reinit_client_->wait_for_service(std::chrono::seconds(1)))
          {
            // Send trigger as a request to the service
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto result = reinit_client_->async_send_request(req);
          }

          reinitPress = true;
        }
      }
      else
      {
        reinitPress = false;
      }

      // Estop Odrive
      if (msg->buttons[1])
      {
        // Rising Edge 
        if (!estopPress)
        {
          RCLCPP_INFO(this->get_logger(), "Estop button pressed");
          
          if (estop_client_->wait_for_service(std::chrono::seconds(1)))
          {
            // Send trigger as a request to the service
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto result = estop_client_->async_send_request(req);
          }

          estopPress = true;
        }
      }
      else
      {
        estopPress = false;
      }

    }
    else
    {
      reinitPress = false;
      estopPress = false;
    }
  }


  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reinit_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr estop_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReinitControlNode>());
    rclcpp::shutdown();
    return 0;
}
