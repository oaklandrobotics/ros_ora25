#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"

class ReinitControlNode : public rclcpp::Node
{
public:
  ReinitControlNode() : Node("reinit_control")
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&ReinitControlNode::joy_callback, this, std::placeholders::_1)
    );

    reinit_pub_ = this->create_publisher<std_msgs::msg::Empty>("/odrive/reinit", 1000);
    estop_pub_ = this->create_publisher<std_msgs::msg::Empty>("/odrive/estop", 1000);

    RCLCPP_INFO(this->get_logger(), "ReinitControlNode initialized");
  }

private:
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
          reinit_pub_->publish(std_msgs::msg::Empty());

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
          estop_pub_->publish(std_msgs::msg::Empty());

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
