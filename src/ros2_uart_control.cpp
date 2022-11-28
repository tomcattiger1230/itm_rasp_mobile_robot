/*
 * @Author: Wei Luo
 * @Date: 2022-09-08 14:31:07
 * @LastEditors: Wei Luo
 * @LastEditTime: 2022-11-28 10:20:11
 * @Note: this cpp is to set up UART communication with BeagleBone onboard
 * computer
 */

#include <itm_rasp_mobile_robot/ros2_uart_control.hpp>

UartControl::UartControl()
    : Node("uart_control_node"), is_initialized(false),
      got_velocity_cmd(false) {
  /* declare several ros parameters */
  this->declare_parameter<std::string>("port", "/dev/ttyS0");
  this->get_parameter("port", port_name_);

  this->declare_parameter<int>("baudrate", 115200);
  this->get_parameter("baudrate", baud_rate);

  this->declare_parameter<std::string>("ros_cmd_topic", "/robot_cmd");
  this->get_parameter("ros_cmd_topic", cmd_topic_name);

  /* This is a subscription to the topic `/robot_cmd` and the callback function is
  `robot_velocity_cmd_callback`. */
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_topic_name, 10,
      std::bind(&UartControl::robot_velocity_cmd_callback, this,
                std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "robot gets ros2 topic is: %s",
              cmd_topic_name.c_str());

  /* This is to set up the serial port. */
  serial::Timeout to = serial::Timeout::simpleTimeout(100);
  sp.setPort(port_name_);
  sp.setBaudrate(baud_rate);
  sp.setTimeout(to);

  try {
    sp.open();

  } catch (serial::IOException &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open port");
  }

  if (sp.isOpen()) {
    RCLCPP_INFO_STREAM(this->get_logger(), port_name_ + " is opened");
    is_initialized = true;
  }
}

UartControl::~UartControl() {}

/**
 * It takes the velocity command from the ROS topic and stores it in the array `received_cmd`
 *
 * @param msg The message that was received.
 */
void UartControl::robot_velocity_cmd_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {

  if (!got_velocity_cmd) {
    got_velocity_cmd = true;
    RCLCPP_INFO_ONCE(this->get_logger(), "Got the first velocity command.");
  }
  received_cmd[0] = msg->twist.linear.x;
  received_cmd[1] = msg->twist.linear.y;
  received_cmd[2] = msg->twist.angular.z;
}

/**
 * It checks if a new velocity command has been received, and if so, it sends it to the Raspberry Pi
 */
void UartControl::run() {
  if (got_velocity_cmd) {
    std::string cmd_string_;
    for (int i = 0; i < 3; i++) {
      cmd_string_ += std::to_string(received_cmd[i]);
      cmd_string_ += std::string(";");
    }
    cmd_string_ += "\n";
    sp.write(cmd_string_);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  /* To set the frequency of the loop. */
  rclcpp::Rate rate(50.0);

  auto uart_control_handle = std::make_shared<UartControl>();
  while (rclcpp::ok()) {
    if (uart_control_handle->is_initialized) {
      uart_control_handle->run();
    } else {
      break;
    }
    rclcpp::spin_some(uart_control_handle);
    rate.sleep();
  }
  RCLCPP_INFO(uart_control_handle->get_logger(), "Serial port is closed.");
  return 0;
}