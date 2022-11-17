/*
 * @Author: Wei Luo
 * @Date: 2022-11-16 16:25:17
 * @LastEditors: Wei Luo
 * @LastEditTime: 2022-11-17 09:43:15
 * @Note: Note
 */

#include <itm_rasp_mobile_robot/robot_serial_cmd.hpp>

SerialCMD::SerialCMD(const ros::NodeHandle &nh,
                     const ros::NodeHandle &private_nh)
    : nh_(nh), private_nh_(private_nh), is_initialized(false),
      got_velocity_cmd(false) {
  /* get some parameters */
  if (private_nh_.hasParam(serial_port))
    private_nh_.getParam("serial_port", serial_port);
  else
    serial_port = "/dev/ttyUSB0";

  if (private_nh_.hasParam("baudrate"))
    private_nh_.getParam("baudrate", baud_rate);
  else
    baud_rate = 115200;

  robot_cmd_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
      "/robot_cmd", 10, &SerialCMD::robot_velocity_cmd_callback, this);

  serial::Timeout to = serial::Timeout::simpleTimeout(100);
  sp.setPort(serial_port);
  sp.setBaudrate(baud_rate);
  sp.setTimeout(to);

  try {
    sp.open();

  } catch (serial::IOException &e) {
    ROS_ERROR_STREAM("Failed to open port");
  }

  if (sp.isOpen()) {
    ROS_INFO_STREAM(serial_port + "is opened");
    is_initialized = true;
  }
}

SerialCMD::~SerialCMD() {}

void SerialCMD::robot_velocity_cmd_callback(
    const geometry_msgs::TwistStamped::ConstPtr &v_cmd) {
  if (!got_velocity_cmd) {
    got_velocity_cmd = true;
    ROS_INFO_ONCE("Got the first velocity command.");
  }

  received_cmd[0] = v_cmd->twist.linear.x;
  received_cmd[1] = v_cmd->twist.linear.y;
  received_cmd[2] = v_cmd->twist.angular.z;
}

void SerialCMD::run() {
  if (got_velocity_cmd) {
    // convert velocity command to string format
    std::string cmd_string_;
    for (int i = 0; i < 3; i++) {
        cmd_string_ += std::to_string(received_cmd[i]);
        cmd_string_ += ";";
    }
    ROS_INFO("Message send to serial: %s", cmd_string_.c_str());

    std::vector<uint8_t> cmd_string_vector_(cmd_string_.begin(), cmd_string_.end());
    uint8_t *cmd_p = &cmd_string_vector_[0];
    sp.write(cmd_p, sizeof(cmd_p));
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "RobotCMDSerial");
  ros::NodeHandle nh_node, private_nh_node("~");

  ros::Rate rate(100);
  SerialCMD serial_obj(nh_node, private_nh_node);

  while (ros::ok()) {
    if (serial_obj.is_initialized) {
      serial_obj.run();
    } else {
      break;
    }
    rate.sleep();

  }
  ROS_INFO("Serial port is closed");
  return 0;
}