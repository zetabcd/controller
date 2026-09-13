#include <px4ctrl/px4_native_position_control.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4NativePositionControl>());
  rclcpp::shutdown();
  return 0;
}
