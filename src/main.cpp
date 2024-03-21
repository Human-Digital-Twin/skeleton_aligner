#include "skeleton_fusion/Filter.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hiros::hdt::Filter>());
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
