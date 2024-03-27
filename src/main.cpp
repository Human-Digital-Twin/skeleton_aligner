#include "skeleton_aligner/Aligner.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hiros::hdt::Aligner>());
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
