#include "skeleton_aligner/Aligner.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hdt::skeleton::Aligner>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
