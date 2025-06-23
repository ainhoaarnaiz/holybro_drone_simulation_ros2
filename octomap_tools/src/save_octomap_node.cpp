#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <memory>

class OctomapSaver : public rclcpp::Node {
public:
  OctomapSaver() : Node("octomap_saver") {
    subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_binary", 10,
      std::bind(&OctomapSaver::callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Waiting for OctoMap on /octomap_binary...");
  }

private:
  void callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received OctoMap, converting...");

    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    if (tree) {
      auto octree = dynamic_cast<octomap::OcTree*>(tree);
      if (octree) {
        std::string path = "/home/ainhoaarnaiz/test.bt";
        octree->writeBinary(path);
        RCLCPP_INFO(this->get_logger(), "Saved OctoMap to %s", path.c_str());

        // Shut down after saving
        rclcpp::shutdown();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to cast to OcTree");
      }
      delete tree;  // Free memory
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert OctoMap message");
    }
  }

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OctomapSaver>();
  rclcpp::spin(node);
  rclcpp::shutdown();  // Will be called again safely
  return 0;
}
