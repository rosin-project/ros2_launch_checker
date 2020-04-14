//
// Created by snuc on 6/3/19.
//

#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
using namespace std::chrono_literals;

class Stresser {
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tb;
  rclcpp::TimerBase::SharedPtr tmr;

  double pub_freq;
  size_t n_links;
  std::string frame_prefix;

  void do_pub_tf() {
    std::vector<geometry_msgs::msg::TransformStamped> tfs;

    for (size_t i = 0; i < n_links; i++) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.frame_id = (i == 0) ? "base_link" : "link_" + frame_prefix + "_" + std::to_string(i);
      tf.header.stamp = node->get_clock()->now();
      tf.child_frame_id = "link_" + frame_prefix + "_" + std::to_string(i + 1);
      tf.transform.rotation.w = cos(i);
      tf.transform.rotation.x = 0;
      tf.transform.rotation.y = 0;
      tf.transform.rotation.z = sin(i);
      tf.transform.translation.x = i;
      tf.transform.translation.y = i;
      tf.transform.translation.z = i;
      tfs.emplace_back(std::move(tf));
    }

    tb->sendTransform(tfs);
  };

public:
  Stresser() {
    node = std::make_shared<rclcpp::Node>("stresser");
    RCLCPP_INFO(node->get_logger(), "creating node");
  auto qos = rclcpp::QoS(10);
    // pub_tf = node->create_publisher<tf2_msgs::msg::TFMessage>("tf", rclcpp::QoS(100));
    tb = std::make_shared<tf2_ros::TransformBroadcaster>(node, qos);
    std::cout<< qos.get_rmw_qos_profile().deadline.nsec <<" , " <<  qos.get_rmw_qos_profile().deadline.sec;
    pub_freq = node->declare_parameter("frequency", 1000.0);
    RCLCPP_INFO(node->get_logger(), "publish frequency %f", pub_freq);

    n_links = node->declare_parameter("n_links", 8);
    tmr = node->create_wall_timer(std::chrono::duration<double>(1 / pub_freq), [this]() { this->do_pub_tf(); });
  }

  void spin() {
    while (rclcpp::ok())
      rclcpp::spin(node);
  }
};

int main(int argc, char **argv) {
  setbuf(stdout, nullptr);

  rclcpp::init(argc, argv);
  Stresser s;
  s.spin();
  rclcpp::shutdown();
  return 0;
}