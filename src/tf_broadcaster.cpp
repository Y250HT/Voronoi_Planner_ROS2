#include <memory>
#include <mutex>
#include <thread>
#include <chrono>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_voronoi_planner {
class TfBroadcaster : public rclcpp::Node {
public:
    TfBroadcaster(const std::string & node_name) : rclcpp::Node(node_name) {}
    virtual ~TfBroadcaster();

    void Initialize();

private:
    void SetStart(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr start);
    void PublishTransform();
    void PublishLoop(double transform_publish_period);

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    geometry_msgs::msg::PoseStamped start_;
    bool start_received_ = false;
    std::mutex mutex_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;
    std::unique_ptr<std::thread> transform_thread_ = nullptr;
};

TfBroadcaster::~TfBroadcaster() {
    if (transform_thread_ != nullptr) {
        transform_thread_->join();
    }
}

void TfBroadcaster::Initialize() {
    start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 1, std::bind(&TfBroadcaster::SetStart, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    double transform_publish_period = this->declare_parameter("transform_publish_period", 0.05);
    transform_thread_ = std::make_unique<std::thread>([this, transform_publish_period] {
        PublishLoop(transform_publish_period);
    });
}

void TfBroadcaster::PublishTransform() {
  std::lock_guard<std::mutex> lock(mutex_);

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = "base_link";

  if (start_received_) {
    transform_stamped.transform.translation.x = start_.pose.position.x;
    transform_stamped.transform.translation.y = start_.pose.position.y;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation = start_.pose.orientation;
  } else {
    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;
  }

  tf_broadcaster_->sendTransform(transform_stamped);
}

void TfBroadcaster::PublishLoop(double transform_publish_period) {
  if (transform_publish_period <= 0.0) {
    return;
  }

  rclcpp::Rate r(1.0 / transform_publish_period);
  while (rclcpp::ok()) {
    PublishTransform();
    r.sleep();
  }
}

void TfBroadcaster::SetStart(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr start) {
  std::lock_guard<std::mutex> lock(mutex_);

  RCLCPP_INFO(this->get_logger(), "A new start is received.");
  start_.header = start->header;
  start_.pose = start->pose.pose;
  start_received_ = true;
}

}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<nav2_voronoi_planner::TfBroadcaster>("tf_broadcaster");
  node->Initialize();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}