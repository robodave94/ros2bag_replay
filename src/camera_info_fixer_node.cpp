// camera_info_fixer_node.cpp
//
// ROS 2 node that republishes a bag's CameraInfo message with corrected header
// fields so that RViz2 DepthCloud can time-sync it with the depth image stream.
//
// Problem:
//   RealSense bags (and others) often record CameraInfo with a zero timestamp
//   and/or an empty frame_id.  RViz2's DepthCloud display requires the
//   CameraInfo stamp to match (within a small tolerance) the depth image stamp,
//   and the frame_id to match the depth sensor's TF frame.  Without this fix the
//   depth cloud does not render.
//
// Solution:
//   Subscribe to the bag's CameraInfo topic with a TRANSIENT_LOCAL (latching)
//   QoS — so the single stored message is received even if published before this
//   node started.  On each incoming depth Image, copy the latched CameraInfo,
//   patch in the image stamp and the configured frame_id, then republish.
//
// Parameters:
//   camera_info_in  (string)  Source CameraInfo topic from the bag.
//                             Default: /device_0/sensor_0/Depth_0/info/camera_info
//   depth_image_in  (string)  Depth Image topic (stamp source only).
//                             Default: /device_0/sensor_0/Depth_0/image/data
//   camera_info_out (string)  Republished, corrected CameraInfo topic.
//                             Default: /viewer/depth_info
//   frame_id        (string)  frame_id to inject into the outgoing CameraInfo.
//                             Must match the frame_id in the depth image header.
//                             Default: "0"

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

using CameraInfo = sensor_msgs::msg::CameraInfo;
using Image      = sensor_msgs::msg::Image;

namespace ros2bag_replay {

class CameraInfoFixerNode : public rclcpp::Node {
 public:
  explicit CameraInfoFixerNode(
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("camera_info_fixer", options)
  {
    // ── Declare parameters ─────────────────────────────────────────────────
    declare_parameter<std::string>(
        "camera_info_in",
        "/device_0/sensor_0/Depth_0/info/camera_info");
    declare_parameter<std::string>(
        "depth_image_in",
        "/device_0/sensor_0/Depth_0/image/data");
    declare_parameter<std::string>(
        "camera_info_out",
        "/viewer/depth_info");
    declare_parameter<std::string>(
        "frame_id",
        "0");

    // ── Read parameters ────────────────────────────────────────────────────
    const std::string ci_in  = get_parameter("camera_info_in").as_string();
    const std::string img_in = get_parameter("depth_image_in").as_string();
    const std::string ci_out = get_parameter("camera_info_out").as_string();
    frame_id_ = get_parameter("frame_id").as_string();

    // ── QoS profiles ───────────────────────────────────────────────────────
    // TRANSIENT_LOCAL / RELIABLE: bag player republishes the one latched
    // CameraInfo at connection time, so we catch it even if subscribed late.
    rclcpp::QoS latch_qos(rclcpp::KeepLast(1));
    latch_qos.transient_local().reliable();

    // VOLATILE / BEST_EFFORT: matches the typical QoS of bag-replayed images.
    rclcpp::QoS img_qos(rclcpp::KeepLast(5));
    img_qos.best_effort().durability_volatile();

    // ── Subscriptions ──────────────────────────────────────────────────────
    ci_sub_ = create_subscription<CameraInfo>(
        ci_in,
        latch_qos,
        [this](const CameraInfo::SharedPtr msg) {
          latched_info_ = msg;
          RCLCPP_INFO_ONCE(get_logger(), "CameraInfo received and latched.");
        });

    img_sub_ = create_subscription<Image>(
        img_in,
        img_qos,
        [this](const Image::SharedPtr msg) { onImage(msg); });

    // ── Publisher ──────────────────────────────────────────────────────────
    ci_pub_ = create_publisher<CameraInfo>(ci_out, latch_qos);

    RCLCPP_INFO(get_logger(),
                "camera_info_fixer: %s + %s -> %s (frame_id='%s')",
                ci_in.c_str(), img_in.c_str(),
                ci_out.c_str(), frame_id_.c_str());
  }

 private:
  void onImage(const Image::SharedPtr & img_msg)
  {
    if (!latched_info_) {
      // CameraInfo not yet received; nothing to republish.
      return;
    }

    // Shallow-copy the latched CameraInfo and patch the header.
    CameraInfo out = *latched_info_;
    out.header.stamp    = img_msg->header.stamp;
    out.header.frame_id = frame_id_;
    ci_pub_->publish(out);
  }

  // ── State ─────────────────────────────────────────────────────────────────
  CameraInfo::SharedPtr latched_info_;
  std::string           frame_id_;

  // ── ROS interfaces ────────────────────────────────────────────────────────
  rclcpp::Subscription<CameraInfo>::SharedPtr ci_sub_;
  rclcpp::Subscription<Image>::SharedPtr      img_sub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr    ci_pub_;
};

}  // namespace ros2bag_replay

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<ros2bag_replay::CameraInfoFixerNode>());
  rclcpp::shutdown();
  return 0;
}
