// src/cloud_merger_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <memory>
#include <string>
#include <vector>

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointXYZI   = pcl::PointXYZI;
using CloudXYZI   = pcl::PointCloud<PointXYZI>;

class CloudMergerNode : public rclcpp::Node
{
public:
  CloudMergerNode() : Node("cloud_merger_node")
  {
    declareParameters();
    initTf();
    waitForTfTree();
    setupSubscribers();
    setupPublisher();

    RCLCPP_INFO(this->get_logger(), "Cloud merger node ready! Merging via front lidar → lidar_link");
  }

private:
  void declareParameters()
  {
    this->declare_parameter<std::string>("destination_frame", "lidar_link");
    this->declare_parameter<std::string>("input_cloud_1", "/velodyne_points1"); // back
    this->declare_parameter<std::string>("input_cloud_2", "/velodyne_points2"); // front
    this->declare_parameter<std::string>("merged_cloud", "/merged_cloud");
    this->declare_parameter<double>("tf_timeout", 0.1);
    this->declare_parameter<int>("sync_queue_size", 10);
    this->declare_parameter<bool>("use_static_tf", true);

    destination_frame_ = this->get_parameter("destination_frame").as_string();
    input_cloud_1_      = this->get_parameter("input_cloud_1").as_string();
    input_cloud_2_      = this->get_parameter("input_cloud_2").as_string();
    merged_topic_       = this->get_parameter("merged_cloud").as_string();
    tf_timeout_         = this->get_parameter("tf_timeout").as_double();
    sync_queue_size_    = this->get_parameter("sync_queue_size").as_int();
    use_static_tf_      = this->get_parameter("use_static_tf").as_bool();

    RCLCPP_INFO(this->get_logger(),
                "Params → dest:%s  in1:%s (back)  in2:%s (front)  static:%s",
                destination_frame_.c_str(),
                input_cloud_1_.c_str(),
                input_cloud_2_.c_str(),
                use_static_tf_ ? "true" : "false");
  }

  void initTf()
  {
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void waitForTfTree()
  {
    rclcpp::Rate rate(2.0);
    const double max_wait = 30.0;
    auto start = this->now();

    RCLCPP_INFO(this->get_logger(), "Waiting for TF tree (max %.0fs)…", max_wait);

    while (rclcpp::ok())
    {
      try {
        rclcpp::Time t = use_static_tf_ ? rclcpp::Time(0) : this->now();

        // Kiểm tra frame đích
        tf_buffer_->lookupTransform(destination_frame_, destination_frame_, t,
                                    rclcpp::Duration::from_seconds(tf_timeout_));

        // Kiểm tra cả hai LiDAR
        tf_buffer_->lookupTransform(destination_frame_, "velodyne_points2", t,
                                    rclcpp::Duration::from_seconds(tf_timeout_));
        tf_buffer_->lookupTransform(destination_frame_, "velodyne_points1", t,
                                    rclcpp::Duration::from_seconds(tf_timeout_));

        // Kiểm tra chuỗi: back → front
        tf_buffer_->lookupTransform("velodyne_points2", "velodyne_points1", t,
                                    rclcpp::Duration::from_seconds(tf_timeout_));

        RCLCPP_INFO(this->get_logger(), "TF tree FULLY ready (including back→front chain)!");
        return;
      }
      catch (const tf2::TransformException & ex) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "Waiting for TF chain: %s", ex.what());
      }

      rate.sleep();
      if ((this->now() - start).seconds() > max_wait) {
        RCLCPP_WARN(this->get_logger(), "TF timeout – continuing anyway.");
        break;
      }
    }
  }

  void setupSubscribers()
  {
    auto qos = rclcpp::SensorDataQoS();

    cloud_sub1_.subscribe(this, input_cloud_1_, qos.get_rmw_qos_profile()); // back
    cloud_sub2_.subscribe(this, input_cloud_2_, qos.get_rmw_qos_profile()); // front

    using Policy = message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>;
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(
        Policy(sync_queue_size_), cloud_sub1_, cloud_sub2_);

    sync_->registerCallback(std::bind(&CloudMergerNode::callback, this,
                                      std::placeholders::_1,  // back
                                      std::placeholders::_2)); // front

    RCLCPP_INFO(this->get_logger(), "Subscribers + sync ready");
  }

  void setupPublisher()
  {
    auto qos = rclcpp::QoS(10).reliable();
    pub_ = this->create_publisher<PointCloud2>(merged_topic_, qos);
    RCLCPP_INFO(this->get_logger(), "Publishing merged cloud on: %s", merged_topic_.c_str());
  }

  void callback(const PointCloud2::ConstSharedPtr & c_back,  // velodyne_points1
                const PointCloud2::ConstSharedPtr & c_front) // velodyne_points2
  {
    PointCloud2 t_back, t_front;

    // 1. Front: trực tiếp → lidar_link
    if (!transformDirect(*c_front, t_front, "velodyne_points2")) return;

    // 2. Back: back → front → lidar_link
    if (!transformViaFront(*c_back, t_back)) return;

    // Merge
    CloudXYZI::Ptr pcl_back(new CloudXYZI), pcl_front(new CloudXYZI), merged(new CloudXYZI);
    pcl::fromROSMsg(t_back, *pcl_back);
    pcl::fromROSMsg(t_front, *pcl_front);
    *merged = *pcl_back + *pcl_front;

    PointCloud2 out;
    pcl::toROSMsg(*merged, out);
    out.header.frame_id = destination_frame_;
    out.header.stamp = c_front->header.stamp;

    pub_->publish(out);

    static rclcpp::Time last_log = this->now();
    if ((this->now() - last_log).seconds() > 2.0) {
      RCLCPP_INFO(this->get_logger(),
                  "Merged %zu (back) + %zu (front) → %zu pts",
                  pcl_back->size(), pcl_front->size(), merged->size());
      last_log = this->now();
    }
  }

  // Transform trực tiếp: source → destination_frame
  bool transformDirect(const PointCloud2 & in, PointCloud2 & out, const std::string & source_frame)
  {
    if (in.header.frame_id.empty()) return false;

    try {
      rclcpp::Time t = use_static_tf_ ? rclcpp::Time(0) : rclcpp::Time(in.header.stamp);
      auto tf = tf_buffer_->lookupTransform(destination_frame_, source_frame, t,
                                            rclcpp::Duration::from_seconds(tf_timeout_));
      pcl_ros::transformPointCloud(destination_frame_, tf, in, out);
      out.header.frame_id = destination_frame_;
      out.header.stamp = in.header.stamp;
      return true;
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Direct TF failed %s→%s: %s", source_frame.c_str(),
                           destination_frame_.c_str(), ex.what());
      return false;
    }
  }

  // Transform QUA FRONT: back → front → destination
  bool transformViaFront(const PointCloud2 & in, PointCloud2 & out)
  {
    if (in.header.frame_id.empty()) return false;

    try {
      rclcpp::Time t = use_static_tf_ ? rclcpp::Time(0) : rclcpp::Time(in.header.stamp);

      // B1: back → front
      auto tf_back_to_front = tf_buffer_->lookupTransform(
          "velodyne_points2", in.header.frame_id, t,
          rclcpp::Duration::from_seconds(tf_timeout_));

      PointCloud2 temp;
      pcl_ros::transformPointCloud("velodyne_points2", tf_back_to_front, in, temp);

      // B2: front → lidar_link
      auto tf_front_to_dest = tf_buffer_->lookupTransform(
          destination_frame_, "velodyne_points2", t,
          rclcpp::Duration::from_seconds(tf_timeout_));

      pcl_ros::transformPointCloud(destination_frame_, tf_front_to_dest, temp, out);

      out.header.frame_id = destination_frame_;
      out.header.stamp = in.header.stamp;

      RCLCPP_DEBUG(this->get_logger(), "Back lidar transformed via front → %s", destination_frame_.c_str());
      return true;
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Back→Front→Dest TF chain failed: %s", ex.what());
      return false;
    }
  }

  // Members
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  message_filters::Subscriber<PointCloud2> cloud_sub1_; // back
  message_filters::Subscriber<PointCloud2> cloud_sub2_; // front

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;

  std::string destination_frame_;
  std::string input_cloud_1_, input_cloud_2_, merged_topic_;
  double tf_timeout_;
  int sync_queue_size_;
  bool use_static_tf_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudMergerNode>());
  rclcpp::shutdown();
  return 0;
}