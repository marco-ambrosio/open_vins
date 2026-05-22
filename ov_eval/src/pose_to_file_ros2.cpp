#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <fstream>
#include <stdexcept>
#include <string>

namespace {

double stamp_to_sec(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

class PoseRecorderNode : public rclcpp::Node {
public:
  PoseRecorderNode() : Node("pose_to_file"), has_covariance_(false) {
    const std::string verbosity = this->declare_parameter<std::string>("verbosity", "INFO");
    const std::string topic = this->declare_parameter<std::string>("topic", "");
    const std::string topic_type = this->declare_parameter<std::string>("topic_type", "");
    const std::string output = this->declare_parameter<std::string>("output", "");

    if (topic.empty() || topic_type.empty() || output.empty()) {
      RCLCPP_FATAL(this->get_logger(), "Parameters topic, topic_type, and output must all be provided.");
      throw std::runtime_error("missing required parameters");
    }

    (void)verbosity;
    initialize_output(output);
    initialize_subscription(topic, topic_type);
  }

private:
  void initialize_output(const std::string &output) {
    boost::filesystem::path file_path(output.c_str());
    if (!file_path.parent_path().empty() && boost::filesystem::create_directories(file_path.parent_path())) {
      RCLCPP_INFO(this->get_logger(), "Created folder path to output file: %s", file_path.parent_path().c_str());
    }
    if (boost::filesystem::exists(output)) {
      RCLCPP_WARN(this->get_logger(), "Output file exists, deleting old file.");
      boost::filesystem::remove(output);
    }

    outfile_.open(output.c_str());
    if (outfile_.fail()) {
      throw std::runtime_error("unable to open output file");
    }
    outfile_ << "# timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33" << std::endl;
  }

  void initialize_subscription(const std::string &topic, const std::string &topic_type) {
    if (topic_type == "PoseWithCovarianceStamped") {
      pose_cov_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          topic, 1000, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            timestamp_ = stamp_to_sec(msg->header.stamp);
            q_ItoG_ << msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;
            p_IinG_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
            cov_pos_ << msg->pose.covariance.at(0), msg->pose.covariance.at(1), msg->pose.covariance.at(2), msg->pose.covariance.at(6),
                msg->pose.covariance.at(7), msg->pose.covariance.at(8), msg->pose.covariance.at(12), msg->pose.covariance.at(13),
                msg->pose.covariance.at(14);
            cov_rot_ << msg->pose.covariance.at(21), msg->pose.covariance.at(22), msg->pose.covariance.at(23), msg->pose.covariance.at(27),
                msg->pose.covariance.at(28), msg->pose.covariance.at(29), msg->pose.covariance.at(33), msg->pose.covariance.at(34),
                msg->pose.covariance.at(35);
            has_covariance_ = true;
            write();
          });
    } else if (topic_type == "PoseStamped") {
      pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          topic, 1000, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            timestamp_ = stamp_to_sec(msg->header.stamp);
            q_ItoG_ << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
            p_IinG_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
            has_covariance_ = false;
            write();
          });
    } else if (topic_type == "TransformStamped") {
      transform_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
          topic, 1000, [this](const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
            timestamp_ = stamp_to_sec(msg->header.stamp);
            q_ItoG_ << msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w;
            p_IinG_ << msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z;
            has_covariance_ = false;
            write();
          });
    } else if (topic_type == "Odometry") {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          topic, 1000, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            timestamp_ = stamp_to_sec(msg->header.stamp);
            q_ItoG_ << msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;
            p_IinG_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
            cov_pos_ << msg->pose.covariance.at(0), msg->pose.covariance.at(1), msg->pose.covariance.at(2), msg->pose.covariance.at(6),
                msg->pose.covariance.at(7), msg->pose.covariance.at(8), msg->pose.covariance.at(12), msg->pose.covariance.at(13),
                msg->pose.covariance.at(14);
            cov_rot_ << msg->pose.covariance.at(21), msg->pose.covariance.at(22), msg->pose.covariance.at(23), msg->pose.covariance.at(27),
                msg->pose.covariance.at(28), msg->pose.covariance.at(29), msg->pose.covariance.at(33), msg->pose.covariance.at(34),
                msg->pose.covariance.at(35);
            has_covariance_ = true;
            write();
          });
    } else {
      throw std::runtime_error("unsupported topic_type: " + topic_type);
    }
  }

  void write() {
    outfile_.precision(5);
    outfile_.setf(std::ios::fixed, std::ios::floatfield);
    outfile_ << timestamp_ << " ";

    outfile_.precision(6);
    outfile_ << p_IinG_.x() << " " << p_IinG_.y() << " " << p_IinG_.z() << " " << q_ItoG_(0) << " " << q_ItoG_(1) << " " << q_ItoG_(2) << " "
             << q_ItoG_(3);

    if (has_covariance_) {
      outfile_.precision(10);
      outfile_ << " " << cov_rot_(0, 0) << " " << cov_rot_(0, 1) << " " << cov_rot_(0, 2) << " " << cov_rot_(1, 1) << " " << cov_rot_(1, 2)
               << " " << cov_rot_(2, 2) << " " << cov_pos_(0, 0) << " " << cov_pos_(0, 1) << " " << cov_pos_(0, 2) << " " << cov_pos_(1, 1)
               << " " << cov_pos_(1, 2) << " " << cov_pos_(2, 2) << std::endl;
    } else {
      outfile_ << std::endl;
    }
  }

  std::ofstream outfile_;
  bool has_covariance_;
  double timestamp_{-1.0};
  Eigen::Vector4d q_ItoG_{Eigen::Vector4d(0, 0, 0, 1)};
  Eigen::Vector3d p_IinG_{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, 3, 3> cov_rot_{Eigen::Matrix<double, 3, 3>::Zero()};
  Eigen::Matrix<double, 3, 3> cov_pos_{Eigen::Matrix<double, 3, 3>::Zero()};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr transform_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PoseRecorderNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("pose_to_file"), "%s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
