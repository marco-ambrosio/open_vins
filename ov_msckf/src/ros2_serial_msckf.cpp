/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "ros/ROS2Visualizer.h"
#include "utils/dataset_reader.h"

using namespace ov_msckf;

namespace {

struct SerializedBagMessage {
  std::string topic;
  double bag_time = 0.0;
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized;
};

void open_bag_reader(rosbag2_cpp::Reader &reader, const std::string &bag_path, const std::string &bag_storage_id,
                     const std::string &bag_serialization_format) {
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;
  storage_options.storage_id = bag_storage_id;

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = bag_serialization_format;
  converter_options.output_serialization_format = bag_serialization_format;

  reader.open(storage_options, converter_options);
}

double bag_time_to_sec(int64_t time_stamp) { return 1e-9 * static_cast<double>(time_stamp); }

template <typename MessageT> std::shared_ptr<MessageT> deserialize_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &msg) {
  auto out = std::make_shared<MessageT>();
  rclcpp::Serialization<MessageT> serialization;
  rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
  serialization.deserialize_message(&serialized_msg, out.get());
  return out;
}

} // namespace

std::shared_ptr<VioManager> sys;
std::shared_ptr<ROS2Visualizer> viz;

// Main function
int main(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  std::string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }

  // Launch our ros node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("ros2_serial_msckf", options);
  node->get_parameter<std::string>("config_path", config_path);

  // Load the config
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  parser->set_node(node);

  // Verbosity
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create our VIO system
  VioManagerOptions params;
  params.print_and_load(parser);
  // params.num_opencv_threads = 0; // uncomment if you want repeatability
  // params.use_multi_threading_pubs = 0; // uncomment if you want repeatability
  params.use_multi_threading_subs = false;
  sys = std::make_shared<VioManager>(params);
  viz = std::make_shared<ROS2Visualizer>(node, sys);

  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT_ERROR(RED "[SERIAL]: unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Our imu topic
  std::string topic_imu = "/imu0";
  node->get_parameter("topic_imu", topic_imu);
  parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);
  PRINT_DEBUG("[SERIAL]: imu: %s\n", topic_imu.c_str());

  // Our camera topics
  std::vector<std::string> topic_cameras;
  for (int i = 0; i < params.state_options.num_cameras; i++) {
    std::string cam_topic = "/cam" + std::to_string(i) + "/image_raw";
    node->get_parameter("topic_camera" + std::to_string(i), cam_topic);
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", cam_topic);
    topic_cameras.emplace_back(cam_topic);
    PRINT_DEBUG("[SERIAL]: cam: %s\n", cam_topic.c_str());
  }

  // Location of the ROS bag we want to read in
  std::string path_to_bag = "/home/patrick/datasets/eth/V1_01_easy";
  node->get_parameter("path_bag", path_to_bag);
  PRINT_DEBUG("[SERIAL]: ros bag path is: %s\n", path_to_bag.c_str());

  // rosbag2 requires a storage backend and serialization format.
  // We default to the standard sqlite3/cdr layout, but these can be overridden for other rosbag2 setups.
  std::string bag_storage_id = "sqlite3";
  std::string bag_serialization_format = "cdr";
  node->get_parameter("bag_storage_id", bag_storage_id);
  node->get_parameter("bag_serialization_format", bag_serialization_format);

  // Load groundtruth if we have it
  // NOTE: needs to be a csv ASL format file
  std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
  std::string path_to_gt = "";
  if (node->get_parameter("path_gt", path_to_gt) && !path_to_gt.empty()) {
    ov_core::DatasetReader::load_gt_file(path_to_gt, gt_states);
    PRINT_DEBUG("[SERIAL]: gt file path is: %s\n", path_to_gt.c_str());
  }

  // Get our start location and how much of the bag we want to play
  // Make the bag duration < 0 to just process to the end of the bag
  double bag_start = 0.0;
  double bag_durr = -1.0;
  node->get_parameter("bag_start", bag_start);
  node->get_parameter("bag_durr", bag_durr);
  PRINT_DEBUG("[SERIAL]: bag start: %.1f\n", bag_start);
  PRINT_DEBUG("[SERIAL]: bag duration: %.1f\n", bag_durr);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Open the bag once to determine its full time range
  double time_begin = -1;
  double time_end = -1;
  try {
    rosbag2_cpp::Reader bag_info_reader;
    open_bag_reader(bag_info_reader, path_to_bag, bag_storage_id, bag_serialization_format);
    while (bag_info_reader.has_next()) {
      auto msg = bag_info_reader.read_next();
      double msg_time = bag_time_to_sec(msg->time_stamp);
      if (time_begin < 0) {
        time_begin = msg_time;
      }
      time_end = msg_time;
    }
  } catch (const std::exception &e) {
    PRINT_ERROR(RED "[SERIAL]: Unable to open rosbag2 bag: %s\n" RESET, path_to_bag.c_str());
    PRINT_ERROR(RED "[SERIAL]: %s\n" RESET, e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  if (time_begin < 0 || time_end < 0) {
    PRINT_ERROR(RED "[SERIAL]: No messages found in the rosbag2 bag. Exiting.\n" RESET);
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  double time_init = time_begin + bag_start;
  double time_finish = (bag_durr < 0) ? time_end : time_init + bag_durr;
  PRINT_DEBUG("time start = %.6f\n", time_init);
  PRINT_DEBUG("time end   = %.6f\n", time_finish);

  // Open the bag again and collect all relevant messages for random access synchronization
  double max_camera_time = -1;
  std::vector<SerializedBagMessage> msgs;
  try {
    rosbag2_cpp::Reader bag_reader;
    open_bag_reader(bag_reader, path_to_bag, bag_storage_id, bag_serialization_format);

    std::unordered_map<std::string, std::string> topic_types;
    for (const auto &topic : bag_reader.get_all_topics_and_types()) {
      topic_types[topic.name] = topic.type;
    }

    if (topic_types.find(topic_imu) != topic_types.end() && topic_types.at(topic_imu) != "sensor_msgs/msg/Imu") {
      PRINT_ERROR(RED "[SERIAL]: IMU topic has unmatched message types!!\n" RESET);
      PRINT_ERROR(RED "[SERIAL]: Supports: sensor_msgs/msg/Imu\n" RESET);
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }
    for (const auto &topic_camera : topic_cameras) {
      if (topic_types.find(topic_camera) != topic_types.end() && topic_types.at(topic_camera) != "sensor_msgs/msg/Image") {
        PRINT_ERROR(RED "[SERIAL]: Image topic has unmatched message types!!\n" RESET);
        PRINT_ERROR(RED "[SERIAL]: Supports: sensor_msgs/msg/Image\n" RESET);
        rclcpp::shutdown();
        return EXIT_FAILURE;
      }
    }

    while (bag_reader.has_next()) {
      auto msg = bag_reader.read_next();
      double msg_time = bag_time_to_sec(msg->time_stamp);
      if (msg_time < time_init) {
        continue;
      }
      if (msg_time > time_finish) {
        break;
      }
      if (msg->topic_name == topic_imu) {
        msgs.push_back({msg->topic_name, msg_time, msg});
        continue;
      }
      for (int i = 0; i < params.state_options.num_cameras; i++) {
        if (msg->topic_name == topic_cameras.at(i)) {
          msgs.push_back({msg->topic_name, msg_time, msg});
          max_camera_time = std::max(max_camera_time, msg_time);
          break;
        }
      }
    }
  } catch (const std::exception &e) {
    PRINT_ERROR(RED "[SERIAL]: Unable to read rosbag2 bag: %s\n" RESET, path_to_bag.c_str());
    PRINT_ERROR(RED "[SERIAL]: %s\n" RESET, e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  PRINT_DEBUG("[SERIAL]: total of %zu messages!\n", msgs.size());

  // Check to make sure we have data to play
  if (msgs.empty()) {
    PRINT_ERROR(RED "[SERIAL]: No messages to play on specified topics. Exiting.\n" RESET);
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Loop through our message array, and lets process them
  std::set<int> used_index;
  for (int m = 0; m < (int)msgs.size(); m++) {

    // End once we reach the last time, or skip if before beginning time (shouldn't happen)
    if (!rclcpp::ok() || msgs.at(m).bag_time > time_finish || msgs.at(m).bag_time > max_camera_time)
      break;
    if (msgs.at(m).bag_time < time_init)
      continue;

    // Skip messages that we have already used
    if (used_index.find(m) != used_index.end()) {
      used_index.erase(m);
      continue;
    }

    // IMU processing
    if (msgs.at(m).topic == topic_imu) {
      viz->callback_inertial(deserialize_message<sensor_msgs::msg::Imu>(msgs.at(m).serialized));
    }

    // Camera processing
    for (int cam_id = 0; cam_id < params.state_options.num_cameras; cam_id++) {

      // Skip if this message is not a camera topic
      if (msgs.at(m).topic != topic_cameras.at(cam_id))
        continue;

      // We have a matching camera topic here, now find the other cameras for this time
      // For each camera, we will find the nearest timestamp (within 0.02sec) that is greater than the current
      // If we are unable, then this message should just be skipped since it isn't a sync'ed pair!
      std::map<int, int> camid_to_msg_index;
      double meas_time = msgs.at(m).bag_time;
      for (int cam_idt = 0; cam_idt < params.state_options.num_cameras; cam_idt++) {
        if (cam_idt == cam_id) {
          camid_to_msg_index.insert({cam_id, m});
          continue;
        }
        int cam_idt_idx = -1;
        for (int mt = m; mt < (int)msgs.size(); mt++) {
          if (msgs.at(mt).topic != topic_cameras.at(cam_idt))
            continue;
          if (std::abs(msgs.at(mt).bag_time - meas_time) < 0.02)
            cam_idt_idx = mt;
          break;
        }
        if (cam_idt_idx != -1) {
          camid_to_msg_index.insert({cam_idt, cam_idt_idx});
        }
      }

      // Skip processing if we were unable to find any messages
      if ((int)camid_to_msg_index.size() != params.state_options.num_cameras) {
        PRINT_DEBUG(YELLOW "[SERIAL]: Unable to find stereo pair for message %d at %.2f into bag (will skip!)\n" RESET, m,
                    meas_time - time_init);
        continue;
      }

      // Check if we should initialize using the groundtruth
      Eigen::Matrix<double, 17, 1> imustate;
      if (!gt_states.empty() && !sys->initialized() && ov_core::DatasetReader::get_gt_state(meas_time, imustate, gt_states)) {
        // biases are pretty bad normally, so zero them
        // imustate.block(11,0,6,1).setZero();
        sys->initialize_with_gt(imustate);
      }

      // Pass our data into our visualizer callbacks!
      if (params.state_options.num_cameras == 1) {
        viz->callback_monocular(deserialize_message<sensor_msgs::msg::Image>(msgs.at(camid_to_msg_index.at(0)).serialized), 0);
      } else if (params.state_options.num_cameras == 2) {
        auto msg0 = deserialize_message<sensor_msgs::msg::Image>(msgs.at(camid_to_msg_index.at(0)).serialized);
        auto msg1 = deserialize_message<sensor_msgs::msg::Image>(msgs.at(camid_to_msg_index.at(1)).serialized);
        used_index.insert(camid_to_msg_index.at(0)); // skip this message
        used_index.insert(camid_to_msg_index.at(1)); // skip this message
        viz->callback_stereo(msg0, msg1, 0, 1);
      } else {
        PRINT_ERROR(RED "[SERIAL]: We currently only support 1 or 2 camera serial input....\n" RESET);
        rclcpp::shutdown();
        return EXIT_FAILURE;
      }

      break;
    }
  }

  // Final visualization
  viz->visualize_final();
  rclcpp::shutdown();

  // Done!
  return EXIT_SUCCESS;
}
