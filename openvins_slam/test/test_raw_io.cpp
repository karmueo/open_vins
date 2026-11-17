#include "openvins_slam/raw_io.hpp"
#include "openvins_slam/mcap_io.hpp"
#include "openvins_slam/output_paths.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <unistd.h>

namespace {
namespace fs = std::filesystem;

class RawIoTest : public ::testing::Test {
protected:
  void SetUp() override {
    char path[] = "/tmp/openvins-raw-test-XXXXXX";
    auto *created = ::mkdtemp(path);
    ASSERT_NE(created, nullptr);
    directory_ = created;
  }
  void TearDown() override { fs::remove_all(directory_); }

  void write_file(const fs::path &path, const std::string &content) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out.write(content.data(), static_cast<std::streamsize>(content.size()));
    ASSERT_TRUE(out.good());
  }

  fs::path directory_;
};

TEST_F(RawIoTest, AlignedLayoutUsesCamTimeNsAndMergesChronologically) {
  const fs::path root = directory_ / "test1";
  write_file(root / "images" / "frame_000001_stc_NA_ts_NA.jpg", "JPG1");
  write_file(root / "images" / "frame_000002_stc_999999_ts_888888.jpg", "JPG2");
  write_file(root / "aligned" / "image_timestamps.txt",
             "frame_index,image_file,ts_ns,stc_ns,cam_time_ns\r\n"
             "1,frame_000001_stc_NA_ts_NA.jpg,-1,-1,1000\r\n"
             "2,frame_000002_stc_999999_ts_888888.jpg,888888,999999,2000\r\n");
  write_file(root / "aligned" / "imu.csv",
             "time_ns,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,acc_x_m_s2,acc_y_m_s2,acc_z_m_s2\r\n"
             "500,0.1,0.2,0.3,0.0,9.8,0.0\r\n"
             "1000,0.4,0.5,0.6,0.1,9.7,0.1\r\n"
             "2500,0.7,0.8,0.9,0.2,9.6,0.2\r\n");

  openvins_slam::RawDataset dataset(root.string());
  EXPECT_EQ(dataset.image_count(), 2u);
  EXPECT_EQ(dataset.imu_count(), 3u);

  std::vector<std::string> events;
  dataset.for_each(
      [&](const openvins_slam::RawImuSample &s) {
        events.push_back("imu:" + std::to_string(s.time_ns));
        if (s.time_ns == 1000) {
          EXPECT_DOUBLE_EQ(s.gyro.x(), 0.4);
          EXPECT_DOUBLE_EQ(s.accel.y(), 9.7);
        }
        return true;
      },
      [&](const openvins_slam::RawImageEntry &img, std::vector<uint8_t> &&bytes, const std::string &format) {
        events.push_back("img:" + std::to_string(img.cam_time_ns) + ":" + format + ":" +
                         std::string(bytes.begin(), bytes.end()));
        return true;
      });

  EXPECT_EQ(events, (std::vector<std::string>{
                        "imu:500",
                        "imu:1000",
                        "img:1000:jpeg:JPG1",
                        "img:2000:jpeg:JPG2",
                        "imu:2500",
                    }));
}

TEST_F(RawIoTest, FlatLayoutSupportsReorderedColumnsAndPngFormat) {
  const fs::path root = directory_ / "flat";
  write_file(root / "images" / "a.png", "PNGDATA");
  write_file(root / "image_timestamps.txt",
             "cam_time_ns,image_file\n"
             "5000,a.png\n");
  write_file(root / "imu.csv",
             "time_ns,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,acc_x_m_s2,acc_y_m_s2,acc_z_m_s2\n"
             "4000,0,0,0,0,0,9.8\n");

  openvins_slam::RawDataset dataset(root.string());
  std::string observed_format;
  dataset.for_each(
      [](const openvins_slam::RawImuSample &) { return true; },
      [&](const openvins_slam::RawImageEntry &img, std::vector<uint8_t> &&bytes, const std::string &format) {
        observed_format = format;
        EXPECT_EQ(img.cam_time_ns, 5000);
        EXPECT_EQ(std::string(bytes.begin(), bytes.end()), "PNGDATA");
        return true;
      });
  EXPECT_EQ(observed_format, "png");
}

TEST_F(RawIoTest, RejectsMissingFilesTraversalAndBadTimestamps) {
  EXPECT_THROW((openvins_slam::RawDataset((directory_ / "missing").string())), std::runtime_error);

  const fs::path bad = directory_ / "bad";
  write_file(bad / "images" / "ok.jpg", "x");
  write_file(bad / "aligned" / "imu.csv",
             "time_ns,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,acc_x_m_s2,acc_y_m_s2,acc_z_m_s2\n"
             "100,0,0,0,0,0,9.8\n");

  // Missing referenced image
  write_file(bad / "aligned" / "image_timestamps.txt",
             "frame_index,image_file,ts_ns,stc_ns,cam_time_ns\n"
             "1,missing.jpg,-1,-1,100\n");
  EXPECT_THROW((openvins_slam::RawDataset(bad.string())), std::runtime_error);

  // Path traversal
  write_file(directory_ / "outside.jpg", "x");
  write_file(bad / "aligned" / "image_timestamps.txt",
             "frame_index,image_file,ts_ns,stc_ns,cam_time_ns\n"
             "1,../outside.jpg,-1,-1,100\n");
  EXPECT_THROW((openvins_slam::RawDataset(bad.string())), std::runtime_error);

  // Non-monotonic camera timestamps
  write_file(bad / "aligned" / "image_timestamps.txt",
             "frame_index,image_file,ts_ns,stc_ns,cam_time_ns\n"
             "1,ok.jpg,-1,-1,200\n"
             "2,ok.jpg,-1,-1,200\n");
  EXPECT_THROW((openvins_slam::RawDataset(bad.string())), std::runtime_error);

  // Valid timestamps, non-monotonic IMU
  write_file(bad / "aligned" / "image_timestamps.txt",
             "frame_index,image_file,ts_ns,stc_ns,cam_time_ns\n"
             "1,ok.jpg,-1,-1,200\n");
  write_file(bad / "aligned" / "imu.csv",
             "time_ns,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,acc_x_m_s2,acc_y_m_s2,acc_z_m_s2\n"
             "100,0,0,0,0,0,9.8\n"
             "100,0,0,0,0,0,9.8\n");
  EXPECT_THROW((openvins_slam::RawDataset(bad.string())), std::runtime_error);
}

TEST_F(RawIoTest, StandaloneMcapOutputWritesExpectedQos) {
  const std::string output_path = (directory_ / "standalone.mcap").string();
  {
    openvins_slam::McapOutput output(output_path);
    std_msgs::msg::String message;
    message.data = "sensor";
    rclcpp::SerializedMessage serialized;
    rclcpp::Serialization<std_msgs::msg::String>().serialize_message(&message, &serialized);
    output.write_serialized("/imu", "std_msgs/msg/String", serialized, 100, false);
    output.write_serialized("/stereo/image_raw", "std_msgs/msg/String", serialized, 200, false);
    output.write_serialized("/openvins/odom", "std_msgs/msg/String", serialized, 200, true);
    tf2_msgs::msg::TFMessage tf_msg;
    rclcpp::SerializedMessage tf_serialized;
    rclcpp::Serialization<tf2_msgs::msg::TFMessage>().serialize_message(&tf_msg, &tf_serialized);
    output.write_serialized("/tf", "tf2_msgs/msg/TFMessage", tf_serialized, 200);
    output.write_serialized("/tf_static", "tf2_msgs/msg/TFMessage", tf_serialized, 200);
    output.commit();
  }

  rosbag2_storage::StorageOptions storage;
  storage.uri = output_path;
  storage.storage_id = "mcap";
  rosbag2_cpp::Reader reader;
  reader.open(storage);
  std::size_t count = 0;
  for (const auto &topic : reader.get_all_topics_and_types()) {
    ++count;
    ASSERT_EQ(topic.offered_qos_profiles.size(), 1u) << topic.name;
    const auto &qos = topic.offered_qos_profiles.front().get_rmw_qos_profile();
    if (topic.name == "/imu" || topic.name == "/stereo/image_raw" || topic.name == "/tf") {
      EXPECT_EQ(qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE) << topic.name;
      EXPECT_EQ(qos.depth, 100u) << topic.name;
    } else {
      EXPECT_EQ(qos.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) << topic.name;
      EXPECT_EQ(qos.depth, 1u) << topic.name;
    }
  }
  EXPECT_EQ(count, 5u);
}

TEST(RawOutputPaths, HandlesTrailingSlashesAndDotsInDirectoryName) {
  const auto paths = openvins_slam::raw_output_paths("/tmp/results", "/data/test1/", "slam_from_bag", true);
  EXPECT_EQ(paths.trajectory, "/tmp/results/openvins_test1_slam_from_bag_trajectory.txt");
  EXPECT_EQ(paths.mcap, "/tmp/results/openvins_test1_slam_from_bag.mcap");

  const auto dotted = openvins_slam::raw_output_paths("/tmp/results", "/data/test1.v2/", "", false);
  EXPECT_EQ(dotted.trajectory, "/tmp/results/openvins_test1.v2__trajectory.txt");
  EXPECT_TRUE(dotted.mcap.empty());

  EXPECT_THROW(openvins_slam::raw_output_paths("/tmp/results", "/", "cfg", false), std::runtime_error);
}

} // namespace
