#include "openvins_slam/mcap_io.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <mcap/writer.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <unistd.h>

namespace {
class McapIoTest : public ::testing::Test {
protected:
  void SetUp() override {
    char path[] = "/tmp/openvins-mcap-test-XXXXXX";
    auto *created = ::mkdtemp(path);
    ASSERT_NE(created, nullptr);
    directory_ = created;
    input_path_ = directory_ + "/input.mcap";
    output_path_ = directory_ + "/output.mcap";
  }
  void TearDown() override { std::filesystem::remove_all(directory_); }

  void make_input(bool with_index = true) {
    mcap::McapWriter writer;
    mcap::McapWriterOptions options("ros2");
    options.compression = mcap::Compression::None;
    options.chunkSize = 50;
    options.noMessageIndex = !with_index;
    ASSERT_TRUE(writer.open(input_path_, options).ok());
    // The original file can contain two distinct schema IDs with the same name.
    mcap::Schema first("test_msgs/msg/Foo", "ros2msg", "uint8 field\n");
    mcap::Schema second = first;
    writer.addSchema(first);
    writer.addSchema(second);
    ASSERT_NE(first.id, second.id);
    mcap::Channel a("/source_a", "cdr", first.id, {{"key", "value"}});
    mcap::Channel b("/source_b", "cdr", second.id);
    writer.addChannel(a);
    writer.addChannel(b);
    std::byte payload[2] = {std::byte{0x01}, std::byte{0x02}};
    for (int i : {3, 1, 4, 2}) {
      mcap::Message msg{};
      msg.channelId = i % 2 ? a.id : b.id;
      msg.sequence = i;
      msg.logTime = i * 100;
      msg.publishTime = i * 100 + 7;
      msg.dataSize = 2;
      msg.data = payload;
      ASSERT_TRUE(writer.write(msg).ok());
    }
    writer.close();
  }
  std::string directory_, input_path_, output_path_;
};

TEST_F(McapIoTest, CopiesUnmodifiedRecordsAndAddsStandardSchemaInSortedReadOrder) {
  make_input();
  {
    openvins_slam::McapInput input(input_path_);
    openvins_slam::McapOutput output(output_path_, input_path_, input);
    std::vector<uint64_t> times;
    input.for_each([&](const mcap::MessageView &view) {
      times.push_back(view.message.logTime);
      output.write_original(view);
      return true;
    });
    EXPECT_EQ(times, (std::vector<uint64_t>{100, 200, 300, 400}));
    std_msgs::msg::String message;
    message.data = "hello";
    rclcpp::SerializedMessage serialized;
    rclcpp::Serialization<std_msgs::msg::String> serializer;
    serializer.serialize_message(&message, &serialized);
    output.write_serialized("/new", "std_msgs/msg/String", serialized, 150);
    EXPECT_THROW(output.write_serialized("/source_a", "std_msgs/msg/String", serialized, 150), std::runtime_error);
    output.commit();
  }
  openvins_slam::McapInput result(output_path_);
  EXPECT_EQ(result.reader().schemas().size(), 3u);
  int original = 0, appended = 0;
  result.for_each([&](const mcap::MessageView &view) {
    if (view.channel->topic == "/new") {
      ++appended;
      EXPECT_EQ(view.schema->name, "std_msgs/msg/String");
    } else {
      ++original;
      EXPECT_EQ(view.message.dataSize, 2u);
      EXPECT_EQ(view.message.data[0], std::byte{0x01});
      EXPECT_EQ(view.message.data[1], std::byte{0x02});
      EXPECT_EQ(view.message.publishTime, view.message.logTime + 7);
      EXPECT_EQ(view.message.sequence, view.message.logTime / 100);
      if (view.channel->topic == "/source_a")
        EXPECT_EQ(view.channel->metadata.at("key"), "value");
    }
    return true;
  });
  EXPECT_EQ(original, 4);
  EXPECT_EQ(appended, 1);
}

TEST_F(McapIoTest, CreatesMissingParentDirectoriesAndOverwritesExistingOutput) {
  make_input();
  {
    std::ofstream(output_path_) << "original";
  }
  {
    openvins_slam::McapInput input(input_path_);
    openvins_slam::McapOutput output(output_path_, input_path_, input);
    input.for_each([&](const mcap::MessageView &view) {
      output.write_original(view);
      return true;
    });
    std::ifstream before(output_path_);
    std::string content;
    before >> content;
    EXPECT_EQ(content, "original");
    {
      std::ofstream concurrent(output_path_);
      concurrent << "concurrent";
    }
    EXPECT_NO_THROW(output.commit());
  }
  {
    openvins_slam::McapInput result(output_path_);
    int messages = 0;
    result.for_each([&](const mcap::MessageView &view) {
      ++messages;
      return true;
    });
    EXPECT_EQ(messages, 4);
  }

  const auto nested = directory_ + "/nested/deeper/output.mcap";
  {
    openvins_slam::McapInput input(input_path_);
    openvins_slam::McapOutput output(nested, input_path_, input);
    input.for_each([&](const mcap::MessageView &view) {
      output.write_original(view);
      return true;
    });
    output.commit();
  }
  EXPECT_TRUE(std::filesystem::is_regular_file(nested));
  for (const auto &entry : std::filesystem::directory_iterator(directory_))
    EXPECT_EQ(entry.path().string().find(".partial."), std::string::npos);
}

TEST_F(McapIoTest, RosbagStorageReadsMatchingPlaybackQosForAllOutputChannels) {
  make_input();
  {
    openvins_slam::McapInput input(input_path_);
    openvins_slam::McapOutput output(output_path_, input_path_, input);
    input.for_each([&](const mcap::MessageView &view) {
      output.write_original(view);
      return true;
    });

    std_msgs::msg::String message;
    rclcpp::SerializedMessage serialized;
    rclcpp::Serialization<std_msgs::msg::String>().serialize_message(&message, &serialized);
    for (const char *topic : {"/openvins/pose_imu", "/openvins/pose_cam0", "/openvins/pose_cam1",
                              "/openvins/path_imu", "/openvins/path_cam0", "/openvins/path_cam1", "/openvins/odom",
                              "/openvins/tracking_image", "/openvins/points"}) {
      output.write_serialized(topic, "std_msgs/msg/String", serialized, 500);
    }
    tf2_msgs::msg::TFMessage transform_message;
    rclcpp::SerializedMessage serialized_transform;
    rclcpp::Serialization<tf2_msgs::msg::TFMessage>().serialize_message(&transform_message, &serialized_transform);
    output.write_serialized("/tf", "tf2_msgs/msg/TFMessage", serialized_transform, 500);
    output.write_serialized("/tf_static", "tf2_msgs/msg/TFMessage", serialized_transform, 500);
    output.commit();
  }

  // Exercise the same MCAP storage plugin ros2 bag play uses, rather than
  // merely testing our own YAML encoder and decoder together.
  rosbag2_storage::StorageOptions storage;
  storage.uri = output_path_;
  storage.storage_id = "mcap";
  rosbag2_cpp::Reader reader;
  reader.open(storage);
  std::size_t checked = 0;
  for (const auto &topic : reader.get_all_topics_and_types()) {
    if (topic.name == "/source_a" || topic.name == "/source_b")
      continue;
    ++checked;
    ASSERT_EQ(topic.offered_qos_profiles.size(), 1u) << topic.name;
    const auto &qos = topic.offered_qos_profiles.front().get_rmw_qos_profile();
    EXPECT_EQ(qos.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE) << topic.name;
    EXPECT_EQ(qos.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST) << topic.name;
    if (topic.name == "/tf") {
      EXPECT_EQ(qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
      EXPECT_EQ(qos.depth, 100u);
    } else {
      EXPECT_EQ(qos.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) << topic.name;
      EXPECT_EQ(qos.depth, 1u) << topic.name;
    }
  }
  EXPECT_EQ(checked, 11u);
}

TEST_F(McapIoTest, RejectsInputWithoutMessageIndexes) {
  make_input(false);
  EXPECT_THROW((openvins_slam::McapInput(input_path_)), std::runtime_error);
}
} // namespace
