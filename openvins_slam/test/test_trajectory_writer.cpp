#include "openvins_slam/trajectory_writer.hpp"

#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <unistd.h>

namespace {
namespace fs = std::filesystem;

class TrajectoryWriterTest : public ::testing::Test {
protected:
  void SetUp() override {
    char path[] = "/tmp/openvins-trajectory-test-XXXXXX";
    auto *created = ::mkdtemp(path);
    ASSERT_NE(created, nullptr);
    directory_ = created;
    input_ = directory_ + "/input.mcap";
    output_ = directory_ + "/output.mcap";
    std::ofstream(input_) << "input";
  }
  void TearDown() override { fs::remove_all(directory_); }
  std::string directory_, input_, output_;
};

TEST_F(TrajectoryWriterTest, RecordsExactImageTimesAndTumPosesWithoutHeader) {
  const auto trajectory_path = directory_ + "/input.txt";
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.1;
  pose.position.y = -2.0;
  pose.position.z = 3.5;
  pose.orientation.x = 0.125;
  pose.orientation.y = -0.25;
  pose.orientation.z = 0.5;
  pose.orientation.w = 1.0;
  {
    openvins_slam::TrajectoryWriter trajectory("", input_, output_);
    EXPECT_EQ(trajectory.path(), trajectory_path);
    trajectory.write(1793850304571406667LL, pose);
    trajectory.write(1793850305000000001LL, pose);
    trajectory.write(1793850306000000000LL, pose);
    EXPECT_THROW(trajectory.write(-1, pose), std::runtime_error);
    EXPECT_FALSE(fs::exists(trajectory_path));
    trajectory.prepare();
    EXPECT_FALSE(fs::exists(trajectory_path));
    trajectory.commit();
  }
  std::ifstream file(trajectory_path);
  std::string row, extra;
  ASSERT_TRUE(std::getline(file, row));
  EXPECT_EQ(row, "1793850304.571406667 0.10000000000000001 -2 3.5 0.125 -0.25 0.5 1");
  ASSERT_TRUE(std::getline(file, row));
  EXPECT_EQ(row, "1793850305.000000001 0.10000000000000001 -2 3.5 0.125 -0.25 0.5 1");
  ASSERT_TRUE(std::getline(file, row));
  EXPECT_EQ(row, "1793850306.000000000 0.10000000000000001 -2 3.5 0.125 -0.25 0.5 1");
  std::getline(file, extra);
  EXPECT_TRUE(extra.empty());
}

TEST_F(TrajectoryWriterTest, RejectsSelfConflictsAndSymlinkAliases) {
  EXPECT_THROW((openvins_slam::TrajectoryWriter(input_, input_, output_)), std::runtime_error);
  EXPECT_THROW((openvins_slam::TrajectoryWriter(output_, input_, output_)), std::runtime_error);
  fs::create_directory_symlink(directory_, directory_ + "/alias");
  EXPECT_THROW((openvins_slam::TrajectoryWriter(directory_ + "/alias/output.mcap", input_, output_)), std::runtime_error);
}

TEST_F(TrajectoryWriterTest, CreatesMissingParentDirectoriesAndOverwritesExistingFile) {
  const auto trajectory_path = directory_ + "/nested/deeper/result.txt";
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  {
    openvins_slam::TrajectoryWriter trajectory(trajectory_path, input_, output_);
    trajectory.write(1793850304571406667LL, pose);
    trajectory.prepare();
    trajectory.commit();
  }
  ASSERT_TRUE(fs::is_regular_file(trajectory_path));

  std::ofstream(trajectory_path) << "old content\n";
  {
    openvins_slam::TrajectoryWriter trajectory(trajectory_path, input_, output_);
    trajectory.write(1793850305000000000LL, pose);
    std::ifstream before(trajectory_path);
    std::string content;
    std::getline(before, content);
    EXPECT_EQ(content, "old content");
    trajectory.prepare();
    std::ifstream prepared(trajectory_path);
    std::getline(prepared, content);
    EXPECT_EQ(content, "old content");
    trajectory.commit();
  }
  std::ifstream file(trajectory_path);
  std::string row, extra;
  ASSERT_TRUE(std::getline(file, row));
  EXPECT_EQ(row, "1793850305.000000000 0 0 0 0 0 0 1");
  std::getline(file, extra);
  EXPECT_TRUE(extra.empty());
}

TEST_F(TrajectoryWriterTest, CleansTemporaryFilesAndOverwritesConcurrentFile) {
  const auto trajectory_path = directory_ + "/result.txt";
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  {
    openvins_slam::TrajectoryWriter trajectory(trajectory_path, input_, output_);
    trajectory.write(1793850304571406667LL, pose);
    trajectory.prepare();
    std::ofstream(trajectory_path) << "concurrent creator";
    EXPECT_NO_THROW(trajectory.commit());
  }
  {
    std::ifstream file(trajectory_path);
    std::string content;
    std::getline(file, content);
    EXPECT_EQ(content, "1793850304.571406667 0 0 0 0 0 0 1");
  }
  for (const auto &entry : fs::directory_iterator(directory_))
    EXPECT_EQ(entry.path().filename().string().find(".partial."), std::string::npos);
  fs::remove(trajectory_path);
  {
    openvins_slam::TrajectoryWriter trajectory(trajectory_path, input_, output_);
  }
  EXPECT_FALSE(fs::exists(trajectory_path));
  for (const auto &entry : fs::directory_iterator(directory_))
    EXPECT_EQ(entry.path().filename().string().find(".partial."), std::string::npos);
}

} // namespace
