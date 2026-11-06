#include "openvins_slam/trajectory_csv.hpp"

#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <unistd.h>

namespace {
namespace fs = std::filesystem;

class TrajectoryCsvTest : public ::testing::Test {
protected:
  void SetUp() override {
    char path[] = "/tmp/openvins-csv-test-XXXXXX";
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

TEST_F(TrajectoryCsvTest, RecordsExactImageAndBagTimesAndPublishesOnCommit) {
  const auto csv_path = directory_ + "/input.csv";
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.1;
  pose.position.y = -2.0;
  pose.position.z = 3.5;
  pose.orientation.w = 1.0;
  {
    openvins_slam::TrajectoryCsv csv("", input_, output_);
    EXPECT_EQ(csv.path(), csv_path);
    csv.write(12, 1793850304571406667LL, 1793850304694863456ULL, 1793850302571406544LL, pose);
    EXPECT_FALSE(fs::exists(csv_path));
    csv.prepare();
    EXPECT_FALSE(fs::exists(csv_path));
    csv.commit();
  }
  std::ifstream file(csv_path);
  std::string header, row, extra;
  std::getline(file, header);
  std::getline(file, row);
  std::getline(file, extra);
  EXPECT_EQ(header, "frame_index,timestamp_ns,bag_timestamp_ns,relative_time_s,pose_valid,frame_id,child_frame_id,x,y,z,qx,qy,qz,qw");
  EXPECT_EQ(row, "12,1793850304571406667,1793850304694863456,2.000000123,1,odom,imu,0.10000000000000001,-2,3.5,0,0,0,1");
  EXPECT_TRUE(extra.empty());
}

TEST_F(TrajectoryCsvTest, RejectsConflictsMissingParentsAndSymlinkAliases) {
  EXPECT_THROW((openvins_slam::TrajectoryCsv(input_, input_, output_)), std::runtime_error);
  EXPECT_THROW((openvins_slam::TrajectoryCsv(output_, input_, output_)), std::runtime_error);
  EXPECT_THROW((openvins_slam::TrajectoryCsv(directory_ + "/absent/out.csv", input_, output_)), std::runtime_error);
  fs::create_directory_symlink(directory_, directory_ + "/alias");
  EXPECT_THROW((openvins_slam::TrajectoryCsv(directory_ + "/alias/output.mcap", input_, output_)), std::runtime_error);
  const auto csv_path = directory_ + "/existing.csv";
  std::ofstream(csv_path) << "original";
  EXPECT_THROW((openvins_slam::TrajectoryCsv(csv_path, input_, output_)), std::runtime_error);
  const auto dangling = directory_ + "/dangling.csv";
  fs::create_symlink(directory_ + "/missing", dangling);
  EXPECT_THROW((openvins_slam::TrajectoryCsv(dangling, input_, output_)), std::runtime_error);
}

TEST_F(TrajectoryCsvTest, CleansTemporaryAndNeverOverwritesConcurrentFile) {
  const auto csv_path = directory_ + "/result.csv";
  {
    openvins_slam::TrajectoryCsv csv(csv_path, input_, output_);
    csv.prepare();
    std::ofstream(csv_path) << "concurrent creator";
    EXPECT_THROW(csv.commit(), std::system_error);
  }
  std::ifstream file(csv_path);
  std::string content;
  std::getline(file, content);
  EXPECT_EQ(content, "concurrent creator");
  for (const auto &entry : fs::directory_iterator(directory_))
    EXPECT_EQ(entry.path().filename().string().find(".partial."), std::string::npos);
  fs::remove(csv_path);
  {
    openvins_slam::TrajectoryCsv csv(csv_path, input_, output_);
  }
  EXPECT_FALSE(fs::exists(csv_path));
  for (const auto &entry : fs::directory_iterator(directory_))
    EXPECT_EQ(entry.path().filename().string().find(".partial."), std::string::npos);
}

} // namespace
