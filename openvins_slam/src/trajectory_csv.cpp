#include "openvins_slam/trajectory_csv.hpp"

#include <cerrno>
#include <filesystem>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <vector>

#include <fcntl.h>
#include <linux/fs.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace openvins_slam {
namespace {

namespace fs = std::filesystem;
constexpr int64_t kNanos = 1000000000LL;

fs::path resolved(const fs::path &path) {
  // weakly_canonical resolves symlinked parent directories even when the
  // destination does not exist yet.
  return fs::weakly_canonical(fs::absolute(path));
}

bool entry_exists(const fs::path &path) {
  std::error_code error;
  const auto status = fs::symlink_status(path, error);
  if (error && error != std::errc::no_such_file_or_directory)
    throw fs::filesystem_error("inspect CSV path", path, error);
  return !error && status.type() != fs::file_type::not_found;
}

} // namespace

TrajectoryCsv::TrajectoryCsv(const std::string &requested_path, const std::string &input_path,
                             const std::string &output_path) {
  fs::path path = requested_path.empty() ? fs::path(input_path).replace_extension(".csv") : fs::path(requested_path);
  if (path.empty() || path.filename().empty()) throw std::runtime_error("invalid trajectory CSV path");
  output_ = path.string();
  if (resolved(path) == resolved(input_path))
    throw std::runtime_error("trajectory CSV and input MCAP must be different files");
  if (!output_path.empty() && resolved(path) == resolved(output_path))
    throw std::runtime_error("trajectory CSV and output MCAP must be different files");
  if (entry_exists(path)) throw std::runtime_error("trajectory CSV path already exists: " + output_);
  const fs::path parent = path.has_parent_path() ? path.parent_path() : fs::path(".");
  if (!fs::is_directory(parent)) throw std::runtime_error("trajectory CSV parent directory does not exist: " + parent.string());

  std::string pattern = output_ + ".partial.XXXXXX";
  std::vector<char> buffer(pattern.begin(), pattern.end());
  buffer.push_back('\0');
  const int fd = ::mkstemp(buffer.data());
  if (fd < 0) throw std::system_error(errno, std::generic_category(), "create temporary CSV near " + output_);
  temporary_ = buffer.data();
  file_ = ::fdopen(fd, "w");
  if (!file_) {
    const int error = errno;
    ::close(fd);
    cleanup();
    throw std::system_error(error, std::generic_category(), "open temporary trajectory CSV");
  }
  try {
    write_text("frame_index,timestamp_ns,bag_timestamp_ns,relative_time_s,pose_valid,frame_id,child_frame_id,x,y,z,qx,qy,qz,qw\n");
  } catch (...) {
    cleanup();
    throw;
  }
}

TrajectoryCsv::~TrajectoryCsv() { cleanup(); }

void TrajectoryCsv::cleanup() noexcept {
  if (file_) {
    std::fclose(file_);
    file_ = nullptr;
  }
  if (!temporary_.empty()) {
    ::unlink(temporary_.c_str());
    temporary_.clear();
  }
}

void TrajectoryCsv::write_text(const std::string &text) {
  if (!file_ || prepared_) throw std::runtime_error("trajectory CSV is not open for writing");
  if (std::fwrite(text.data(), 1, text.size(), file_) != text.size()) {
    const int error = errno ? errno : EIO;
    throw std::system_error(error, std::generic_category(), "write trajectory CSV " + output_);
  }
}

void TrajectoryCsv::write(uint64_t frame_index, int64_t timestamp_ns, uint64_t bag_timestamp_ns,
                          int64_t first_image_ns, const geometry_msgs::msg::Pose &pose) {
  if (timestamp_ns < first_image_ns) throw std::runtime_error("trajectory image timestamp precedes first image");
  const int64_t relative_ns = timestamp_ns - first_image_ns;
  std::ostringstream row;
  row.imbue(std::locale::classic());
  row << frame_index << ',' << timestamp_ns << ',' << bag_timestamp_ns << ','
      << relative_ns / kNanos << '.' << std::setfill('0') << std::setw(9) << relative_ns % kNanos
      << std::setfill(' ') << ",1,odom,imu," << std::setprecision(17)
      << pose.position.x << ',' << pose.position.y << ',' << pose.position.z << ','
      << pose.orientation.x << ',' << pose.orientation.y << ',' << pose.orientation.z << ',' << pose.orientation.w << '\n';
  write_text(row.str());
}

void TrajectoryCsv::prepare() {
  if (!file_ || prepared_) throw std::runtime_error("trajectory CSV cannot be prepared twice");
  if (std::fflush(file_) != 0) throw std::system_error(errno, std::generic_category(), "flush trajectory CSV " + output_);
  if (::fsync(::fileno(file_)) != 0) throw std::system_error(errno, std::generic_category(), "sync trajectory CSV " + output_);
  auto *file = file_;
  file_ = nullptr;
  if (std::fclose(file) != 0) throw std::system_error(errno, std::generic_category(), "close trajectory CSV " + output_);
  prepared_ = true;
}

void TrajectoryCsv::commit() {
  if (!prepared_ || temporary_.empty()) throw std::runtime_error("trajectory CSV is not prepared for commit");
  if (::syscall(SYS_renameat2, AT_FDCWD, temporary_.c_str(), AT_FDCWD, output_.c_str(), RENAME_NOREPLACE) != 0)
    throw std::system_error(errno, std::generic_category(), "publish trajectory CSV " + output_);
  temporary_.clear();
}

} // namespace openvins_slam
