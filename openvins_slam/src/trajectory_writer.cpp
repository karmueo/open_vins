#include "openvins_slam/trajectory_writer.hpp"

#include <cerrno>
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <vector>

#include <fcntl.h>
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

fs::path default_trajectory_path(const std::string &input_path, const std::string &output_path) {
  const fs::path input(input_path);
  const fs::path directory = output_path.empty() ? input.parent_path() : fs::path(output_path).parent_path();
  return directory / (input.stem().string() + "_openvins_trajectory.txt");
}

} // namespace

TrajectoryWriter::TrajectoryWriter(const std::string &requested_path, const std::string &input_path,
                                   const std::string &output_path) {
  fs::path path = requested_path.empty() ? default_trajectory_path(input_path, output_path) : fs::path(requested_path);
  if (path.empty() || path.filename().empty()) throw std::runtime_error("invalid trajectory file path");
  output_ = path.string();
  if (resolved(path) == resolved(input_path))
    throw std::runtime_error("trajectory file and input MCAP must be different files");
  if (!output_path.empty() && resolved(path) == resolved(output_path))
    throw std::runtime_error("trajectory file and output MCAP must be different files");
  const fs::path parent = path.has_parent_path() ? path.parent_path() : fs::path(".");
  std::error_code error;
  fs::create_directories(parent, error);
  if (error) throw fs::filesystem_error("create trajectory file parent directory", parent, error);
  if (!fs::is_directory(parent))
    throw std::runtime_error("trajectory file parent path is not a directory: " + parent.string());

  std::string pattern = output_ + ".partial.XXXXXX";
  std::vector<char> buffer(pattern.begin(), pattern.end());
  buffer.push_back('\0');
  const int fd = ::mkstemp(buffer.data());
  if (fd < 0) throw std::system_error(errno, std::generic_category(), "create temporary trajectory file near " + output_);
  temporary_ = buffer.data();
  file_ = ::fdopen(fd, "w");
  if (!file_) {
    const int error = errno;
    ::close(fd);
    cleanup();
    throw std::system_error(error, std::generic_category(), "open temporary trajectory file");
  }
}

TrajectoryWriter::~TrajectoryWriter() { cleanup(); }

void TrajectoryWriter::cleanup() noexcept {
  if (file_) {
    std::fclose(file_);
    file_ = nullptr;
  }
  if (!temporary_.empty()) {
    ::unlink(temporary_.c_str());
    temporary_.clear();
  }
}

void TrajectoryWriter::write_text(const std::string &text) {
  if (!file_ || prepared_) throw std::runtime_error("trajectory file is not open for writing");
  if (std::fwrite(text.data(), 1, text.size(), file_) != text.size()) {
    const int error = errno ? errno : EIO;
    throw std::system_error(error, std::generic_category(), "write trajectory file " + output_);
  }
}

void TrajectoryWriter::write(int64_t timestamp_ns, const geometry_msgs::msg::Pose &pose) {
  if (timestamp_ns < 0) throw std::runtime_error("invalid trajectory image timestamp");
  std::ostringstream row;
  row.imbue(std::locale::classic());
  row << timestamp_ns / kNanos << '.' << std::setfill('0') << std::setw(9) << timestamp_ns % kNanos
      << std::setfill(' ') << ' ' << std::setprecision(17)
      << pose.position.x << ' ' << pose.position.y << ' ' << pose.position.z << ' '
      << pose.orientation.x << ' ' << pose.orientation.y << ' ' << pose.orientation.z << ' ' << pose.orientation.w << '\n';
  write_text(row.str());
}

void TrajectoryWriter::prepare() {
  if (!file_ || prepared_) throw std::runtime_error("trajectory file cannot be prepared twice");
  if (std::fflush(file_) != 0) throw std::system_error(errno, std::generic_category(), "flush trajectory file " + output_);
  if (::fsync(::fileno(file_)) != 0) throw std::system_error(errno, std::generic_category(), "sync trajectory file " + output_);
  auto *file = file_;
  file_ = nullptr;
  if (std::fclose(file) != 0) throw std::system_error(errno, std::generic_category(), "close trajectory file " + output_);
  prepared_ = true;
}

void TrajectoryWriter::commit() {
  if (!prepared_ || temporary_.empty()) throw std::runtime_error("trajectory file is not prepared for commit");
  // Rename within the same directory atomically replaces any existing file, so
  // reruns overwrite the previous trajectory while readers never see a partial
  // one.
  if (::rename(temporary_.c_str(), output_.c_str()) != 0)
    throw std::system_error(errno, std::generic_category(), "publish trajectory file " + output_);
  temporary_.clear();
}

} // namespace openvins_slam
