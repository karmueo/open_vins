/**
 * @file raw_io.cpp
 * @brief 读取 RAW 图像、时间戳索引和 IMU 样本，并按时间戳归并。
 */
#include "openvins_slam/raw_io.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <unordered_map>

namespace openvins_slam {
namespace {

namespace fs = std::filesystem;

std::string trim(std::string_view view) {
  while (!view.empty() && std::isspace(static_cast<unsigned char>(view.front()))) view.remove_prefix(1);
  while (!view.empty() && std::isspace(static_cast<unsigned char>(view.back()))) view.remove_suffix(1);
  return std::string(view);
}

std::vector<std::string> split_csv(const std::string &line) {
  std::vector<std::string> fields;
  std::size_t start = 0;
  while (true) {
    const auto comma = line.find(',', start);
    if (comma == std::string::npos) {
      fields.push_back(trim(std::string_view(line).substr(start)));
      break;
    }
    fields.push_back(trim(std::string_view(line).substr(start, comma - start)));
    start = comma + 1;
  }
  return fields;
}

int64_t parse_non_negative_ns(const std::string &token, const std::string &context) {
  if (token.empty()) throw std::runtime_error("empty timestamp in " + context);
  int64_t value = -1;
  const char *begin = token.data();
  const char *end = begin + token.size();
  const auto result = std::from_chars(begin, end, value, 10);
  if (result.ec != std::errc{} || result.ptr != end || value < 0)
    throw std::runtime_error("invalid nanosecond timestamp '" + token + "' in " + context);
  return value;
}

double parse_finite_double(const std::string &token, const std::string &context) {
  if (token.empty()) throw std::runtime_error("empty numeric field in " + context);
  char *end = nullptr;
  const double value = std::strtod(token.c_str(), &end);
  if (end != token.c_str() + token.size() || !std::isfinite(value))
    throw std::runtime_error("invalid floating-point value '" + token + "' in " + context);
  return value;
}

std::unordered_map<std::string, std::size_t> index_header(const std::vector<std::string> &columns,
                                                          const std::string &context) {
  std::unordered_map<std::string, std::size_t> indices;
  for (std::size_t i = 0; i < columns.size(); ++i) {
    if (!indices.emplace(columns[i], i).second)
      throw std::runtime_error("duplicate column '" + columns[i] + "' in " + context);
  }
  return indices;
}

std::size_t require_column(const std::unordered_map<std::string, std::size_t> &indices,
                           const std::string &name, const std::string &context) {
  const auto found = indices.find(name);
  if (found == indices.end())
    throw std::runtime_error("missing required column '" + name + "' in " + context);
  return found->second;
}

std::vector<uint8_t> read_binary_file(const std::string &path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) throw std::runtime_error("cannot open image file: " + path);
  const auto end_pos = file.tellg();
  if (end_pos <= 0) throw std::runtime_error("empty or unreadable image file: " + path);
  std::vector<uint8_t> bytes(static_cast<std::size_t>(end_pos));
  file.seekg(0, std::ios::beg);
  if (!file.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(bytes.size())))
    throw std::runtime_error("failed reading image file: " + path);
  return bytes;
}

std::string image_format_for_path(const std::string &path) {
  std::string ext = fs::path(path).extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (ext == ".png") return "png";
  return "jpeg";
}

bool next_data_line(std::ifstream &stream, std::string &line, std::size_t &line_no) {
  while (std::getline(stream, line)) {
    ++line_no;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    const auto trimmed = trim(line);
    if (trimmed.empty() || trimmed.front() == '#') continue;
    line = trimmed;
    return true;
  }
  return false;
}

} // namespace

/**
 * @brief 校验并加载 RAW 输入，按配置预加载压缩图片。
 * @param[in] directory RAW 数据目录。
 * @param[in] cache 是否预加载全部压缩图像。
 * @param[in] should_cancel 可选的取消回调，每次预加载读图前调用。
 * @throws std::runtime_error 预加载期间取消，或 RAW 输入读取失败。
 */
RawDataset::RawDataset(const std::string &directory, bool cache, const std::function<bool()> &should_cancel) {
  if (directory.empty()) throw std::runtime_error("parameter in_raw is required");
  std::error_code ec;
  const fs::path root(directory);
  if (!fs::is_directory(root, ec))
    throw std::runtime_error("in_raw is not a directory: " + directory);
  directory_ = root.string();

  const fs::path images_dir = root / "images";
  if (!fs::is_directory(images_dir, ec))
    throw std::runtime_error("missing images/ directory under in_raw: " + images_dir.string());
  images_directory_ = images_dir.string();

  const fs::path aligned_dir = root / "aligned";
  const fs::path aligned_ts = aligned_dir / "image_timestamps.txt";
  const fs::path aligned_imu = aligned_dir / "imu.csv";
  const fs::path root_ts = root / "image_timestamps.txt";
  const fs::path root_imu = root / "imu.csv";

  if (fs::is_regular_file(aligned_ts, ec) && fs::is_regular_file(aligned_imu, ec)) {
    image_timestamps_path_ = aligned_ts.string();
    imu_csv_path_ = aligned_imu.string();
  } else if (fs::is_regular_file(root_ts, ec) && fs::is_regular_file(root_imu, ec)) {
    image_timestamps_path_ = root_ts.string();
    imu_csv_path_ = root_imu.string();
  } else {
    throw std::runtime_error("could not find image_timestamps.txt and imu.csv under " +
                             aligned_dir.string() + " or " + root.string());
  }

  // Parse image_timestamps.txt using image_file and cam_time_ns columns only.
  {
    std::ifstream stream(image_timestamps_path_);
    if (!stream.is_open())
      throw std::runtime_error("cannot open " + image_timestamps_path_);
    std::string line;
    std::size_t line_no = 0;
    if (!next_data_line(stream, line, line_no))
      throw std::runtime_error("empty image_timestamps.txt: " + image_timestamps_path_);
    const auto header = split_csv(line);
    const auto col_map = index_header(header, image_timestamps_path_);
    const auto file_col = require_column(col_map, "image_file", image_timestamps_path_);
    const auto time_col = require_column(col_map, "cam_time_ns", image_timestamps_path_);

    int64_t prev_ns = -1;
    while (next_data_line(stream, line, line_no)) {
      const auto row = split_csv(line);
      const std::string context = image_timestamps_path_ + ":" + std::to_string(line_no);
      if (row.size() != header.size())
        throw std::runtime_error("unexpected column count in " + context);
      const std::string &image_file = row[file_col];
      if (image_file.empty() || image_file == "NA" || image_file == "-1")
        throw std::runtime_error("invalid image_file in " + context);
      const fs::path rel(image_file);
      if (rel.is_absolute())
        throw std::runtime_error("image_file must be relative to images/: " + image_file);
      for (const auto &part : rel) {
        if (part == "..")
          throw std::runtime_error("image_file must not traverse outside images/: " + image_file);
      }
      const fs::path resolved_image = images_dir / rel;
      if (!fs::is_regular_file(resolved_image, ec))
        throw std::runtime_error("image listed in " + context + " does not exist: " + resolved_image.string());
      const int64_t cam_ns = parse_non_negative_ns(row[time_col], context);
      if (!images_.empty() && cam_ns <= prev_ns)
        throw std::runtime_error("camera timestamps in image_timestamps.txt must increase strictly at " + context);
      prev_ns = cam_ns;
      images_.push_back({cam_ns, image_file, resolved_image.string()});
    }
    if (images_.empty())
      throw std::runtime_error("image_timestamps.txt contains no image rows: " + image_timestamps_path_);
  }

  // Parse imu.csv using time_ns, gyro_*_rad_s, and acc_*_m_s2 columns.
  {
    std::ifstream stream(imu_csv_path_);
    if (!stream.is_open())
      throw std::runtime_error("cannot open " + imu_csv_path_);
    std::string line;
    std::size_t line_no = 0;
    if (!next_data_line(stream, line, line_no))
      throw std::runtime_error("empty imu.csv: " + imu_csv_path_);
    const auto header = split_csv(line);
    const auto col_map = index_header(header, imu_csv_path_);
    const auto time_col = require_column(col_map, "time_ns", imu_csv_path_);
    const auto gx_col = require_column(col_map, "gyro_x_rad_s", imu_csv_path_);
    const auto gy_col = require_column(col_map, "gyro_y_rad_s", imu_csv_path_);
    const auto gz_col = require_column(col_map, "gyro_z_rad_s", imu_csv_path_);
    const auto ax_col = require_column(col_map, "acc_x_m_s2", imu_csv_path_);
    const auto ay_col = require_column(col_map, "acc_y_m_s2", imu_csv_path_);
    const auto az_col = require_column(col_map, "acc_z_m_s2", imu_csv_path_);

    int64_t prev_ns = -1;
    while (next_data_line(stream, line, line_no)) {
      const auto row = split_csv(line);
      const std::string context = imu_csv_path_ + ":" + std::to_string(line_no);
      if (row.size() != header.size())
        throw std::runtime_error("unexpected column count in " + context);
      const int64_t time_ns = parse_non_negative_ns(row[time_col], context);
      if (!imu_.empty() && time_ns <= prev_ns)
        throw std::runtime_error("IMU timestamps in imu.csv must increase strictly at " + context);
      prev_ns = time_ns;
      RawImuSample sample;
      sample.time_ns = time_ns;
      sample.gyro = Eigen::Vector3d(parse_finite_double(row[gx_col], context),
                                    parse_finite_double(row[gy_col], context),
                                    parse_finite_double(row[gz_col], context));
      sample.accel = Eigen::Vector3d(parse_finite_double(row[ax_col], context),
                                     parse_finite_double(row[ay_col], context),
                                     parse_finite_double(row[az_col], context));
      imu_.push_back(sample);
    }
    if (imu_.empty())
      throw std::runtime_error("imu.csv contains no IMU samples: " + imu_csv_path_);
  }

  if (cache) {
    cached_images_.reserve(images_.size());
    for (const auto &entry : images_) {
      if (should_cancel && should_cancel()) throw std::runtime_error("RAW image preloading interrupted");
      cached_images_.push_back(read_binary_file(entry.image_path));
    }
  }
}

/**
 * @brief 从缓存或磁盘获取图像，并与 IMU 按时间戳归并。
 * @param[in] on_imu IMU 样本回调，返回 false 时提前停止。
 * @param[in] on_image 图片回调，返回 false 时提前停止。
 */
void RawDataset::for_each(
    const std::function<bool(const RawImuSample &)> &on_imu,
    const std::function<bool(const RawImageEntry &, std::vector<uint8_t> &&, const std::string &)> &on_image) const {
  std::size_t imu_idx = 0;
  std::size_t img_idx = 0;
  while (imu_idx < imu_.size() || img_idx < images_.size()) {
    const bool emit_imu =
        img_idx == images_.size() || (imu_idx < imu_.size() && imu_[imu_idx].time_ns <= images_[img_idx].cam_time_ns);
    if (emit_imu) {
      if (!on_imu(imu_[imu_idx])) return;
      ++imu_idx;
    } else {
      const auto &entry = images_[img_idx];
      // 为回调提供独立的字节缓冲区，使同一数据集可以重复遍历。
      auto bytes = cached_images_.empty() ? read_binary_file(entry.image_path) : cached_images_[img_idx];
      const auto format = image_format_for_path(entry.image_path);
      if (!on_image(entry, std::move(bytes), format)) return;
      ++img_idx;
    }
  }
}

} // namespace openvins_slam
