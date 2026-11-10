#include "openvins_slam/mcap_io.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include <mcap/writer.hpp>
#include <rclcpp/qos.hpp>
#include <rosbag2_cpp/message_definitions/local_message_definition_source.hpp>
#include <rosbag2_storage/qos.hpp>

namespace openvins_slam {
namespace {

void require_ok(const mcap::Status &status, const std::string &operation) {
  if (!status.ok())
    throw std::runtime_error(operation + ": " + status.message);
}

// McapWriter::close() returns void, so write and fsync errors must propagate
// through the IWritable itself. The descriptor stays open until commit.
class FdWritable final : public mcap::IWritable {
public:
  ~FdWritable() override {
    if (fd_ >= 0)
      ::close(fd_);
  }
  void reset(int fd) { fd_ = fd; }
  uint64_t size() const override { return size_; }
  void end() override { sync(); }
  void sync() {
    if (fd_ < 0)
      throw std::runtime_error("MCAP temporary file is not open");
    while (::fsync(fd_) != 0) {
      if (errno != EINTR)
        throw std::system_error(errno, std::generic_category(), "fsync MCAP");
    }
  }
  void close_checked() {
    if (fd_ < 0)
      return;
    int fd = fd_;
    fd_ = -1;
    if (::close(fd) != 0)
      throw std::system_error(errno, std::generic_category(), "close MCAP");
  }

protected:
  void handleWrite(const std::byte *data, uint64_t size) override {
    while (size != 0) {
      const size_t chunk = static_cast<size_t>(
          std::min<uint64_t>(size, static_cast<uint64_t>(std::numeric_limits<ssize_t>::max())));
      const ssize_t written = ::write(fd_, data, chunk);
      if (written < 0) {
        if (errno == EINTR)
          continue;
        throw std::system_error(errno, std::generic_category(), "write MCAP");
      }
      if (written == 0)
        throw std::runtime_error("write MCAP returned zero bytes");
      data += written;
      size -= static_cast<uint64_t>(written);
      size_ += static_cast<uint64_t>(written);
    }
  }

private:
  int fd_ = -1;
  uint64_t size_ = 0;
};

} // namespace

McapInput::McapInput(const std::string &path) {
  require_ok(reader_.open(path), "open input MCAP " + path);
  const auto on_problem = [](const mcap::Status &status) {
    throw std::runtime_error("read MCAP summary: " + status.message);
  };
  require_ok(reader_.readSummary(mcap::ReadSummaryMethod::NoFallbackScan, on_problem), "read MCAP summary");
  if (!reader_.header() || reader_.header()->profile != "ros2")
    throw std::runtime_error("input MCAP must have the ros2 profile");
  if (reader_.chunkIndexes().empty())
    throw std::runtime_error("input MCAP has no chunk indexes; sorted reading is required");
  for (const auto &chunk : reader_.chunkIndexes()) {
    if (chunk.messageIndexOffsets.empty() || chunk.messageIndexLength == 0)
      throw std::runtime_error("input MCAP has a chunk without message indexes; sorted reading is required");
  }
}

McapInput::~McapInput() = default;

void McapInput::for_each(const std::function<bool(const mcap::MessageView &)> &callback) {
  mcap::ReadMessageOptions options;
  options.readOrder = mcap::ReadMessageOptions::ReadOrder::LogTimeOrder;
  const auto on_problem = [](const mcap::Status &status) {
    throw std::runtime_error("read indexed MCAP messages: " + status.message);
  };
  uint64_t count = 0;
  bool completed = true;
  mcap::Timestamp last = 0;
  for (const auto &view : reader_.readMessages(on_problem, options)) {
    if (count != 0 && view.message.logTime < last)
      throw std::runtime_error("MCAP indexed reader returned messages out of log-time order");
    last = view.message.logTime;
    ++count;
    if (!callback(view)) {
      completed = false;
      break;
    }
  }
  if (completed && reader_.statistics() && count != reader_.statistics()->messageCount)
    throw std::runtime_error("MCAP indexed message count differs from summary statistics");
}

mcap::McapReader &McapInput::reader() { return reader_; }

class McapOutput::Impl {
public:
  ~Impl() noexcept {
    if (writer_open_) {
      try {
        writer_.terminate();
      } catch (...) {
      }
    }
    if (!temporary_.empty())
      ::unlink(temporary_.c_str());
  }

  void open(const std::string &output_path, const std::string &input_path, McapInput &input) {
    if (output_path.empty())
      throw std::invalid_argument("output MCAP path is empty");
    const std::filesystem::path input_file = std::filesystem::weakly_canonical(std::filesystem::absolute(input_path));
    const std::filesystem::path output_file =
        std::filesystem::weakly_canonical(std::filesystem::absolute(output_path));
    if (input_file == output_file)
      throw std::runtime_error("input and output MCAP must be different files");
    const std::filesystem::path parent = output_file.parent_path();
    std::error_code error;
    std::filesystem::create_directories(parent, error);
    if (error)
      throw std::filesystem::filesystem_error("create output MCAP parent directory", parent, error);
    output_ = output_path;
    std::string pattern = output_path + ".partial.XXXXXX";
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    const int fd = ::mkstemp(buffer.data());
    if (fd < 0)
      throw std::system_error(errno, std::generic_category(), "create temporary MCAP near " + output_path);
    temporary_ = buffer.data();
    file_.reset(fd);
    mcap::McapWriterOptions options("ros2");
    // JPEG input is already compressed; avoiding recompression also reduces
    // CPU time during offline VIO and preserves indexed random access.
    options.compression = mcap::Compression::None;
    writer_open_ = true;
    writer_.open(file_, options);

    const auto schemas = input.reader().schemas();
    std::vector<mcap::SchemaId> schema_ids;
    schema_ids.reserve(schemas.size());
    for (const auto &entry : schemas)
      schema_ids.push_back(entry.first);
    std::sort(schema_ids.begin(), schema_ids.end());
    for (auto id : schema_ids) {
      auto schema = *schemas.at(id);
      writer_.addSchema(schema);
      schema_map_.emplace(id, schema.id);
    }
    const auto channels = input.reader().channels();
    std::vector<mcap::ChannelId> channel_ids;
    channel_ids.reserve(channels.size());
    for (const auto &entry : channels)
      channel_ids.push_back(entry.first);
    std::sort(channel_ids.begin(), channel_ids.end());
    for (auto id : channel_ids) {
      auto channel = *channels.at(id);
      if (channel.schemaId != 0) {
        const auto found = schema_map_.find(channel.schemaId);
        if (found == schema_map_.end())
          throw std::runtime_error("input channel references a missing schema");
        channel.schemaId = found->second;
      }
      original_topics_.insert(channel.topic);
      writer_.addChannel(channel);
      channel_map_.emplace(id, channel.id);
    }
  }

  void write_original(const mcap::MessageView &view) {
    ensure_open();
    const auto found = channel_map_.find(view.message.channelId);
    if (found == channel_map_.end())
      throw std::runtime_error("input message references an unregistered channel");
    auto copied = view.message;
    copied.channelId = found->second;
    require_ok(writer_.write(copied), "write original MCAP message");
    ++message_count_;
  }

  void write_serialized(const std::string &topic, const std::string &ros_type,
                        const rclcpp::SerializedMessage &data, uint64_t stamp_ns) {
    ensure_open();
    if (original_topics_.count(topic) != 0)
      throw std::runtime_error("output topic conflicts with an input topic: " + topic);
    auto channel_it = output_channels_.find(topic);
    if (channel_it == output_channels_.end()) {
      auto schema_it = output_schemas_.find(ros_type);
      if (schema_it == output_schemas_.end()) {
        const auto definition = definitions_.get_full_text(ros_type);
        if (definition.encoded_message_definition.empty() || definition.encoding.empty())
          throw std::runtime_error("ROS 2 message definition is empty: " + ros_type);
        mcap::Schema schema(ros_type, definition.encoding, definition.encoded_message_definition);
        writer_.addSchema(schema);
        schema_it = output_schemas_.emplace(ros_type, schema.id).first;
      }
      mcap::Channel channel(topic, "cdr", schema_it->second);
      // rosbag2's MCAP plugin reads offered_qos_profiles from channel
      // metadata. Without it playback falls back to volatile durability,
      // preventing TF listeners from receiving the latched /tf_static.
      // Match the live publishers in main.cpp: /tf uses a volatile depth-100
      // queue; all other result and debug topics use transient-local depth 1.
      const auto qos = topic == "/tf" ? rclcpp::QoS(100).reliable().durability_volatile()
                                       : rclcpp::QoS(1).reliable().transient_local();
      channel.metadata["offered_qos_profiles"] = rosbag2_storage::serialize_rclcpp_qos_vector({qos});
      writer_.addChannel(channel);
      channel_it = output_channels_.emplace(topic, OutputChannel{channel.id, ros_type, 0}).first;
    } else if (channel_it->second.type != ros_type) {
      throw std::runtime_error("output topic used with multiple ROS types: " + topic);
    }
    auto &output_channel = channel_it->second;
    const auto &serialized = data.get_rcl_serialized_message();
    mcap::Message message{};
    message.channelId = output_channel.id;
    message.sequence = output_channel.sequence++;
    message.logTime = stamp_ns;
    message.publishTime = stamp_ns;
    message.dataSize = serialized.buffer_length;
    message.data = reinterpret_cast<const std::byte *>(serialized.buffer);
    require_ok(writer_.write(message), "write ROS 2 MCAP message " + topic);
    ++message_count_;
  }

  void commit() {
    ensure_open();
    writer_.close();
    writer_open_ = false;
    file_.sync();
    file_.close_checked();
    // Validate the footer and complete summary before the final name appears.
    mcap::McapReader check;
    require_ok(check.open(temporary_), "reopen temporary MCAP for validation");
    const auto on_problem = [](const mcap::Status &status) {
      throw std::runtime_error("validate output MCAP: " + status.message);
    };
    require_ok(check.readSummary(mcap::ReadSummaryMethod::NoFallbackScan, on_problem), "validate output MCAP summary");
    if (!check.header() || check.header()->profile != "ros2" || !check.statistics() ||
        check.statistics()->messageCount != message_count_)
      throw std::runtime_error("output MCAP summary message count or profile is invalid");
    check.close();
    // Rename within the same directory atomically replaces any existing file,
    // so reruns overwrite the previous output while readers never observe a
    // partially written MCAP.
    if (::rename(temporary_.c_str(), output_.c_str()) != 0)
      throw std::system_error(errno, std::generic_category(), "publish output MCAP " + output_);
    temporary_.clear();
  }

private:
  struct OutputChannel {
    mcap::ChannelId id;
    std::string type;
    uint32_t sequence;
  };
  void ensure_open() const {
    if (!writer_open_)
      throw std::runtime_error("MCAP output is already closed");
  }

  std::string output_;
  std::string temporary_;
  // Destruction is in reverse order: writer must never outlive its IWritable.
  FdWritable file_;
  mcap::McapWriter writer_;
  bool writer_open_ = false;
  uint64_t message_count_ = 0;
  std::unordered_map<mcap::SchemaId, mcap::SchemaId> schema_map_;
  std::unordered_map<mcap::ChannelId, mcap::ChannelId> channel_map_;
  std::unordered_set<std::string> original_topics_;
  std::unordered_map<std::string, mcap::SchemaId> output_schemas_;
  std::unordered_map<std::string, OutputChannel> output_channels_;
  rosbag2_cpp::LocalMessageDefinitionSource definitions_;
};

McapOutput::McapOutput(const std::string &output_path, const std::string &input_path, McapInput &input)
    : impl_(std::make_unique<Impl>()) {
  impl_->open(output_path, input_path, input);
}
McapOutput::~McapOutput() = default;
void McapOutput::write_original(const mcap::MessageView &view) { impl_->write_original(view); }
void McapOutput::write_serialized(const std::string &topic, const std::string &ros_type,
                                  const rclcpp::SerializedMessage &data, uint64_t stamp_ns) {
  impl_->write_serialized(topic, ros_type, data, stamp_ns);
}
void McapOutput::commit() { impl_->commit(); }

} // namespace openvins_slam
