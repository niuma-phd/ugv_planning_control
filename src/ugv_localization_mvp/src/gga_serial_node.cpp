#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "ugv_localization_mvp/gga_parser.hpp"

namespace ugv_localization_mvp
{
namespace
{

std::optional<speed_t> baudConstant(int baud_rate)
{
  switch (baud_rate) {
    case 1200: return B1200;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return std::nullopt;
  }
}

bool sentenceIdValid(const std::string & sentence_id)
{
  return sentence_id.size() == 5U && sentence_id.substr(2U) == "GGA" &&
         std::all_of(sentence_id.begin(), sentence_id.begin() + 2, [](char character) {
           return character >= 'A' && character <= 'Z';
         });
}

bool hasGgaSentenceType(std::string_view sentence)
{
  return sentence.size() >= 6U && sentence.front() == '$' &&
         sentence.substr(3U, 3U) == "GGA";
}

}  // namespace

class GgaSerialNode : public rclcpp::Node
{
public:
  GgaSerialNode()
  : Node("gga_serial"), line_buffer_(loadMaximumSentenceLength())
  {
    device_ = declare_parameter<std::string>("device", "");
    baud_rate_ = declare_parameter<int>("baud_rate", 0);
    data_bits_ = declare_parameter<int>("data_bits", 0);
    parity_ = declare_parameter<std::string>("parity", "");
    stop_bits_ = declare_parameter<int>("stop_bits", 0);
    antenna_frame_ = declare_parameter<std::string>("antenna_frame", "gps_link");
    poll_rate_hz_ = declare_parameter<double>("poll_rate_hz", 50.0);
    status_rate_hz_ = declare_parameter<double>("status_rate_hz", 5.0);
    fix_timeout_sec_ = declare_parameter<double>("fix_timeout_sec", 1.0);
    reconnect_period_sec_ = declare_parameter<double>("reconnect_period_sec", 2.0);
    quality_profile_valid_ = declare_parameter<bool>("quality_profile_valid", false);
    minimum_satellites_ = declare_parameter<int>("minimum_satellites", 1);
    maximum_hdop_ = declare_parameter<double>("maximum_hdop", 99.0);

    const auto sentence_ids = declare_parameter<std::vector<std::string>>(
      "accepted_sentence_ids", std::vector<std::string>{"GPGGA"});
    const auto fix_qualities = declare_parameter<std::vector<std::int64_t>>(
      "accepted_fix_qualities", std::vector<std::int64_t>{1, 2, 4, 5, 6, 8, 9});
    validateConfiguration(sentence_ids, fix_qualities);
    accepted_sentence_ids_.insert(sentence_ids.begin(), sentence_ids.end());
    for (const auto quality : fix_qualities) {
      quality_settings_.accepted_fix_qualities.push_back(static_cast<int>(quality));
    }
    quality_settings_.profile_valid = quality_profile_valid_;
    quality_settings_.minimum_satellites = minimum_satellites_;
    quality_settings_.maximum_hdop = maximum_hdop_;

    const auto fix_topic = declare_parameter<std::string>("fix_topic", "/gps/fix");
    const auto validated_fix_topic = declare_parameter<std::string>(
      "validated_fix_topic", "/gps/validated_fix");
    const auto valid_topic = declare_parameter<std::string>(
      "position_valid_topic", "/gps/gga_position_valid");
    const auto raw_topic = declare_parameter<std::string>(
      "raw_sentence_topic", "/gps/gga_sentence");
    if (fix_topic.empty() || validated_fix_topic.empty() || valid_topic.empty() ||
      raw_topic.empty())
    {
      throw std::invalid_argument("GGA output topics must not be empty");
    }
    if (fix_topic == validated_fix_topic) {
      throw std::invalid_argument("fix_topic and validated_fix_topic must be different");
    }

    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(fix_topic, rclcpp::QoS(10).reliable());
    validated_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      validated_fix_topic, rclcpp::QoS(10).reliable());
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(valid_topic, rclcpp::QoS(10).reliable());
    raw_pub_ = create_publisher<std_msgs::msg::String>(raw_topic, rclcpp::QoS(10).reliable());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / poll_rate_hz_),
      std::bind(&GgaSerialNode::onTimer, this));
    next_open_attempt_at_ = steadyNow();
    last_status_publish_at_ = steadyNow() - std::chrono::duration_cast<SteadyDuration>(
      std::chrono::duration<double>(1.0 / status_rate_hz_));
    publishValidity(false, true);

    RCLCPP_INFO(
      get_logger(), "configured position-only GGA input on '%s'; no recovery pose is published",
      device_.c_str());
  }

  ~GgaSerialNode() override {closeSerial();}

private:
  using SteadyClock = std::chrono::steady_clock;
  using SteadyDuration = SteadyClock::duration;

  std::size_t loadMaximumSentenceLength()
  {
    const int length = declare_parameter<int>(
      "maximum_sentence_length", static_cast<int>(kMaximumGgaSentenceLength));
    if (length < 16 || length > static_cast<int>(kMaximumGgaSentenceLength)) {
      throw std::invalid_argument("maximum_sentence_length must be from 16 through 128");
    }
    return static_cast<std::size_t>(length);
  }

  static bool finitePositive(double value) {return std::isfinite(value) && value > 0.0;}
  SteadyClock::time_point steadyNow() const {return SteadyClock::now();}

  void validateConfiguration(
    const std::vector<std::string> & sentence_ids,
    const std::vector<std::int64_t> & fix_qualities) const
  {
    if (device_.empty() || device_.front() != '/') {
      throw std::invalid_argument(
              "device must be an explicit absolute path; the GGA adapter never scans serial ports");
    }
    if (!baudConstant(baud_rate_)) {
      throw std::invalid_argument("baud_rate is unsupported or was not supplied explicitly");
    }
    if (data_bits_ < 5 || data_bits_ > 8 ||
      (parity_ != "none" && parity_ != "even" && parity_ != "odd") ||
      (stop_bits_ != 1 && stop_bits_ != 2))
    {
      throw std::invalid_argument(
              "data_bits, parity, and stop_bits must be supplied as a supported serial framing");
    }
    if (antenna_frame_.empty() || !finitePositive(poll_rate_hz_) ||
      !finitePositive(status_rate_hz_) || status_rate_hz_ > poll_rate_hz_ ||
      !finitePositive(fix_timeout_sec_) || !finitePositive(reconnect_period_sec_))
    {
      throw std::invalid_argument("GGA frames, rates, and timeouts must be valid");
    }
    if (sentence_ids.empty() || !std::all_of(
        sentence_ids.begin(), sentence_ids.end(), sentenceIdValid))
    {
      throw std::invalid_argument("accepted_sentence_ids must list explicit uppercase GGA IDs");
    }
    if (quality_profile_valid_) {
      if (fix_qualities.empty() || minimum_satellites_ <= 0 || !finitePositive(maximum_hdop_)) {
        throw std::invalid_argument(
                "an approved quality profile needs fix qualities, satellites, and HDOP limits");
      }
    }
    for (const auto quality : fix_qualities) {
      if (quality < 1 || quality > 9 || quality == 7) {
        throw std::invalid_argument(
                "accepted_fix_qualities must use documented nonzero, non-manual GGA qualities");
      }
    }
  }

  bool configureSerial(int descriptor) const
  {
    termios settings{};
    if (tcgetattr(descriptor, &settings) != 0) {return false;}
    cfmakeraw(&settings);
    settings.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    settings.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    switch (data_bits_) {
      case 5: settings.c_cflag |= CS5; break;
      case 6: settings.c_cflag |= CS6; break;
      case 7: settings.c_cflag |= CS7; break;
      case 8: settings.c_cflag |= CS8; break;
      default: return false;
    }
    settings.c_cflag &= static_cast<tcflag_t>(~(PARENB | PARODD | CSTOPB));
    if (parity_ == "even") {
      settings.c_cflag |= PARENB;
    } else if (parity_ == "odd") {
      settings.c_cflag |= static_cast<tcflag_t>(PARENB | PARODD);
    }
    if (stop_bits_ == 2) {settings.c_cflag |= CSTOPB;}
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    const auto speed = baudConstant(baud_rate_);
    if (!speed || cfsetispeed(&settings, *speed) != 0 || cfsetospeed(&settings, *speed) != 0 ||
      tcsetattr(descriptor, TCSANOW, &settings) != 0 || tcflush(descriptor, TCIOFLUSH) != 0)
    {
      return false;
    }
    return true;
  }

  bool openSerial()
  {
    const int descriptor = open(
      device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (descriptor < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "cannot open configured GPS serial device '%s': %s",
        device_.c_str(), std::strerror(errno));
      return false;
    }
    struct stat metadata {};
    if (fstat(descriptor, &metadata) != 0 || !S_ISCHR(metadata.st_mode) ||
      ioctl(descriptor, TIOCEXCL) != 0 || !configureSerial(descriptor))
    {
      const int saved_errno = errno;
      close(descriptor);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "configured GPS path '%s' is not an exclusively usable serial character device: %s",
        device_.c_str(), std::strerror(saved_errno));
      return false;
    }
    serial_fd_ = descriptor;
    line_buffer_.clear();
    // A reopened receiver starts a new transport session. tcflush() above removes
    // buffered bytes, so the first complete sentence becomes the new UTC baseline.
    last_utc_seconds_.reset();
    last_valid_fix_at_.reset();
    setPositionValid(false);
    RCLCPP_INFO(get_logger(), "opened configured GPS serial device '%s'", device_.c_str());
    return true;
  }

  void closeSerial()
  {
    if (serial_fd_ < 0) {return;}
    ioctl(serial_fd_, TIOCNXCL);
    close(serial_fd_);
    serial_fd_ = -1;
    line_buffer_.clear();
    last_valid_fix_at_.reset();
    setPositionValid(false);
  }

  bool qualityAccepted(const GgaFix & fix) const
  {
    return ggaPositionQualityAccepted(fix, quality_settings_);
  }

  std::int8_t navSatStatus(const GgaFix & fix) const
  {
    // NavSatStatus describes the receiver-reported fix. The separately
    // published position-valid Bool applies the local, field-approved quality gate.
    if (fix.fix_quality == 0 || fix.fix_quality == 7) {
      return sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    }
    if (fix.fix_quality == 9) {return sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;}
    if (fix.fix_quality == 2 || fix.fix_quality == 4 || fix.fix_quality == 5 ||
      fix.fix_quality == 8)
    {
      return sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    }
    return sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  }

  void processSentence(const std::string & sentence)
  {
    // Other NMEA sentence types are receiver chatter, not failed GGA samples.
    if (!hasGgaSentenceType(sentence)) {return;}

    GgaFix fix;
    std::string reason;
    if (!parseGgaSentence(sentence, fix, reason)) {
      setPositionValid(false);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "dropping invalid GGA sentence: %s", reason.c_str());
      return;
    }
    if (accepted_sentence_ids_.count(fix.sentence_id) == 0U) {
      setPositionValid(false);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "dropping unconfigured GGA sentence ID '%s'",
        fix.sentence_id.c_str());
      return;
    }
    if (last_utc_seconds_ && !ggaUtcStrictlyNewer(*last_utc_seconds_, fix.utc_seconds_of_day)) {
      setPositionValid(false);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "dropping repeated or regressing GGA UTC epoch");
      return;
    }
    last_utc_seconds_ = fix.utc_seconds_of_day;

    std_msgs::msg::String raw;
    raw.data = sentence;
    raw_pub_->publish(raw);

    const auto stamp = now();
    if (stamp.nanoseconds() <= 0) {
      setPositionValid(false);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "ROS receive time is unavailable; dropping GGA fix");
      return;
    }
    const bool accepted = qualityAccepted(fix);
    sensor_msgs::msg::NavSatFix output;
    output.header.stamp = stamp;
    output.header.frame_id = antenna_frame_;
    output.status.status = navSatStatus(fix);
    output.status.service = fix.sentence_id == "GPGGA" ?
      sensor_msgs::msg::NavSatStatus::SERVICE_GPS : 0U;
    output.latitude = fix.latitude_deg;
    output.longitude = fix.longitude_deg;
    // GGA altitude is mean-sea-level height. NavSatFix uses WGS84 ellipsoid height.
    output.altitude = fix.altitude_msl_m + fix.geoid_separation_m;
    output.position_covariance.fill(0.0);
    output.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    fix_pub_->publish(output);

    if (accepted) {
      validated_fix_pub_->publish(output);
      last_valid_fix_at_ = steadyNow();
      setPositionValid(true);
    } else {
      setPositionValid(false);
    }
  }

  void readAvailable()
  {
    pollfd descriptor{serial_fd_, POLLIN, 0};
    const int poll_result = poll(&descriptor, 1, 0);
    if (poll_result < 0 && errno != EINTR) {
      handleSerialFailure(std::string("poll failed: ") + std::strerror(errno));
      return;
    }
    if (poll_result <= 0) {return;}
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      handleSerialFailure("serial device reported disconnect or error");
      return;
    }
    if ((descriptor.revents & POLLIN) == 0) {return;}

    std::array<char, 512> bytes{};
    for (int reads = 0; reads < 16; ++reads) {
      const ssize_t count = read(serial_fd_, bytes.data(), bytes.size());
      if (count > 0) {
        for (const auto & sentence : line_buffer_.append(
            std::string_view(bytes.data(), static_cast<std::size_t>(count))))
        {
          processSentence(sentence);
        }
        continue;
      }
      if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {return;}
      if (errno == EINTR) {continue;}
      handleSerialFailure(std::string("serial read failed: ") + std::strerror(errno));
      return;
    }
  }

  void handleSerialFailure(const std::string & reason)
  {
    RCLCPP_ERROR(
      get_logger(), "%s on configured GPS device '%s'", reason.c_str(), device_.c_str());
    closeSerial();
    next_open_attempt_at_ = steadyNow() + std::chrono::duration_cast<SteadyDuration>(
      std::chrono::duration<double>(reconnect_period_sec_));
  }

  void setPositionValid(bool valid)
  {
    if (position_valid_ == valid) {return;}
    position_valid_ = valid;
    publishValidity(valid, true);
  }

  void publishValidity(bool valid, bool force)
  {
    const auto current = steadyNow();
    const double age = std::chrono::duration<double>(current - last_status_publish_at_).count();
    if (!force && age < 1.0 / status_rate_hz_) {return;}
    std_msgs::msg::Bool message;
    message.data = valid;
    valid_pub_->publish(message);
    last_status_publish_at_ = current;
  }

  void onTimer()
  {
    if (serial_fd_ < 0) {
      setPositionValid(false);
      if (steadyNow() >= next_open_attempt_at_ && !openSerial()) {
        next_open_attempt_at_ = steadyNow() + std::chrono::duration_cast<SteadyDuration>(
          std::chrono::duration<double>(reconnect_period_sec_));
      }
      publishValidity(position_valid_, false);
      return;
    }
    readAvailable();
    if (last_valid_fix_at_ &&
      std::chrono::duration<double>(steadyNow() - *last_valid_fix_at_).count() > fix_timeout_sec_)
    {
      last_valid_fix_at_.reset();
      setPositionValid(false);
    }
    publishValidity(position_valid_, false);
  }

  NmeaLineBuffer line_buffer_;
  std::string device_;
  int baud_rate_{0};
  int data_bits_{0};
  std::string parity_;
  int stop_bits_{0};
  std::string antenna_frame_;
  double poll_rate_hz_{50.0};
  double status_rate_hz_{5.0};
  double fix_timeout_sec_{1.0};
  double reconnect_period_sec_{2.0};
  bool quality_profile_valid_{false};
  int minimum_satellites_{1};
  double maximum_hdop_{99.0};
  int serial_fd_{-1};
  bool position_valid_{false};
  std::unordered_set<std::string> accepted_sentence_ids_;
  GgaQualitySettings quality_settings_;
  std::optional<double> last_utc_seconds_;
  std::optional<SteadyClock::time_point> last_valid_fix_at_;
  SteadyClock::time_point next_open_attempt_at_{};
  SteadyClock::time_point last_status_publish_at_{};
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr validated_fix_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ugv_localization_mvp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ugv_localization_mvp::GgaSerialNode>());
  rclcpp::shutdown();
  return 0;
}
