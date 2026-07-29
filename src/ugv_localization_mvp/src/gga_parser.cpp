#include "ugv_localization_mvp/gga_parser.hpp"

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <utility>

namespace ugv_localization_mvp
{
namespace
{

bool decimalNumber(std::string_view text)
{
  if (text.empty()) {return false;}
  std::size_t index = (text.front() == '-' || text.front() == '+') ? 1U : 0U;
  if (index == text.size()) {return false;}
  bool decimal_seen = false;
  bool digit_seen = false;
  for (; index < text.size(); ++index) {
    const char character = text[index];
    if (character == '.' && !decimal_seen) {
      decimal_seen = true;
      continue;
    }
    if (character < '0' || character > '9') {return false;}
    digit_seen = true;
  }
  return digit_seen && text.back() != '.';
}

bool parseFiniteDouble(std::string_view text, double & value)
{
  if (!decimalNumber(text)) {return false;}
  std::string owned(text);
  char * end = nullptr;
  errno = 0;
  value = std::strtod(owned.c_str(), &end);
  return errno != ERANGE && end == owned.c_str() + owned.size() && std::isfinite(value);
}

bool parseInteger(std::string_view text, int & value)
{
  if (text.empty()) {return false;}
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

bool decimalDigits(std::string_view text)
{
  return !text.empty() && std::all_of(
    text.begin(), text.end(), [](char character) {return character >= '0' && character <= '9';});
}

bool parseUtc(std::string_view text, double & seconds_of_day)
{
  if (text.size() < 6U || !decimalDigits(text.substr(0U, 6U))) {return false;}
  if (text.size() > 6U &&
    (text[6] != '.' || text.size() == 7U || !decimalDigits(text.substr(7U))))
  {
    return false;
  }
  const int hour = (text[0] - '0') * 10 + text[1] - '0';
  const int minute = (text[2] - '0') * 10 + text[3] - '0';
  double second = 0.0;
  if (!parseFiniteDouble(text.substr(4U), second) || hour > 23 || minute > 59 ||
    second < 0.0 || second >= 60.0)
  {
    return false;
  }
  seconds_of_day = static_cast<double>(hour * 3600 + minute * 60) + second;
  return true;
}

bool parseCoordinate(
  std::string_view value, std::string_view direction, bool latitude, double & degrees)
{
  const std::size_t degree_digits = latitude ? 2U : 3U;
  const auto decimal = value.find('.');
  if (decimal != degree_digits + 2U || decimal == std::string_view::npos ||
    decimal + 1U == value.size() || !decimalDigits(value.substr(0U, decimal)) ||
    !decimalDigits(value.substr(decimal + 1U)))
  {
    return false;
  }
  int whole_degrees = 0;
  if (!parseInteger(value.substr(0U, degree_digits), whole_degrees)) {return false;}
  double minutes = 0.0;
  if (!parseFiniteDouble(value.substr(degree_digits), minutes) || minutes < 0.0 ||
    minutes >= 60.0)
  {
    return false;
  }
  const int maximum_degrees = latitude ? 90 : 180;
  if (whole_degrees > maximum_degrees ||
    (whole_degrees == maximum_degrees && minutes != 0.0))
  {
    return false;
  }
  if (direction.size() != 1U) {return false;}
  const char positive = latitude ? 'N' : 'E';
  const char negative = latitude ? 'S' : 'W';
  if (direction[0] != positive && direction[0] != negative) {return false;}
  degrees = static_cast<double>(whole_degrees) + minutes / 60.0;
  if (direction[0] == negative) {degrees = -degrees;}
  return true;
}

std::vector<std::string_view> splitFields(std::string_view payload)
{
  std::vector<std::string_view> fields;
  std::size_t begin = 0U;
  while (begin <= payload.size()) {
    const auto end = payload.find(',', begin);
    if (end == std::string_view::npos) {
      fields.push_back(payload.substr(begin));
      break;
    }
    fields.push_back(payload.substr(begin, end - begin));
    begin = end + 1U;
  }
  return fields;
}

bool validStationId(std::string_view station_id)
{
  return station_id.size() <= 4U && std::all_of(
    station_id.begin(), station_id.end(), [](char character) {
      return (character >= '0' && character <= '9') ||
             (character >= 'A' && character <= 'Z') ||
             (character >= 'a' && character <= 'z');
    });
}

}  // namespace

bool parseGgaSentence(std::string_view sentence, GgaFix & fix, std::string & reason)
{
  while (!sentence.empty() && (sentence.back() == '\r' || sentence.back() == '\n')) {
    sentence.remove_suffix(1U);
  }
  if (sentence.empty() || sentence.size() > kMaximumGgaSentenceLength) {
    reason = "GGA sentence is empty or too long";
    return false;
  }
  if (std::any_of(sentence.begin(), sentence.end(), [](char character) {
      const auto byte = static_cast<unsigned char>(character);
      return byte < 0x20U || byte > 0x7eU;
    }))
  {
    reason = "GGA sentence contains non-printable or non-ASCII bytes";
    return false;
  }
  if (sentence.front() != '$') {
    reason = "GGA sentence must begin with '$'";
    return false;
  }
  const auto asterisk = sentence.find('*');
  if (asterisk == std::string_view::npos || asterisk + 3U != sentence.size() ||
    sentence.find('*', asterisk + 1U) != std::string_view::npos)
  {
    reason = "GGA sentence must end in one two-digit checksum";
    return false;
  }
  unsigned int expected_checksum = 0U;
  const auto checksum_result = std::from_chars(
    sentence.data() + asterisk + 1U, sentence.data() + sentence.size(), expected_checksum, 16);
  if (checksum_result.ec != std::errc{} || checksum_result.ptr != sentence.data() + sentence.size() ||
    expected_checksum > 0xffU)
  {
    reason = "GGA checksum is not hexadecimal";
    return false;
  }
  unsigned int actual_checksum = 0U;
  for (std::size_t index = 1U; index < asterisk; ++index) {
    actual_checksum ^= static_cast<unsigned char>(sentence[index]);
  }
  if (actual_checksum != expected_checksum) {
    reason = "GGA checksum mismatch";
    return false;
  }

  const auto fields = splitFields(sentence.substr(1U, asterisk - 1U));
  if (fields.size() != 15U) {
    reason = "GGA sentence must contain exactly 15 comma-separated fields";
    return false;
  }
  if (fields[0].size() != 5U || fields[0].substr(2U) != "GGA" ||
    fields[0][0] < 'A' || fields[0][0] > 'Z' || fields[0][1] < 'A' || fields[0][1] > 'Z')
  {
    reason = "NMEA sentence is not an uppercase two-letter GGA talker";
    return false;
  }

  GgaFix parsed;
  parsed.sentence_id = std::string(fields[0]);
  if (!parseUtc(fields[1], parsed.utc_seconds_of_day)) {
    reason = "GGA UTC field is invalid";
    return false;
  }
  if (!parseCoordinate(fields[2], fields[3], true, parsed.latitude_deg) ||
    !parseCoordinate(fields[4], fields[5], false, parsed.longitude_deg))
  {
    reason = "GGA latitude or longitude is invalid";
    return false;
  }
  if (!parseInteger(fields[6], parsed.fix_quality) || parsed.fix_quality < 0 ||
    parsed.fix_quality > 9)
  {
    reason = "GGA fix quality must be an integer from 0 through 9";
    return false;
  }
  if (!parseInteger(fields[7], parsed.satellites) || parsed.satellites < 0 ||
    parsed.satellites > 99)
  {
    reason = "GGA satellite count is invalid";
    return false;
  }
  if (!parseFiniteDouble(fields[8], parsed.hdop) || parsed.hdop < 0.0) {
    reason = "GGA HDOP is invalid";
    return false;
  }
  if (!parseFiniteDouble(fields[9], parsed.altitude_msl_m) || fields[10] != "M" ||
    !parseFiniteDouble(fields[11], parsed.geoid_separation_m) || fields[12] != "M")
  {
    reason = "GGA altitude and geoid separation must be finite metres";
    return false;
  }
  if (!fields[13].empty()) {
    double age = 0.0;
    if (!parseFiniteDouble(fields[13], age) || age < 0.0) {
      reason = "GGA differential age is invalid";
      return false;
    }
    parsed.differential_age_s = age;
  }
  if (!validStationId(fields[14])) {
    reason = "GGA station ID must be at most four alphanumeric characters";
    return false;
  }
  parsed.station_id = std::string(fields[14]);
  fix = std::move(parsed);
  reason.clear();
  return true;
}

bool ggaUtcStrictlyNewer(double previous_seconds, double current_seconds)
{
  if (!std::isfinite(previous_seconds) || !std::isfinite(current_seconds) ||
    previous_seconds < 0.0 || previous_seconds >= 86400.0 || current_seconds < 0.0 ||
    current_seconds >= 86400.0)
  {
    return false;
  }
  if (current_seconds > previous_seconds) {return true;}
  return previous_seconds - current_seconds > 43200.0;
}

bool ggaPositionQualityAccepted(const GgaFix & fix, const GgaQualitySettings & settings)
{
  return settings.profile_valid && settings.minimum_satellites > 0 &&
         std::isfinite(settings.maximum_hdop) && settings.maximum_hdop > 0.0 &&
         std::find(
           settings.accepted_fix_qualities.begin(), settings.accepted_fix_qualities.end(),
           fix.fix_quality) != settings.accepted_fix_qualities.end() &&
         fix.satellites >= settings.minimum_satellites && fix.hdop > 0.0 &&
         fix.hdop <= settings.maximum_hdop;
}

NmeaLineBuffer::NmeaLineBuffer(std::size_t maximum_sentence_length)
: maximum_sentence_length_(maximum_sentence_length)
{
  if (maximum_sentence_length_ == 0U) {
    throw std::invalid_argument("maximum NMEA sentence length must be positive");
  }
}

std::vector<std::string> NmeaLineBuffer::append(std::string_view bytes)
{
  std::vector<std::string> sentences;
  for (const char character : bytes) {
    if (character == '$') {
      current_.assign(1U, character);
      discarding_ = false;
      continue;
    }
    if (character == '\r' || character == '\n') {
      if (!discarding_ && !current_.empty()) {sentences.push_back(current_);}
      current_.clear();
      discarding_ = false;
      continue;
    }
    const auto byte = static_cast<unsigned char>(character);
    if (discarding_ || current_.empty()) {continue;}
    if (byte < 0x20U || byte > 0x7eU || current_.size() >= maximum_sentence_length_) {
      current_.clear();
      discarding_ = true;
      continue;
    }
    current_.push_back(character);
  }
  return sentences;
}

void NmeaLineBuffer::clear()
{
  current_.clear();
  discarding_ = false;
}

}  // namespace ugv_localization_mvp
