#include "ugv_subject2_mvp/waypoint_file_loader.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ugv_subject2_mvp
{
namespace
{

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1U);
}

std::vector<std::string> split_csv_line(const std::string & line, const std::size_t line_number)
{
  if (line.find('"') != std::string::npos) {
    throw std::runtime_error(
            "waypoint CSV line " + std::to_string(line_number) +
            " uses quoted fields, which are not supported");
  }
  std::vector<std::string> fields;
  std::size_t begin = 0U;
  while (true) {
    const auto comma = line.find(',', begin);
    fields.push_back(trim(line.substr(begin, comma - begin)));
    if (comma == std::string::npos) {
      break;
    }
    begin = comma + 1U;
  }
  return fields;
}

double parse_finite_number(
  const std::string & value, const std::string & column, const std::size_t line_number)
{
  if (value.empty()) {
    throw std::runtime_error(
            "waypoint CSV line " + std::to_string(line_number) +
            " has an empty '" + column + "' value");
  }
  errno = 0;
  char * end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (errno == ERANGE || end != value.c_str() + value.size() || !std::isfinite(parsed)) {
    throw std::runtime_error(
            "waypoint CSV line " + std::to_string(line_number) +
            " has an invalid finite '" + column + "' value: '" + value + "'");
  }
  return parsed;
}

}  // namespace

std::vector<FileWaypoint> load_waypoint_csv(const std::string & file_path)
{
  if (file_path.empty()) {
    throw std::invalid_argument("waypoint_file must be a non-empty absolute path");
  }
  const std::filesystem::path path(file_path);
  if (!path.is_absolute()) {
    throw std::invalid_argument("waypoint_file must be an absolute path: " + file_path);
  }
  std::error_code status_error;
  if (!std::filesystem::is_regular_file(path, status_error)) {
    throw std::runtime_error("waypoint_file is not a readable regular file: " + file_path);
  }
  const auto initial_file_size = std::filesystem::file_size(path, status_error);
  if (status_error) {
    throw std::runtime_error("cannot determine waypoint CSV size: " + file_path);
  }
  if (initial_file_size > kMaximumWaypointFileBytes) {
    throw std::runtime_error(
            "waypoint CSV exceeds the 2 MiB file-size limit: " + file_path);
  }

  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::runtime_error("cannot open waypoint CSV: " + file_path);
  }

  std::unordered_map<std::string, std::size_t> columns;
  std::vector<FileWaypoint> waypoints;
  std::string line;
  std::size_t line_number = 0U;
  std::size_t column_count = 0U;
  bool have_header = false;

  const auto consume_line = [&](const std::string & raw_line, const std::size_t current_line) {
    line_number = current_line;
    const std::string content = trim(raw_line);
    if (content.empty() || content.front() == '#') {
      return;
    }

    const auto fields = split_csv_line(content, current_line);
    if (!have_header) {
      column_count = fields.size();
      for (std::size_t index = 0U; index < fields.size(); ++index) {
        const auto & name = fields[index];
        if (name != "x_m" && name != "y_m" && name != "z_m" && name != "yaw_rad") {
          throw std::runtime_error(
                  "waypoint CSV line " + std::to_string(line_number) +
                  " has unsupported column '" + name + "'");
        }
        if (!columns.emplace(name, index).second) {
          throw std::runtime_error("waypoint CSV has duplicate column '" + name + "'");
        }
      }
      if (columns.count("x_m") == 0U || columns.count("y_m") == 0U) {
        throw std::runtime_error("waypoint CSV header must contain x_m and y_m");
      }
      have_header = true;
      return;
    }

    if (fields.size() != column_count) {
      throw std::runtime_error(
              "waypoint CSV line " + std::to_string(line_number) +
              " has " + std::to_string(fields.size()) + " fields; expected " +
              std::to_string(column_count));
    }

    FileWaypoint waypoint;
    waypoint.x_m = parse_finite_number(fields.at(columns.at("x_m")), "x_m", line_number);
    waypoint.y_m = parse_finite_number(fields.at(columns.at("y_m")), "y_m", line_number);
    if (const auto found = columns.find("z_m"); found != columns.end()) {
      waypoint.z_m = parse_finite_number(fields.at(found->second), "z_m", line_number);
    }
    if (const auto found = columns.find("yaw_rad"); found != columns.end()) {
      waypoint.yaw_rad = parse_finite_number(fields.at(found->second), "yaw_rad", line_number);
    }
    if (waypoints.size() >= kMaximumWaypointCount) {
      throw std::runtime_error("waypoint CSV exceeds the 10000-waypoint limit");
    }
    waypoints.push_back(std::move(waypoint));
  };

  // Do not rely only on the pre-open size check: the file may grow after that
  // check. Reading one byte at a time keeps both the cumulative allocation and
  // each line bounded even under a concurrent file replacement or append.
  std::uintmax_t bytes_read = 0U;
  std::size_t current_line = 1U;
  char character = '\0';
  while (input.get(character)) {
    ++bytes_read;
    if (bytes_read > kMaximumWaypointFileBytes) {
      throw std::runtime_error("waypoint CSV exceeded the 2 MiB limit while reading");
    }
    if (character == '\n') {
      consume_line(line, current_line);
      line.clear();
      ++current_line;
      continue;
    }
    if (line.size() >= kMaximumWaypointLineBytes) {
      throw std::runtime_error(
              "waypoint CSV line " + std::to_string(current_line) +
              " exceeds the 4096-byte line limit");
    }
    line.push_back(character);
  }
  if (!line.empty()) {
    consume_line(line, current_line);
  }

  if (input.bad()) {
    throw std::runtime_error("failed while reading waypoint CSV: " + file_path);
  }
  if (!have_header) {
    throw std::runtime_error("waypoint CSV has no header");
  }
  if (waypoints.size() < 2U) {
    throw std::runtime_error("waypoint CSV must contain at least two waypoints");
  }

  bool has_nonzero_segment = false;
  for (std::size_t index = 1U; index < waypoints.size(); ++index) {
    if (waypoints[index].x_m != waypoints[index - 1U].x_m ||
      waypoints[index].y_m != waypoints[index - 1U].y_m)
    {
      has_nonzero_segment = true;
      break;
    }
  }
  if (!has_nonzero_segment) {
    throw std::runtime_error("waypoint CSV must contain at least one non-zero-length segment");
  }
  return waypoints;
}

}  // namespace ugv_subject2_mvp
