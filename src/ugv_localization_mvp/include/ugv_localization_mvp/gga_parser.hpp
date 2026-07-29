#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ugv_localization_mvp
{

constexpr std::size_t kMaximumGgaSentenceLength = 128U;

struct GgaFix
{
  std::string sentence_id;
  double utc_seconds_of_day{0.0};
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_msl_m{0.0};
  double geoid_separation_m{0.0};
  int fix_quality{0};
  int satellites{0};
  double hdop{0.0};
  std::optional<double> differential_age_s;
  std::string station_id;
};

struct GgaQualitySettings
{
  bool profile_valid{false};
  std::vector<int> accepted_fix_qualities;
  int minimum_satellites{0};
  double maximum_hdop{0.0};
};

bool parseGgaSentence(std::string_view sentence, GgaFix & fix, std::string & reason);
bool ggaUtcStrictlyNewer(double previous_seconds, double current_seconds);
bool ggaPositionQualityAccepted(const GgaFix & fix, const GgaQualitySettings & settings);

class NmeaLineBuffer
{
public:
  explicit NmeaLineBuffer(std::size_t maximum_sentence_length = kMaximumGgaSentenceLength);
  std::vector<std::string> append(std::string_view bytes);
  void clear();

private:
  std::size_t maximum_sentence_length_;
  std::string current_;
  bool discarding_{false};
};

}  // namespace ugv_localization_mvp
