#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "ugv_localization_mvp/gga_parser.hpp"

namespace
{

std::string nmea(const std::string & payload)
{
  unsigned int checksum = 0U;
  for (const unsigned char byte : payload) {checksum ^= byte;}
  std::ostringstream sentence;
  sentence << '$' << payload << '*' << std::uppercase << std::hex << std::setw(2) <<
    std::setfill('0') << checksum;
  return sentence.str();
}

const std::string kDocumentPayload =
  "GPGGA,024941.00,3110.4693903,N,12123.2621695,E,1,16,0.6,57.0924,M,0.00,M,99,AAAA";

}  // namespace

TEST(GgaParser, RejectsIncorrectDocumentChecksumAndAcceptsCorrectedSentence)
{
  ugv_localization_mvp::GgaFix fix;
  std::string reason;
  EXPECT_FALSE(ugv_localization_mvp::parseGgaSentence(
      "$" + kDocumentPayload + "*55", fix, reason));
  EXPECT_EQ(reason, "GGA checksum mismatch");

  ASSERT_EQ(nmea(kDocumentPayload).substr(nmea(kDocumentPayload).size() - 3U), "*65");
  ASSERT_TRUE(ugv_localization_mvp::parseGgaSentence(nmea(kDocumentPayload), fix, reason));
  EXPECT_EQ(fix.sentence_id, "GPGGA");
  EXPECT_NEAR(fix.utc_seconds_of_day, 2.0 * 3600.0 + 49.0 * 60.0 + 41.0, 1.0e-9);
  EXPECT_NEAR(fix.latitude_deg, 31.174489838333, 1.0e-12);
  EXPECT_NEAR(fix.longitude_deg, 121.387702825, 1.0e-12);
  EXPECT_EQ(fix.fix_quality, 1);
  EXPECT_EQ(fix.satellites, 16);
  EXPECT_DOUBLE_EQ(fix.hdop, 0.6);
  EXPECT_DOUBLE_EQ(fix.altitude_msl_m, 57.0924);
  EXPECT_DOUBLE_EQ(fix.geoid_separation_m, 0.0);
  ASSERT_TRUE(fix.differential_age_s);
  EXPECT_DOUBLE_EQ(*fix.differential_age_s, 99.0);
  EXPECT_EQ(fix.station_id, "AAAA");
}

TEST(GgaParser, AppliesSouthernAndWesternHemisphereSigns)
{
  const auto sentence = nmea(
    "GNGGA,235959.50,3459.5000000,S,12345.2500000,W,4,12,0.8,10.0,M,-2.0,M,,0012");
  ugv_localization_mvp::GgaFix fix;
  std::string reason;
  ASSERT_TRUE(ugv_localization_mvp::parseGgaSentence(sentence + "\r\n", fix, reason));
  EXPECT_NEAR(fix.latitude_deg, -(34.0 + 59.5 / 60.0), 1.0e-12);
  EXPECT_NEAR(fix.longitude_deg, -(123.0 + 45.25 / 60.0), 1.0e-12);
  EXPECT_FALSE(fix.differential_age_s);
}

TEST(GgaParser, RejectsMalformedOrUnsafeFields)
{
  const std::vector<std::string> payloads = {
    "GPRMC,024941.00,3110.4693903,N,12123.2621695,E,1,16,0.6,57.0,M,0.0,M,,0001",
    "GPGGA,246000.00,3110.4693903,N,12123.2621695,E,1,16,0.6,57.0,M,0.0,M,,0001",
    "GPGGA,024941.00,3160.0000000,N,12123.2621695,E,1,16,0.6,57.0,M,0.0,M,,0001",
    "GPGGA,024941.00,3110.4693903,Q,12123.2621695,E,1,16,0.6,57.0,M,0.0,M,,0001",
    "GPGGA,024941.00,3110.4693903,N,18100.0000000,E,1,16,0.6,57.0,M,0.0,M,,0001",
    "GPGGA,024941.00,3110.4693903,N,12123.2621695,E,10,16,0.6,57.0,M,0.0,M,,0001",
    "GPGGA,024941.00,3110.4693903,N,12123.2621695,E,1,100,0.6,57.0,M,0.0,M,,0001",
    "GPGGA,024941.00,3110.4693903,N,12123.2621695,E,1,16,nan,57.0,M,0.0,M,,0001",
    "GPGGA,024941.00,3110.4693903,N,12123.2621695,E,1,16, 0.6,57.0,M,0.0,M,,0001",
    "GPGGA,024941.00,3110.4693903,N,12123.2621695,E,1,16,6e-1,57.0,M,0.0,M,,0001",
    "GPGGA,024941.00,3110.4693903,N,12123.2621695,E,1,16,0.6,57.0,F,0.0,M,,0001",
    "GPGGA,024941.00,3110.4693903,N,12123.2621695,E,1,16,0.6,57.0,M,0.0,M,-1,0001",
    "GPGGA,024941.00,3110.4693903,N,12123.2621695,E,1,16,0.6,57.0,M,0.0,M,,TOO-LONG",
    "GPGGA,024941.00,3110.4693903,N,12123.2621695,E,1,16,0.6,57.0,M,0.0,M,",
  };
  for (const auto & payload : payloads) {
    ugv_localization_mvp::GgaFix fix;
    std::string reason;
    EXPECT_FALSE(ugv_localization_mvp::parseGgaSentence(nmea(payload), fix, reason)) << payload;
    EXPECT_FALSE(reason.empty()) << payload;
  }

  std::string embedded_nul = nmea(kDocumentPayload);
  embedded_nul[10] = '\0';
  ugv_localization_mvp::GgaFix fix;
  std::string reason;
  EXPECT_FALSE(ugv_localization_mvp::parseGgaSentence(embedded_nul, fix, reason));
}

TEST(GgaParser, PreservesFixQualityForIndependentQualityGate)
{
  const auto no_fix_sentence = nmea(
    "GPGGA,024941.00,3110.4693903,N,12123.2621695,E,0,00,9.9,57.0,M,0.0,M,,");
  ugv_localization_mvp::GgaFix fix;
  std::string reason;
  ASSERT_TRUE(ugv_localization_mvp::parseGgaSentence(no_fix_sentence, fix, reason));
  EXPECT_EQ(fix.fix_quality, 0);

  ugv_localization_mvp::GgaQualitySettings settings;
  settings.profile_valid = true;
  settings.accepted_fix_qualities = {1, 2, 4, 5, 6, 8, 9};
  settings.minimum_satellites = 4;
  settings.maximum_hdop = 2.0;
  EXPECT_FALSE(ugv_localization_mvp::ggaPositionQualityAccepted(fix, settings));

  ASSERT_TRUE(ugv_localization_mvp::parseGgaSentence(nmea(kDocumentPayload), fix, reason));
  EXPECT_TRUE(ugv_localization_mvp::ggaPositionQualityAccepted(fix, settings));
  settings.profile_valid = false;
  EXPECT_FALSE(ugv_localization_mvp::ggaPositionQualityAccepted(fix, settings));
  settings.profile_valid = true;
  settings.minimum_satellites = 17;
  EXPECT_FALSE(ugv_localization_mvp::ggaPositionQualityAccepted(fix, settings));
  settings.minimum_satellites = 4;
  settings.maximum_hdop = 0.5;
  EXPECT_FALSE(ugv_localization_mvp::ggaPositionQualityAccepted(fix, settings));
}

TEST(GgaParser, RejectsRepeatedOrRegressingUtcAndAllowsMidnightRollover)
{
  EXPECT_TRUE(ugv_localization_mvp::ggaUtcStrictlyNewer(100.0, 100.1));
  EXPECT_FALSE(ugv_localization_mvp::ggaUtcStrictlyNewer(100.0, 100.0));
  EXPECT_FALSE(ugv_localization_mvp::ggaUtcStrictlyNewer(100.0, 99.0));
  EXPECT_TRUE(ugv_localization_mvp::ggaUtcStrictlyNewer(86399.9, 0.1));
  EXPECT_FALSE(ugv_localization_mvp::ggaUtcStrictlyNewer(-1.0, 0.0));
}

TEST(GgaParser, LineBufferHandlesNoiseChunksMultipleLinesAndOversizeInput)
{
  const auto first = nmea(kDocumentPayload);
  const auto second = nmea(
    "GPGGA,024942.00,3110.4693903,N,12123.2621695,E,1,16,0.6,57.0,M,0.0,M,,0001");
  ugv_localization_mvp::NmeaLineBuffer buffer;

  EXPECT_TRUE(buffer.append("noise" + first.substr(0U, 20U)).empty());
  auto sentences = buffer.append(first.substr(20U) + "\r\n" + second + "\n");
  ASSERT_EQ(sentences.size(), 2U);
  EXPECT_EQ(sentences[0], first);
  EXPECT_EQ(sentences[1], second);

  const std::string oversize = "$" + std::string(200U, 'X') + "\r\n";
  sentences = buffer.append(oversize + second + "\r\n");
  ASSERT_EQ(sentences.size(), 1U);
  EXPECT_EQ(sentences[0], second);

  std::string binary = "$BROKEN";
  binary.push_back('\0');
  binary += "TAIL\r\n";
  EXPECT_TRUE(buffer.append(binary).empty());
}
