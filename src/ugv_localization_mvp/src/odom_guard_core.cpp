#include "ugv_localization_mvp/odom_guard_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ugv_localization_mvp
{
namespace
{
constexpr double kNsPerSecond = 1.0e9;
constexpr double kPi = 3.14159265358979323846;

bool finite(double value) {return std::isfinite(value);}
double wrappedAbs(double angle)
{
  return std::abs(std::remainder(angle, 2.0 * kPi));
}
}  // namespace

const char * toString(OdomFault fault)
{
  switch (fault) {
    case OdomFault::kNone: return "none";
    case OdomFault::kAlreadyLatched: return "already_latched";
    case OdomFault::kNonFinite: return "non_finite";
    case OdomFault::kInvalidQuaternion: return "invalid_quaternion";
    case OdomFault::kStale: return "stale";
    case OdomFault::kFutureStamp: return "future_stamp";
    case OdomFault::kRepeatedStamp: return "repeated_stamp";
    case OdomFault::kBackwardStamp: return "backward_stamp";
    case OdomFault::kTranslationJump: return "translation_jump";
    case OdomFault::kYawJump: return "yaw_jump";
  }
  return "unknown";
}

OdomGuardCore::OdomGuardCore(OdomGuardSettings settings) : settings_(settings)
{
  if (settings_.max_age_s < 0.0 || settings_.future_tolerance_s < 0.0 ||
    settings_.quaternion_norm_tolerance < 0.0 || settings_.max_translation_jump_m < 0.0 ||
    settings_.max_yaw_jump_rad < 0.0)
  {
    throw std::invalid_argument("odom guard thresholds must be non-negative");
  }
}

OdomFault OdomGuardCore::evaluate(const OdomSample & sample, std::int64_t now_ns)
{
  if (latched_) {return OdomFault::kAlreadyLatched;}
  const OdomFault fault = check(sample, now_ns);
  if (fault != OdomFault::kNone) {
    latch(fault);
    return fault;
  }
  last_good_ = sample;
  has_last_good_ = true;
  return OdomFault::kNone;
}

OdomFault OdomGuardCore::check(const OdomSample & sample, std::int64_t now_ns) const
{
  if (!sample.auxiliary_finite || !finite(sample.x) || !finite(sample.y) || !finite(sample.z) ||
    !finite(sample.qx) || !finite(sample.qy) || !finite(sample.qz) || !finite(sample.qw) ||
    !finite(sample.vx) || !finite(sample.vy) || !finite(sample.vz) ||
    !finite(sample.wx) || !finite(sample.wy) || !finite(sample.wz))
  {
    return OdomFault::kNonFinite;
  }
  const double q_norm = std::sqrt(
    sample.qx * sample.qx + sample.qy * sample.qy + sample.qz * sample.qz + sample.qw * sample.qw);
  if (q_norm < 1.0e-12 || std::abs(q_norm - 1.0) > settings_.quaternion_norm_tolerance) {
    return OdomFault::kInvalidQuaternion;
  }
  const double age_s = static_cast<double>(now_ns - sample.stamp_ns) / kNsPerSecond;
  if (age_s > settings_.max_age_s) {return OdomFault::kStale;}
  if (age_s < -settings_.future_tolerance_s) {return OdomFault::kFutureStamp;}
  if (!has_last_good_) {return OdomFault::kNone;}
  if (sample.stamp_ns == last_good_.stamp_ns) {return OdomFault::kRepeatedStamp;}
  if (sample.stamp_ns < last_good_.stamp_ns) {return OdomFault::kBackwardStamp;}

  const double dx = sample.x - last_good_.x;
  const double dy = sample.y - last_good_.y;
  const double dz = sample.z - last_good_.z;
  if (std::sqrt(dx * dx + dy * dy + dz * dz) > settings_.max_translation_jump_m) {
    return OdomFault::kTranslationJump;
  }
  if (wrappedAbs(yaw(sample) - yaw(last_good_)) > settings_.max_yaw_jump_rad) {
    return OdomFault::kYawJump;
  }
  return OdomFault::kNone;
}

double OdomGuardCore::yaw(const OdomSample & sample)
{
  const double siny_cosp = 2.0 * (sample.qw * sample.qz + sample.qx * sample.qy);
  const double cosy_cosp = 1.0 - 2.0 * (sample.qy * sample.qy + sample.qz * sample.qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

void OdomGuardCore::latch(OdomFault fault)
{
  latched_ = true;
  latched_fault_ = fault;
}

void OdomGuardCore::reset()
{
  latched_ = false;
  latched_fault_ = OdomFault::kNone;
  has_last_good_ = false;
  last_good_ = OdomSample{};
}

}  // namespace ugv_localization_mvp
