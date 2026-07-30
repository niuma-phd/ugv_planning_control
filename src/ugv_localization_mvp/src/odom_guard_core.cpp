#include "ugv_localization_mvp/odom_guard_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ugv_localization_mvp
{
namespace
{
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
    case OdomFault::kFrameMismatch: return "frame_mismatch";
    case OdomFault::kTranslationJump: return "translation_jump";
    case OdomFault::kYawJump: return "yaw_jump";
  }
  return "unknown";
}

OdomGuardCore::OdomGuardCore(OdomGuardSettings settings) : settings_(settings)
{
  if (!finite(settings_.quaternion_norm_tolerance) ||
    !finite(settings_.max_translation_jump_m) || !finite(settings_.max_yaw_jump_rad) ||
    settings_.quaternion_norm_tolerance < 0.0 || settings_.max_translation_jump_m < 0.0 ||
    settings_.max_yaw_jump_rad < 0.0)
  {
    throw std::invalid_argument("odom guard thresholds must be non-negative");
  }
}

OdomFault OdomGuardCore::evaluate(const OdomSample & sample)
{
  if (latched_) {return OdomFault::kAlreadyLatched;}
  const OdomFault fault = check(sample);
  if (fault != OdomFault::kNone) {
    latch(fault);
    return fault;
  }
  last_good_ = sample;
  has_last_good_ = true;
  return OdomFault::kNone;
}

OdomFault OdomGuardCore::check(const OdomSample & sample) const
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
  if (!has_last_good_) {return OdomFault::kNone;}

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
  const double norm = std::sqrt(
    sample.qx * sample.qx + sample.qy * sample.qy +
    sample.qz * sample.qz + sample.qw * sample.qw);
  const double qx = sample.qx / norm;
  const double qy = sample.qy / norm;
  const double qz = sample.qz / norm;
  const double qw = sample.qw / norm;
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
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
