#pragma once

#include <string>

namespace ugv_localization_mvp
{

enum class OdomFault
{
  kNone,
  kAlreadyLatched,
  kNonFinite,
  kInvalidQuaternion,
  kFrameMismatch,
  kTranslationJump,
  kYawJump,
};

const char * toString(OdomFault fault);

struct OdomGuardSettings
{
  double quaternion_norm_tolerance{0.05};
  double max_translation_jump_m{1.50};
  double max_yaw_jump_rad{0.80};
};

struct OdomSample
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double wx{0.0};
  double wy{0.0};
  double wz{0.0};
  bool auxiliary_finite{true};
};

class OdomGuardCore
{
public:
  explicit OdomGuardCore(OdomGuardSettings settings = {});

  OdomFault evaluate(const OdomSample & sample);
  void reset();
  bool latched() const {return latched_;}
  bool hasLastGood() const {return has_last_good_;}
  OdomFault latchedFault() const {return latched_fault_;}

private:
  OdomFault check(const OdomSample & sample) const;
  static double yaw(const OdomSample & sample);
  void latch(OdomFault fault);

  OdomGuardSettings settings_;
  bool latched_{false};
  OdomFault latched_fault_{OdomFault::kNone};
  bool has_last_good_{false};
  OdomSample last_good_{};
};

}  // namespace ugv_localization_mvp
