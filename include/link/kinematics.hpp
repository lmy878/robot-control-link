#pragma once
#include "link/common.hpp"

namespace robotlink {

struct WheelSpeeds { f32 fl = 0, fr = 0, rl = 0, rr = 0; };

// Omnidirectional (mecanum) inverse kinematics: body velocity -> wheel speeds.
//   vx : forward (m/s), vy : left (m/s), wz : yaw rate (rad/s)
//   wheel_radius : m, half_length/half_width : m
WheelSpeeds mecanumInverse(f32 vx, f32 vy, f32 wz,
                           f32 wheel_radius, f32 half_length, f32 half_width);

struct JointLimit { f32 min_rad = 0, max_rad = 0, max_vel = 0; };

// Clamp in place to [min,max]; returns false if any target was out of range.
bool clampJoints(const JointLimit* limits, f32* joints, std::size_t n);

} // namespace robotlink
