#include "link/kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace robotlink {

WheelSpeeds mecanumInverse(f32 vx, f32 vy, f32 wz,
                           f32 wheel_radius, f32 half_length, f32 half_width) {
    const f32 k = half_length + half_width;
    WheelSpeeds w;
    w.fl = (vx - vy - k * wz) / wheel_radius;
    w.fr = (vx + vy + k * wz) / wheel_radius;
    w.rl = (vx + vy - k * wz) / wheel_radius;
    w.rr = (vx - vy + k * wz) / wheel_radius;
    return w;
}

bool clampJoints(const JointLimit* limits, f32* joints, std::size_t n) {
    bool ok = true;
    for (std::size_t i = 0; i < n; ++i) {
        if (joints[i] < limits[i].min_rad) { joints[i] = limits[i].min_rad; ok = false; }
        if (joints[i] > limits[i].max_rad) { joints[i] = limits[i].max_rad; ok = false; }
    }
    return ok;
}

} // namespace robotlink
