// Analytical kinematics of the tog-sim arm: Epson GX8-C653S SCARA (J1,J2 revolute, J3 prismatic, J4 revolute)
// + tilt axis (J5, revolute about the tool-local X axis) + fixed tool to the suction-cup face (TCP).
//
// Frame conventions (see togsim_description):
//   base_link : robot mounting surface, z up.
//   flange    : j4_link origin = quill tip; its z axis is up, rotated by yaw = J1+J2+J4 about z.
//   tilt axis : at flange - (0,0,tilt_pivot_drop), direction = flange x axis.
//   tool axis : at J5 = 0 points along -z (down); TCP = pivot + R_flange * R_x(J5) * (0,0,-tool_length).
//   TCP pose  : position p, yaw psi (tool x axis heading about world z), tilt theta (J5).
//
// Header-only; no ROS dependency so it can be unit-tested and reused from Python via pybind later.
#pragma once
#include <array>
#include <cmath>
#include <optional>

namespace togsim {

struct KinematicParams {
  double l1 = 0.40;
  double l2 = 0.25;
  double z_flange0 = 0.299;      // flange height above base_link at J3 = 0
  double tilt_pivot_drop = 0.045;
  double tool_length = 0.1992;
  double j3_min = -0.33;
  double j3_max = 0.0;
  bool elbow_right = true;       // prefer J2 >= 0 ("righty")
  double j1_min = -1.0821, j1_max = 4.2237;
  double j2_min = -2.57, j2_max = 2.57;
  double tilt_min = -0.7854, tilt_max = 0.7854;
};

using Joints = std::array<double, 5>;  // j1, j2, j3, j4, tilt

struct TcpPose {
  double x = 0, y = 0, z = 0;  // cup-face centre in base_link
  double yaw = 0;              // heading of the tool x axis (tilt axis) about world z
  double tilt = 0;             // J5 angle: rotation of the tool about its x axis (0 = vertical)
};

inline double wrapToPi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

// Unwrap `target` to the representative closest to `reference`.
inline double unwrapNear(double target, double reference) {
  return reference + wrapToPi(target - reference);
}

inline TcpPose forward(const Joints& q, const KinematicParams& p = {}) {
  const double c1 = std::cos(q[0]), s1 = std::sin(q[0]);
  const double c12 = std::cos(q[0] + q[1]), s12 = std::sin(q[0] + q[1]);
  const double psi = q[0] + q[1] + q[3];
  const double theta = q[4];
  // pivot of the tilt axis
  const double px = p.l1 * c1 + p.l2 * c12;
  const double py = p.l1 * s1 + p.l2 * s12;
  const double pz = p.z_flange0 + q[2] - p.tilt_pivot_drop;
  // tool vector: R_z(psi) * R_x(theta) * (0,0,-L) = R_z(psi) * (0, L sin(theta), -L cos(theta))
  const double ty = p.tool_length * std::sin(theta);
  const double tz = -p.tool_length * std::cos(theta);
  TcpPose t;
  t.x = px - std::sin(psi) * ty;
  t.y = py + std::cos(psi) * ty;
  t.z = pz + tz;
  t.yaw = wrapToPi(psi);
  t.tilt = theta;
  return t;
}

enum class IkStatus { Ok, Unreachable, OutOfLimits };

struct IkResult {
  IkStatus status = IkStatus::Unreachable;
  Joints q{};
};

// Closed-form inverse kinematics. `seed` is used to pick the elbow configuration (when `elbow_from_seed`)
// and to unwrap J1/J4 continuously.
inline IkResult inverse(const TcpPose& t, const Joints& seed, const KinematicParams& p = {},
                        bool elbow_from_seed = true) {
  IkResult r;
  const double theta = t.tilt;
  if (theta < p.tilt_min || theta > p.tilt_max) { r.status = IkStatus::OutOfLimits; return r; }
  // pivot = tcp - R_z(psi) R_x(theta) (0,0,-L)
  const double ty = p.tool_length * std::sin(theta);
  const double tz = -p.tool_length * std::cos(theta);
  const double px = t.x + std::sin(t.yaw) * ty;
  const double py = t.y - std::cos(t.yaw) * ty;
  const double pz = t.z - tz;
  // planar 2R
  const double r2 = px * px + py * py;
  const double c2 = (r2 - p.l1 * p.l1 - p.l2 * p.l2) / (2.0 * p.l1 * p.l2);
  if (c2 > 1.0 + 1e-9 || c2 < -1.0 - 1e-9) { r.status = IkStatus::Unreachable; return r; }
  const double c2c = std::max(-1.0, std::min(1.0, c2));
  if (c2c > 0.995) { r.status = IkStatus::Unreachable; return r; }  // near full extension: singular
  const double s2mag = std::sqrt(std::max(0.0, 1.0 - c2c * c2c));
  bool right = p.elbow_right;
  if (elbow_from_seed && std::fabs(seed[1]) > 1e-6) right = seed[1] >= 0.0;
  const double s2 = right ? s2mag : -s2mag;
  const double j2 = std::atan2(s2, c2c);
  const double j1 = std::atan2(py, px) - std::atan2(p.l2 * s2, p.l1 + p.l2 * c2c);
  const double j3 = pz + p.tilt_pivot_drop - p.z_flange0;
  const double j4 = t.yaw - j1 - j2;

  Joints q;
  q[0] = unwrapNear(j1, seed[0]);
  q[1] = j2;
  q[2] = j3;
  q[3] = unwrapNear(j4, seed[3]);
  q[4] = theta;
  // J1 range is asymmetric (-62°..242°): if the unwrapped value is outside, try the other representative
  if (q[0] < p.j1_min || q[0] > p.j1_max) {
    const double alt = q[0] + (q[0] < p.j1_min ? 2.0 * M_PI : -2.0 * M_PI);
    if (alt >= p.j1_min && alt <= p.j1_max) { q[3] -= (alt - q[0]); q[0] = alt; }
  }
  r.q = q;
  if (q[0] < p.j1_min || q[0] > p.j1_max || q[1] < p.j2_min || q[1] > p.j2_max ||
      q[2] < p.j3_min - 1e-9 || q[2] > p.j3_max + 1e-9) {
    r.status = IkStatus::OutOfLimits;
    return r;
  }
  r.status = IkStatus::Ok;
  return r;
}

// Reachable annulus of the tilt pivot in the base xy-plane.
inline bool inWorkspaceXY(double x, double y, const KinematicParams& p = {}) {
  const double r = std::hypot(x, y);
  const double rmax = std::sqrt(p.l1 * p.l1 + p.l2 * p.l2 + 2.0 * p.l1 * p.l2 * 0.995);
  const double rmin = std::fabs(p.l1 - p.l2) + 0.02;
  return r > rmin && r < rmax;
}

}  // namespace togsim
