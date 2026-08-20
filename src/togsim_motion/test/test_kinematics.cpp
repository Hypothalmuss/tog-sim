#include <gtest/gtest.h>
#include <random>
#include "togsim_motion/scara5_kinematics.hpp"

using namespace togsim;

TEST(Kinematics, HomePoseMatchesUrdfConstants) {
  KinematicParams p;
  Joints q{0, 0, 0, 0, 0};
  auto t = forward(q, p);
  EXPECT_NEAR(t.x, p.l1 + p.l2, 1e-12);
  EXPECT_NEAR(t.y, 0.0, 1e-12);
  EXPECT_NEAR(t.z, p.z_flange0 - p.tilt_pivot_drop - p.tool_length, 1e-12);
  EXPECT_NEAR(t.yaw, 0.0, 1e-12);
}

TEST(Kinematics, TiltMovesCupSideways) {
  KinematicParams p;
  Joints q{0, 0, 0, 0, 0.5};
  auto t = forward(q, p);
  // tilting about the tool x axis (= world x at yaw 0) swings the cup along +y and raises it
  EXPECT_NEAR(t.y, p.tool_length * std::sin(0.5), 1e-12);
  EXPECT_GT(t.z, forward(Joints{0, 0, 0, 0, 0}, p).z);
}

TEST(Kinematics, ForwardInverseRoundTrip) {
  KinematicParams p;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> u1(p.j1_min, p.j1_max), u2(p.j2_min, p.j2_max), u3(p.j3_min, p.j3_max),
      u4(-3.0, 3.0), u5(p.tilt_min, p.tilt_max);
  int ok = 0, tested = 0;
  for (int i = 0; i < 10000; ++i) {
    Joints q{u1(rng), u2(rng), u3(rng), u4(rng), u5(rng)};
    if (std::fabs(std::cos(q[1])) > 0.995) continue;  // skip singular samples (near full extension / folded)
    ++tested;
    auto t = forward(q, p);
    auto r = inverse(t, q, p, true);
    ASSERT_EQ(r.status, IkStatus::Ok) << "sample " << i;
    for (int k = 0; k < 5; ++k) EXPECT_NEAR(r.q[k], q[k], 1e-7) << "joint " << k << " sample " << i;
    auto t2 = forward(r.q, p);
    EXPECT_NEAR(t2.x, t.x, 1e-9);
    EXPECT_NEAR(t2.y, t.y, 1e-9);
    EXPECT_NEAR(t2.z, t.z, 1e-9);
    ++ok;
  }
  EXPECT_GT(tested, 9000);
  EXPECT_EQ(ok, tested);
}

TEST(Kinematics, ElbowSelectionFollowsSeed) {
  KinematicParams p;
  TcpPose t;
  t.x = 0.45; t.y = 0.1; t.z = 0.0; t.yaw = 0.3; t.tilt = 0.0;
  auto right = inverse(t, Joints{0, 0.5, -0.1, 0, 0}, p, true);
  auto left = inverse(t, Joints{0, -0.5, -0.1, 0, 0}, p, true);
  ASSERT_EQ(right.status, IkStatus::Ok);
  ASSERT_EQ(left.status, IkStatus::Ok);
  EXPECT_GT(right.q[1], 0.0);
  EXPECT_LT(left.q[1], 0.0);
  auto tr = forward(right.q, p), tl = forward(left.q, p);
  EXPECT_NEAR(tr.x, tl.x, 1e-9);
  EXPECT_NEAR(tr.y, tl.y, 1e-9);
}

TEST(Kinematics, UnreachableAndLimits) {
  KinematicParams p;
  TcpPose far; far.x = 0.9; far.z = 0.0;
  EXPECT_EQ(inverse(far, Joints{}, p).status, IkStatus::Unreachable);
  TcpPose tooLow; tooLow.x = 0.45; tooLow.z = -2.0;
  EXPECT_EQ(inverse(tooLow, Joints{}, p).status, IkStatus::OutOfLimits);
  TcpPose tooTilted; tooTilted.x = 0.45; tooTilted.tilt = 1.2;
  EXPECT_EQ(inverse(tooTilted, Joints{}, p).status, IkStatus::OutOfLimits);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
