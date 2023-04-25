/**
 *  @file testPose2MobileVetLin2Arms.cpp
 *  @author Jing Dong
 *  @date Aug 22, 2016
 **/

#include <CppUnitLite/TestHarness.h>
#include <gpmp2/geometry/numericalDerivativeDynamic.h>
#include <gpmp2/kinematics/Pose2MobileVetLin2Arms.h>
#include <gpmp2/kinematics/mobileBaseUtils.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;

/* ************************************************************************** */
// fk wrapper
Pose3 fkpose(const Pose2MobileVetLin2Arms& r, const Pose2Vector& p, size_t i) {
  vector<Pose3> pos;
  r.forwardKinematics(p, {}, pos);
  return pos[i];
}

TEST(Pose2MobileVetLin2Arms, 2linkPlanarExamples) {
  // 2 link simple example, with none zero base poses
  // total dof = 3 + 1 + 2 + 2 = 8, Pose2 + Vector5
  Vector2 a(1, 1), alpha(0, 0), d(0, 0);
  Arm arm(2, a, alpha, d);
  Pose3 base1_pose(Rot3::Ypr(M_PI / 4.0, 0, 0), Point3(1.0, 0.0, 1.0));
  Pose3 base2_pose(Rot3::Ypr(3.0 * M_PI / 4.0, 0, 0), Point3(1.0, 0.0, 3.0));
  Pose2MobileVetLin2Arms marm(arm, arm, Pose3(), base1_pose, base2_pose, false);

  Pose2Vector q;
  // Vector5 qdot;
  // Vector qdymc;
  vector<Pose3> pvec_exp, pvec_act;
  // vector<Vector3> vvec_exp, vvec_act;
  vector<Matrix> pJp_exp, pJp_act;
  // vector<Matrix> vJp_exp, vJp_act, vJv_exp, vJv_act;

  // origin with zero poses
  q = Pose2Vector(Pose2(), (Vector5() << 0.0, 0.0, 0.0, 0.0, 0.0).finished());
  pvec_exp.clear();
  pvec_exp.push_back(Pose3());
  pvec_exp.push_back(Pose3());
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI / 4.0, 0, 0),
                           Point3(1.707106781186548, 0.707106781186548, 1.0)));
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI / 4.0, 0, 0),
                           Point3(2.414213562373095, 1.414213562373095, 1.0)));
  pvec_exp.push_back(Pose3(Rot3::Ypr(3.0 * M_PI / 4.0, 0, 0),
                           Point3(0.292893218813452, 0.707106781186548, 3.0)));
  pvec_exp.push_back(Pose3(Rot3::Ypr(3.0 * M_PI / 4.0, 0, 0),
                           Point3(-0.414213562373096, 1.414213562373095, 3.0)));
  pJp_exp.clear();
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(0))),
      q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(1))),
      q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(2))),
      q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(3))),
      q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(4))),
      q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(5))),
      q, 1e-6));
  marm.forwardKinematics(q, {}, pvec_act, nullptr, &pJp_act);
  EXPECT(assert_equal(pvec_exp[0], pvec_act[0], 1e-9));
  EXPECT(assert_equal(pvec_exp[1], pvec_act[1], 1e-9));
  EXPECT(assert_equal(pvec_exp[2], pvec_act[2], 1e-9));
  EXPECT(assert_equal(pvec_exp[3], pvec_act[3], 1e-9));
  EXPECT(assert_equal(pvec_exp[4], pvec_act[4], 1e-9));
  EXPECT(assert_equal(pvec_exp[5], pvec_act[5], 1e-9));
  EXPECT(assert_equal(pJp_exp[0], pJp_act[0], 1e-6));
  EXPECT(assert_equal(pJp_exp[1], pJp_act[1], 1e-6));
  EXPECT(assert_equal(pJp_exp[2], pJp_act[2], 1e-6));
  EXPECT(assert_equal(pJp_exp[3], pJp_act[3], 1e-6));
  EXPECT(assert_equal(pJp_exp[4], pJp_act[4], 1e-6));
  EXPECT(assert_equal(pJp_exp[5], pJp_act[5], 1e-6));

  // origin with non-zero poses
  q = Pose2Vector(
      Pose2(1.0, 0, M_PI / 4.0),
      (Vector5() << 1.1, M_PI / 4.0, M_PI / 4.0, M_PI / 4.0, M_PI / 4.0)
          .finished());
  pvec_exp.clear();
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI / 4.0, 0, 0), Point3(1.0, 0, 0)));
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI / 4.0, 0, 0), Point3(1.0, 0, 1.1)));
  pvec_exp.push_back(Pose3(Rot3::Ypr(3.0 * M_PI / 4.0, 0, 0),
                           Point3(1.0, 1.414213562373095, 2.1)));
  pvec_exp.push_back(Pose3(Rot3::Ypr(4.0 * M_PI / 4.0, 0, 0),
                           Point3(0.0, 1.414213562373095, 2.1)));
  pvec_exp.push_back(
      Pose3(Rot3::Ypr(5.0 * M_PI / 4.0, 0, 0), Point3(1.0, 0.0, 4.1)));
  pvec_exp.push_back(
      Pose3(Rot3::Ypr(6.0 * M_PI / 4.0, 0, 0), Point3(1.0, -1.0, 4.1)));
  pJp_exp.clear();
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(0))),
      q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(1))),
      q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(2))),
      q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(3))),
      q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(4))),
      q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(5))),
      q, 1e-6));
  marm.forwardKinematics(q, {}, pvec_act, nullptr, &pJp_act);
  EXPECT(assert_equal(pvec_exp[0], pvec_act[0], 1e-9));
  EXPECT(assert_equal(pvec_exp[1], pvec_act[1], 1e-9));
  EXPECT(assert_equal(pvec_exp[2], pvec_act[2], 1e-9));
  EXPECT(assert_equal(pvec_exp[3], pvec_act[3], 1e-9));
  EXPECT(assert_equal(pvec_exp[4], pvec_act[4], 1e-9));
  EXPECT(assert_equal(pvec_exp[5], pvec_act[5], 1e-9));
  EXPECT(assert_equal(pJp_exp[0], pJp_act[0], 1e-6));
  EXPECT(assert_equal(pJp_exp[1], pJp_act[1], 1e-6));
  EXPECT(assert_equal(pJp_exp[2], pJp_act[2], 1e-6));
  EXPECT(assert_equal(pJp_exp[3], pJp_act[3], 1e-6));
  EXPECT(assert_equal(pJp_exp[4], pJp_act[4], 1e-6));
  EXPECT(assert_equal(pJp_exp[5], pJp_act[5], 1e-6));

  // random to test jacobians

  // random base poses
  Pose3 torso_base = Pose3(Rot3::Ypr(1.3, -0.9, 1.1), Point3(2.3, -5.3, 8.0));
  base1_pose = Pose3(Rot3::Ypr(-3.3, -4.1, 6.5), Point3(-1.3, 2.2, -0.8));
  base2_pose = Pose3(Rot3::Ypr(2.4, -3.2, 9.1), Point3(3.2, 3.8, -1.2));
  marm = Pose2MobileVetLin2Arms(arm, arm, torso_base, base1_pose, base2_pose);

  q = Pose2Vector(Pose2(7.5, -2.8, 0.1),
                  (Vector5() << 0.9, -3.4, 5.3, 4.5, -9.9).finished());
  pJp_exp.clear();
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(0))),
      q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(1))),
      q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(2))),
      q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(3))),
      q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(4))),
      q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(
      std::function<Pose3(const Pose2Vector&)>(
          std::bind(&fkpose, marm, std::placeholders::_1, size_t(5))),
      q, 1e-6));
  marm.forwardKinematics(q, {}, pvec_act, nullptr, &pJp_act);
  EXPECT(assert_equal(pJp_exp[0], pJp_act[0], 1e-6));
  EXPECT(assert_equal(pJp_exp[1], pJp_act[1], 1e-6));
  EXPECT(assert_equal(pJp_exp[2], pJp_act[2], 1e-6));
  EXPECT(assert_equal(pJp_exp[3], pJp_act[3], 1e-6));
  EXPECT(assert_equal(pJp_exp[4], pJp_act[4], 1e-6));
  EXPECT(assert_equal(pJp_exp[5], pJp_act[5], 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
