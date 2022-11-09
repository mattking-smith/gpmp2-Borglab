/**
 *  @file testSE3PlanarFactorPose3nPose2.cpp
 *  @author Matt King-Smith
 **/

#include <CppUnitLite/TestHarness.h>
#include <gpmp2/kinematics/SE3PlanarFactorPose3nPose2.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;

/* ************************************************************************** */
TEST(SE3PlanarFactorPose3nPose2, Factor) {
  Key Pose2Key = Symbol('x', 1), Pose3Key = Symbol('X', 1);
  const double cost_sigma = 0.1;
  SE3PlanarFactorPose3nPose2 factor(Pose3Key, Pose2Key, cost_sigma);
  Pose2 p2D;
  Pose3 p3D;
  Matrix actualH1, actualH2;
  Matrix expectH1, expectH2;
  Vector actual, expect;

  
  p2D = Pose2(5, 7, 3);
  p3D = Pose3(Pose2(5, 7, 3));

  expect = (Vector(3) << 0, 0, 0).finished();
  actual = factor.evaluateError(p3D, p2D, actualH1, actualH2);
  expectH1 = numericalDerivative11(
      std::function<Vector(const Pose3 &)>(std::bind(
          &SE3PlanarFactorPose3nPose2::evaluateError, factor, std::placeholders::_1, p2D, boost::none, boost::none)),
      p3D, 1e-6);
  expectH2 = numericalDerivative11(
      std::function<Vector(const Pose2 &)>(std::bind(
          &SE3PlanarFactorPose3nPose2::evaluateError, factor, p3D, std::placeholders::_1, boost::none, boost::none)),
      p2D, 1e-6);
  
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
}



/* ************************************************************************** */
// TEST(GaussianProcessPriorPose2, Optimization) {
//   /**
//    * A simple graph:
//    *
//    * p1   p2
//    * |    |
//    * x1   x2
//    *  \  /
//    *   gp
//    *  /  \
//    * v1  v2
//    *
//    * p1 and p2 are pose prior factor to fix the poses, gp is the GP factor
//    * that get correct velocity of v2
//    */

//   noiseModel::Isotropic::shared_ptr model_prior =
//       noiseModel::Isotropic::Sigma(3, 0.001);
//   double delta_t = 1;
//   Matrix Qc = 0.01 * Matrix::Identity(3, 3);
//   noiseModel::Gaussian::shared_ptr Qc_model =
//       noiseModel::Gaussian::Covariance(Qc);

//   Pose2 pose1(0, 0, 0), pose2(1, 0, 0);
//   Vector v1 = (Vector(3) << 1, 0, 0).finished();
//   Vector v2 = (Vector(3) << 2.0, -0.5, 0.6).finished();  // rnd value

//   NonlinearFactorGraph graph;
//   graph.add(PriorFactor<Pose2>(Symbol('x', 1), pose1, model_prior));
//   graph.add(PriorFactor<Pose2>(Symbol('x', 2), pose2, model_prior));
//   // graph.add(PriorFactor<Vector6>(Symbol('v', 1), v1, model_prior));
//   graph.add(GaussianProcessPriorPose2(Symbol('x', 1), Symbol('v', 1),
//                                       Symbol('x', 2), Symbol('v', 2),
//                                       delta_t, Qc_model));

//   Values init_values;
//   init_values.insert(Symbol('x', 1), pose1);
//   init_values.insert(Symbol('v', 1), v1);
//   init_values.insert(Symbol('x', 2), pose2);
//   init_values.insert(Symbol('v', 2), v2);

//   GaussNewtonParams parameters;
//   GaussNewtonOptimizer optimizer(graph, init_values, parameters);
//   optimizer.optimize();
//   Values values = optimizer.values();

//   EXPECT_DOUBLES_EQUAL(0, graph.error(values), 1e-6);
//   EXPECT(assert_equal(pose1, values.at<Pose2>(Symbol('x', 1)), 1e-6));
//   EXPECT(assert_equal(pose2, values.at<Pose2>(Symbol('x', 2)), 1e-6));
//   EXPECT(assert_equal(v1, values.at<Vector>(Symbol('v', 1)), 1e-6));
//   EXPECT(assert_equal(v1, values.at<Vector>(Symbol('v', 2)), 1e-6));
// }

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
