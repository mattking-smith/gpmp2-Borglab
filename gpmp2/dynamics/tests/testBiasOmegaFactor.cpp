/**
 *  @file testBiasOmegaFactor.cpp
 *  @author Matt King-Smith
 **/

#include <CppUnitLite/TestHarness.h>
#include <gpmp2/dynamics/BiasOmegaFactor.h>
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
TEST(BiasOmegaFactor, Factor) {
  Key biasKey = Symbol('b', 1), velKey = Symbol('v', 1);
  const double cost_sigma = 0.1;
  const double omega_z = 0.3;
  BiasOmegaFactor factor(biasKey, velKey, omega_z, cost_sigma);
  imuBias::ConstantBias bias = imuBias::ConstantBias(Vector3(),(Vector3()<< 0, 0, 0.2).finished());
  Vector3 vel = (Vector3() << 0, 0, 1).finished();
  Matrix actualH1, actualH2;
  Matrix expectH1, expectH2;
  Vector actual, expect;
  
  expect = (Vector(1) << omega_z - vel(2) - 0.2).finished();
  actual = factor.evaluateError(bias, vel, &actualH1, &actualH2);
  expectH1 = numericalDerivative11(
      std::function<Vector(const imuBias::ConstantBias &)>(std::bind(
          &BiasOmegaFactor::evaluateError, factor, std::placeholders::_1, vel, nullptr, nullptr)),
      bias, 1e-6);
  expectH2 = numericalDerivative11(
      std::function<Vector(const Vector3 &)>(std::bind(
          &BiasOmegaFactor::evaluateError, factor, bias, std::placeholders::_1, nullptr, nullptr)),
      vel, 1e-6);
  
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
