/**
 *  @file  ObstaclePlanarSDFFactor.h
 *  @brief Obstacle avoidance cost factor, using planar signed distance field
 *  @author Jing Dong
 *  @date  Nov 15, 2015
 **/

#pragma once

#include <gpmp2/obstacle/PlanarSDF.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <vector>

namespace gpmp2 {

/**
 * unary factor for obstacle avoidance, planar version
 * template robot model version
 */
template <class ROBOT>
class ObstaclePlanarSDFFactor
    : public gtsam::NoiseModelFactorN<typename ROBOT::Pose> {
 public:
  // typedefs
  using Robot = ROBOT;
  using Pose = typename Robot::Pose;

 private:
  // typedefs
  typedef ObstaclePlanarSDFFactor This;
  typedef gtsam::NoiseModelFactorN<Pose> Base;

  // obstacle cost settings
  double epsilon_;  // distance from object that start non-zero cost

  // arm: planar one, all alpha = 0
  Robot robot_;

  // signed distance field from matlab
  PlanarSDF sdf_;

 public:
  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /** Default constructor */
  ObstaclePlanarSDFFactor() {}

  /**
   * Constructor
   * @param cost_model cost function covariance, should to identity model
   * @param field      signed distance field
   * @param nn_index   nearest neighbour index of signed distance field
   */
  ObstaclePlanarSDFFactor(gtsam::Key poseKey, const Robot& robot,
                          const PlanarSDF& sdf, double cost_sigma,
                          double epsilon)
      : Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(),
                                                 cost_sigma),
             poseKey),
        epsilon_(epsilon),
        robot_(robot),
        sdf_(sdf) {
    // TODO: check robot is planar
  }

  virtual ~ObstaclePlanarSDFFactor() {}

  /// error function
  /// numerical jacobians / analytic jacobians from cost function
  gtsam::Vector evaluateError(
      const Pose& conf, gtsam::OptionalMatrixType H1 = nullptr) const override;

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "ObstaclePlanarSDFFactor :" << std::endl;
    Base::print("", keyFormatter);
  }

#ifdef GPMP2_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int version) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
#endif
};

}  // namespace gpmp2

#include <gpmp2/obstacle/ObstaclePlanarSDFFactor-inl.h>
