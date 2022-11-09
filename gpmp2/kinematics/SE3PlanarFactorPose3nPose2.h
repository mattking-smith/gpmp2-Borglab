/**
 *  @file   SE3PlanarFactorPose3nPose2.h
 *  @brief  Factor to constrain SE(3) state to SE(2) Planar Motion
 *  @author Matthew King-Smith
 *  @date   Oct 27, 2022
 **/

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <vector>

namespace gpmp2 {

/**
 * unary factor for constraining Pose3 to only have planar motion
 */
class SE3PlanarFactorPose3nPose2
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose2> {
private:
  // typedefs
  typedef SE3PlanarFactorPose3nPose2 This;
  typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose2> Base;

public:
  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /* Default constructor */
  SE3PlanarFactorPose3nPose2() {}

  /**
   * Constructor
   * @param cost_sigma cost function covariance
   */
  SE3PlanarFactorPose3nPose2(gtsam::Key pose3Key, gtsam::Key pose2Key,
                             double cost_sigma)
      : Base(gtsam::noiseModel::Isotropic::Sigma(3, cost_sigma), pose3Key,
             pose2Key) {}

  virtual ~SE3PlanarFactorPose3nPose2() {}

  /// error function
  /// numerical/analytic Jacobians from cost function
  gtsam::Vector
  evaluateError(const gtsam::Pose3 &pose3D, const gtsam::Pose2 &pose2D,
                boost::optional<gtsam::Matrix &> H1 = boost::none,
                boost::optional<gtsam::Matrix &> H2 = boost::none) const {
    using namespace gtsam;
    const Rot2& R1 = pose2D.rotation();
    double c_psi = cos(pose3D.rotation().yaw());
    double s_psi = sin(pose3D.rotation().yaw());

    /* [0 0 0 1 0 0
        0 0 0 0 1 0
        0 0 1 0 0 0]*/

    if (H1) { 
      *H1 = (Matrix(3, 6) << 0, 0, 0, c_psi, -s_psi, 0,//
                              0, 0, 0, s_psi, c_psi, 0,//
                              0, 0, 1, 0, 0, 0)
                .finished();
    }
    if (H2) {
      *H2 = (Matrix(3, 3) << -R1.c(), R1.s(), 0.0,//
                              -R1.s(), -R1.c(), 0.0,//
                              0.0,  0.0, -1.0)
                .finished();
    }
    Vector pose2vec = Vector(3);
    pose2vec(0) = pose2D.x();
    pose2vec(1) = pose2D.y();
    pose2vec(2) = pose2D.theta();

     
    Vector pose3planarvec = Vector(3);
    pose3planarvec(0) = pose3D.x();
    pose3planarvec(1) = pose3D.y();
    pose3planarvec(2) = pose3D.rotation().yaw();
    
    return (Vector(3) << pose3planarvec - pose2vec).finished();
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "SE3PlanarFactorPose3nPose2 :" << std::endl;
    Base::print("", keyFormatter);
  }

  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
  }
};

} // namespace gpmp2
