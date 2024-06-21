/**
 *  @file  SE3Est2LTIDyn.h
 *  @brief Planar Linear Time Invariant Dynamics with Control SE3 Estimates
 *  @date 05-08-2023
 *  @author Matthew King-Smith
 **/

#pragma once

#include <gpmp2/gp/GPutils.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gpmp2 {

/*
 * 6-way factor for converting SE3 estimates and IMU sensor measurements to
 * constrained SE2 dynamics for planning
 */
class SE3Est2LTIDyn
    : public gtsam::NoiseModelFactor6<
          gtsam::Pose3, gtsam::Vector, gtsam::imuBias::ConstantBias,
          gtsam::Pose2, gtsam::Vector, gtsam::Vector> {
private:
  size_t dof_;
  double delta_t_;
  double omega_z_;

  typedef SE3Est2LTIDyn This;
  typedef gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector,
                                   gtsam::imuBias::ConstantBias, gtsam::Pose2,
                                   gtsam::Vector, gtsam::Vector>
      Base;

public:
  SE3Est2LTIDyn() {} /* Default constructor only for serialization */

  /// Constructor
  /// @param delta_t is the time between the two states
  SE3Est2LTIDyn(gtsam::Key poseSE3Key, gtsam::Key velSE3Key, gtsam::Key biasKey,
                gtsam::Key poseSE2Key, gtsam::Key velSE2Key, gtsam::Key conKey,
                double omega_z_meas, double delta_t,
                const gtsam::SharedNoiseModel Qc_model)
      : Base(gtsam::noiseModel::Gaussian::Covariance(
                 calcQ(getQc(Qc_model), delta_t)),
             poseSE3Key, velSE3Key, biasKey, poseSE2Key, velSE2Key, conKey),
        dof_(Qc_model->dim()), delta_t_(delta_t), omega_z_(omega_z_meas) {}

  virtual ~SE3Est2LTIDyn() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// factor error function
  gtsam::Vector evaluateError(const gtsam::Pose3 &poseSE3,
                              const gtsam::Vector &velSE3,
                              const gtsam::imuBias::ConstantBias &bias,
                              const gtsam::Pose2 &poseSE2,
                              const gtsam::Vector &velSE2,
                              const gtsam::Vector &con,
                              gtsam::OptionalMatrixType H1 = nullptr,
                              gtsam::OptionalMatrixType H2 = nullptr,
                              gtsam::OptionalMatrixType H3 = nullptr,
                              gtsam::OptionalMatrixType H4 = nullptr,
                              gtsam::OptionalMatrixType H5 = nullptr,
                              gtsam::OptionalMatrixType H6 = nullptr) const {

    const gtsam::Rot2 &R = poseSE2.rotation();

    gtsam::Vector GyroBias = (gtsam::Vector(3) << bias.gyroscope()).finished();

    // state vectors
    gtsam::Vector x1 = gtsam::Vector(2 * dof_);
    x1(0) = poseSE3.x();
    x1(1) = poseSE3.y();
    x1(2) = poseSE3.rotation().yaw();
    // linear x-, y- velocities
    x1(3) = velSE3(0);
    x1(4) = velSE3(1);
    // omega_z determined from rate gyro measurement and bias
    x1(5) = omega_z_ - GyroBias(2);

    gtsam::Vector x2 = gtsam::Vector(2 * dof_);
    x2(0) = poseSE2.x();
    x2(1) = poseSE2.y();
    x2(2) = poseSE2.theta();
    x2(3) = velSE2(0);
    x2(4) = velSE2(1);
    x2(5) = velSE2(2);

    // Control matrix
    gtsam::Matrix B =
        (gtsam::Matrix(2 * dof_, dof_) << gtsam::Matrix::Zero(dof_, dof_), //
         gtsam::Matrix::Identity(dof_, dof_))
            .finished();

    // ASTROS mass and inertia about the z-axis
    float m = 410.0, Izz = 28.5;

    // Matrix gain values for ASTROS (need to pass in variable in the future)
    B(3, 0) = 1.0 / m;
    B(4, 1) = 1.0 / m;
    B(5, 2) = 1.0 / Izz;

    // Matrix A
    gtsam::Matrix A =
        (gtsam::Matrix(2 * dof_, 2 * dof_) << gtsam::Matrix::Zero(dof_, dof_),
         gtsam::Matrix::Identity(dof_, dof_), //
         gtsam::Matrix::Zero(dof_, dof_), gtsam::Matrix::Zero(dof_, dof_))
            .finished();

    // Control Jacobian
    gtsam::Matrix J = (gtsam::Matrix(2 * dof_, dof_)
                           << gtsam::Matrix::Identity(dof_, dof_), //
                       gtsam::Matrix::Identity(dof_, dof_))
                          .finished();
    J(0, 0) = (delta_t_ * delta_t_) / (2.0 * m);
    J(1, 1) = (delta_t_ * delta_t_) / (2.0 * m);
    J(2, 2) = (delta_t_ * delta_t_) / (2.0 * Izz);
    J(3, 0) = delta_t_ / m;
    J(4, 1) = delta_t_ / m;
    J(5, 2) = delta_t_ / Izz;

    double c_psi = cos(poseSE3.rotation().yaw());
    double s_psi = sin(poseSE3.rotation().yaw());

    // Jacobians

    // Pose3 Jacobian
    if (H1)
      *H1 = (gtsam::Matrix(2 * dof_, 2 * dof_) << 
             0, 0, 0, c_psi, -s_psi, 0, //
             0, 0, 0, s_psi, c_psi, 0, //
             0, 0, 1, 0, 0, 0,
             gtsam::Matrix::Zero(dof_, 2 * dof_))
                .finished();

    // Linear SE3 velocities (x-, y-)
    if (H2)
      *H2 = (gtsam::Matrix(2 * dof_, dof_) << 
              delta_t_, 0,        0, //
              0,        delta_t_, 0, //
              0,        0,        0,//
              1,        0,        0,//
              0,        1,        0,//
              0,        0,        0).finished();

    // IMU bias Jacobian
    if (H3)
      *H3 = (gtsam::Matrix(2 * dof_, 2 * dof_) << 
              0, 0, 0, 0, 0, 0, //
              0, 0, 0, 0, 0, 0, //
              0, 0, 0, 0, 0, -delta_t_,//
              0, 0, 0, 0, 0, 0,//
              0, 0, 0, 0, 0, 0,//
              0, 0, 0, 0, 0, -1).finished();


    // Pose2 Jacobian
    if (H4)
      *H4 = (gtsam::Matrix(2 * dof_, dof_) << -R.c(), R.s(), 0.0, //
             -R.s(), -R.c(), 0.0,                                 //
             0.0, 0.0, -1.0,                                        //
             gtsam::Matrix::Zero(dof_, dof_))
                .finished();

    // SE2 Velocity (dot_x, dot_y, dot_psi) Jacobian
    if (H5)
      *H5 = (gtsam::Matrix(2 * dof_, dof_) << gtsam::Matrix::Zero(dof_, dof_),
             -1.0 * gtsam::Matrix::Identity(dof_, dof_))
                .finished();

    // Control jacobian
    if (H6)
      *H6 = J;

    // transition matrix & error
    return calcPhi(dof_, delta_t_) * x1 - x2 +
           calcPhi(dof_, delta_t_) *
               (gtsam::Matrix::Identity(2 * dof_, 2 * dof_) -
                0.5 * delta_t_ * A) *
               B * delta_t_ * con;
  }

  /** dimensions */
  size_t dim() const { return dof_; }

  /** number of variables attached to this factor */
  size_t size() const { return 6; }

  /** equals specialized to this factor */
  virtual bool equals(const gtsam::NonlinearFactor &expected,
                      double tol = 1e-9) const {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != NULL && Base::equals(*e, tol) &&
           fabs(this->delta_t_ - e->delta_t_) < tol;
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "6-way Estimation to Planning Factor(" << dof_ << ")"
              << std::endl;
    Base::print("", keyFormatter);
  }

private:
#ifdef GPMP2_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(dof_);
    ar &BOOST_SERIALIZATION_NVP(delta_t_);
  }
#endif
};

} // namespace gpmp2

/// traits
namespace gtsam {
template <>
struct traits<gpmp2::SE3Est2LTIDyn> : public Testable<gpmp2::SE3Est2LTIDyn> {};
} // namespace gtsam
