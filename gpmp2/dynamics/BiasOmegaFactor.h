/**
 *  @file   BiasOmegaFactor.h
 *  @brief  Factor to take an omega measurement and correct with current bias
 *  @author Matthew King-Smith
 *  @date   Oct 27, 2022
 **/

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <vector>

namespace gpmp2 {

class BiasOmegaFactor
    : public gtsam::NoiseModelFactor2<gtsam::imuBias::ConstantBias,
                                      gtsam::Vector> {
private:
  double omega_z_;

  // typedefs
  typedef BiasOmegaFactor This;
  typedef gtsam::NoiseModelFactor2<gtsam::imuBias::ConstantBias, gtsam::Vector>
      Base;

public:
  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /* Default constructor */
  BiasOmegaFactor() {}

  /**
   * Constructor
   * @param cost_sigma cost function covariance
   */
  BiasOmegaFactor(gtsam::Key biasKey, gtsam::Key velKey, double omega_z,
                  double cost_sigma)
      : Base(gtsam::noiseModel::Isotropic::Sigma(1, cost_sigma), biasKey,
             velKey),
        omega_z_(omega_z) {}

  virtual ~BiasOmegaFactor() {}

  /// error function
  /// numerical/analytic Jacobians from cost function
  gtsam::Vector evaluateError(const gtsam::imuBias::ConstantBias &bias,
                              const gtsam::Vector &vel,
                              gtsam::OptionalMatrixType H1 = nullptr,
                              gtsam::OptionalMatrixType H2 = nullptr) const {
    using namespace gtsam;

    if (H1) {
      *H1 = (Matrix(1, 6) << 0, 0, 0, 0, 0, -1.0).finished();
    }
    if (H2) {
      *H2 = (Matrix(1, 3) << 0.0, 0.0, -1.0).finished();
    }

    Vector ratebias = (Vector(3) << bias.gyroscope()).finished();
    return (Vector(1) << -ratebias(2) + omega_z_ - vel(2)).finished();
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "BiasOmegaFactor :" << std::endl;
    Base::print("", keyFormatter);
  }

private:
#ifdef GPMP2_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
  }
#endif
};

} // namespace gpmp2
