/**
 *  @file   SignedDistanceFieldConverter.cpp
 *  @brief  Converts binary 2D and 3D volumes to signed distance fields
 *  @author Matthew King-Smith
 *  @date   Nov 16, 2025
 **/

#include <gpmp2/obstacle/DistanceTransform.h>

#include <cmath>
#include <limits>

namespace
{

Eigen::VectorXd computeEuclideanDistanceTransform(Eigen::VectorXd const & func_)
{
    size_t const dim = func_.size();
    size_t k = 0; ///< Index of right most parabola in lower envelope
    Eigen::VectorXi v(dim); ///< Locations of parabolas in lower envelope
    Eigen::VectorXd z(dim + 1); ///< Locations of boundaries between parabolas
    v[0] = 0;
    z[0] = -std::numeric_limits<double>::infinity();
    z[1] =  std::numeric_limits<double>::infinity();

    // Compute lower envelope
    double s; /// < Parabola intersection point
    for(size_t q = 1; q < dim; ++q)
    {

        while(true)
        {
            s = ((func_[q] + static_cast<double>(q*q)) - (func_[v[k]] + static_cast<double>(v[k]*v[k]))) / (2.0 * static_cast<double>(q - v[k]));

            if (s > z[k]) break;

            if (k == 0) break;
            k--;
        }
        ++k;
        v[k]   = q;
        z[k]   = s;
        z[k+1] = std::numeric_limits<double>::infinity();
    }

    k = 0;
    Eigen::VectorXd distanceTransform = Eigen::VectorXd::Zero(dim);
    for(size_t q = 0; q < dim; ++q)
    {
        while (z[k+1] < static_cast<double>(q)) ++k;
        distanceTransform[q] = static_cast<double>((q - v[k]) * (q - v[k])) + func_[v[k]];
    }
    return distanceTransform;
}

gtsam::Matrix computeEDTFromBinary(gtsam::Matrix const & binaryMap_)
{
    auto const rows = binaryMap_.rows();
    auto const cols = binaryMap_.cols();

    // Convert binary map to function map: 0 → free space, 1 → obstacle
    gtsam::Matrix funcMap_(rows,cols);
    funcMap_ = binaryMap_.unaryExpr([](double x)
    {
        return x == 1.0 ? 0.0 : std::numeric_limits<double>::max();
    });

    gtsam::Matrix edt(rows, cols);

    // --- Row-wise pass ---
    for(size_t r = 0; r < rows; ++r)
    {
        Eigen::VectorXd const f = funcMap_.row(r).transpose();
        edt.row(r)  = computeEuclideanDistanceTransform(f);
    }

    // --- Pass in y-direction (rows) ---
    for(size_t c = 0; c< cols; ++c)
    {
        Eigen::VectorXd const f = edt.col(c);
        edt.col(c)  = computeEuclideanDistanceTransform(f);
    }
    return edt;
};

} // namespace anonymous

gtsam::Matrix gpmp2::dt::roundMatrix(gtsam::Matrix const & mat_, int const decimals_)
{
    double scale = std::pow(10.0, decimals_);
    return (mat_ * scale).array().round() / scale;
}

gtsam::Matrix gpmp2::dt::computeSignedDistanceField(gtsam::Matrix const & binaryMap_, double const cellSize_)
{
    gtsam::Matrix const distToObject = computeEDTFromBinary(binaryMap_);
    gtsam::Matrix const distToObjectSqrt = distToObject.array().sqrt();

    gtsam::Matrix const invBinaryMap = gtsam::Matrix::Ones(binaryMap_.rows(), binaryMap_.cols()) - binaryMap_;
    gtsam::Matrix const distToBackground = computeEDTFromBinary(invBinaryMap);
    gtsam::Matrix const distToBackgroundSqrt = distToBackground.array().sqrt();

    return (distToObjectSqrt - distToBackgroundSqrt) * cellSize_;
}