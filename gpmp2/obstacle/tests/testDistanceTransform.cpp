/**
 *  @file   testDistanceTransform.cpp
 *  @brief  Test cases when converting obstacle map to signed distance field
 *  @author Matthew King-Smith
 *  @date   Apr 25, 2026
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Matrix.h>
#include <gpmp2/obstacle/DistanceTransform.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;

TEST(DistanceTransform, single_pixel) {
    Matrix singleObstacleMap {Matrix::Zero(3,3)};
    singleObstacleMap << 0, 0, 0,
                         0, 1, 0,
                         0, 0, 0;

    Matrix expectedSDF(3,3);
    expectedSDF << sqrt(2), 1, sqrt(2),
                   1,       -1, 1,
                   sqrt(2), 1, sqrt(2);

    const auto computedSDF = gpmp2::dt::computeSignedDistanceField(singleObstacleMap, 1.0);

    EXPECT(assert_equal(expectedSDF, computedSDF, 1e-6));
}

TEST(DistanceTransform, empty)
{
    Matrix const m = Matrix::Zero(3,3);

    auto const sdf = gpmp2::dt::computeSignedDistanceField(m, 1.0);

    // convention: large positive everywhere
    EXPECT((sdf.array() > 0).all());
}

TEST(DistanceTransform, full)
{
    Matrix const m = Matrix::Ones(3,3);

    auto const sdf = gpmp2::dt::computeSignedDistanceField(m, 1.0);

    // everything inside obstacle → negative
    EXPECT((sdf.array() <= 0).all());
}

TEST(DistanceTransform, horizontal_line)
{
    Matrix m = Matrix::Zero(5,5);
    m.row(2).setOnes();

    Matrix expected(5,5);
    expected <<
        2, 2, 2, 2, 2,
        1, 1, 1, 1, 1,
       -1,-1,-1,-1,-1,
        1, 1, 1, 1, 1,
        2, 2, 2, 2, 2;

    auto const sdf = gpmp2::dt::computeSignedDistanceField(m, 1.0);
    EXPECT(assert_equal(expected, sdf, 1e-6));
}

TEST(DistanceTransform, vertical_line)
{
    Matrix m = Matrix::Zero(5,5);
    m.col(2).setOnes();

    Matrix expected(5,5);
    expected <<
        2, 1,-1, 1, 2,
        2, 1,-1, 1, 2,
        2, 1,-1, 1, 2,
        2, 1,-1, 1, 2,
        2, 1,-1, 1, 2;

    auto const sdf = gpmp2::dt::computeSignedDistanceField(m, 1.0);
    EXPECT(assert_equal(expected, sdf, 1e-6));
}

TEST(DistanceTransform, square_block)
{
    Matrix m = Matrix::Zero(5,5);
    m.block(1,1,3,3).setOnes();

    Matrix expected(5,5);
    expected <<
        sqrt(2), 1, 1, 1, sqrt(2),
        1,      -1,-1,-1, 1,
        1,      -1,-2,-1, 1,
        1,      -1,-1,-1, 1,
        sqrt(2), 1, 1, 1, sqrt(2);

    auto const sdf = gpmp2::dt::computeSignedDistanceField(m, 1.0);
    EXPECT(assert_equal(expected, sdf, 1e-6));
}

TEST(DistanceTransform, diagonal)
{
    Matrix m = Matrix::Zero(3,3);
    m(0,0) = 1;
    m(1,1) = 1;
    m(2,2) = 1;

    auto const sdf = gpmp2::dt::computeSignedDistanceField(m, 1.0);

    // just check invariants (exact values messy)
    EXPECT((sdf.diagonal().array() < 0).all());
}

/* ************************************************************************** */
/* main function */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
