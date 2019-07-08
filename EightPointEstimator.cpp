#include <Eigen/Geometry>
#include <Eigen/Jacobi>
#include <iostream>

#include "vslam_utils/helperFunctions.h"
#include "EightPointEstimator.h"

namespace eight_point_estimator {

  bool meetsPositiveDepthConstraint(const AffineTransform& transform, const Vector3f& x1, const Vector3f& x2)
  {
    Matrix3f rot = transform.rotation();
    Vector3f trans = transform.translation();
    Vector3f x2_trans_parallel = vslam::getProjection(trans, x2);
    Vector3f x2_trans_orthogonal = x2 - x2_trans_parallel;
    Vector3f x1_adj = rot * x1;
    Vector3f x1_trans_parallel = vslam::getProjection(trans, x1_adj);
    Vector3f x1_trans_orthogonal = x1_adj - x1_trans_parallel;

    // Ensure that the orthogonal components are in the same direction
    // TODO: Resolve hardcoding
    float threshold = 0.01;
    float x1_orth_x2_orth_angle = vslam::getAngle(x1_trans_orthogonal, x2_trans_orthogonal);
    std::cout << "x1_orth_x2_orth_angle: " << x1_orth_x2_orth_angle << "\n";
    if ( x1_orth_x2_orth_angle > threshold )
     return false;

    // Ensure x1 and x2 are pointing toward each other
    float x1_angle = vslam::getAngle(trans, x1_adj);
    std::cout << "x1_angle: " << x1_angle << "\n";
    float x2_angle = vslam::getAngle(trans, x2);
    std::cout << "x2_angle: " << x2_angle << "\n";
    if ( x1_angle - x2_angle < 0 )
      return false;

    return true;
  }

  Vector3f recoverVectorFromCrossProductMatrix(Matrix3f mat)
  {
    return Vector3f(-mat(1,2), mat(0,2), -mat(0,1));
  }

  bool estimate(const std::vector<Eigen::Vector3f>& image1Pts,
                const std::vector<Eigen::Vector3f>& image2Pts,
                AffineTransform& resultPose)
  {
    // Make sure we have all correspondences for world and image points
    assert(image1Pts.size() == image2Pts.size());
    int numPoints = image1Pts.size();
    // Also make sure A matrix will have at least rank 8
    // This only checks number of points
    assert(numPoints >= 8);

    // Construct A matrix
    Matrix<float,Dynamic,9> A = Matrix<float,Dynamic,9>::Zero(numPoints,9);
    for (int i = 0; i < numPoints; i++) {
      float x1, x2, y1, y2, z1, z2;
      x1 = image1Pts[i](0); x2 = image2Pts[i](0);
      y1 = image1Pts[i](1); y2 = image2Pts[i](1);
      z1 = image1Pts[i](2); z2 = image2Pts[i](2);
      Matrix<float,1,9> A_row;
      A_row << x1*x2, x1*y2, x1*z2, y1*x2, y1*y2, y1*z2, z1*x2, z1*y2, z1*z2;
      A.block<1,9>(i,0) = A_row;
    }

    // Solve SVD of A
    JacobiSVD svdSolver_A(A, ComputeFullV);

    // Make sure A has at least rank 8
    // TODO Create a method to check the condition that makes sense

    // Recover calibration parameters from SVD of A
    Matrix<float,9,1> nonTrivialRightSingularVector = svdSolver_A.matrixV().block<9,1>(0,8);
    Matrix<float,9,1> E_s = nonTrivialRightSingularVector;

    // Recover the approxmation of E from above
    Matrix3f E_approx;
    E_approx << E_s(0), E_s(3), E_s(6), E_s(1), E_s(4), E_s(7), E_s(2), E_s(5), E_s(8);

    // Project E_approx to the essential space
    JacobiSVD svdSolver_E_approx(E_approx, ComputeFullU | ComputeFullV);
    // Find SVD of +-E_approx with U,V in SO3
    // Note that
    Matrix3f U = svdSolver_E_approx.matrixU();
    if (vslam::equal(U.determinant(), -1, 1e-6)) {
      U = -U;
    }
    Matrix3f V = svdSolver_E_approx.matrixV();
    if (vslam::equal(V.determinant(), -1, 1e-6)) {
      V = -V;
    }

    Matrix3f Sigma = DiagonalMatrix<float,3>(1,1,0);
    Matrix3f E = U * Sigma * V.transpose();

    // Calculate possible decompositions of Essential Matrix
    std::vector<AffineTransform> possibleTransforms;
    Matrix3f R_z_pi_div_2 = AngleAxisf(0.5*M_PI, Vector3f::UnitZ()).toRotationMatrix();

    Matrix3f R_1 = U * R_z_pi_div_2 * V.transpose();
    Matrix3f R_2 = U * R_z_pi_div_2 * V.transpose();
    Matrix3f T_hat_1 = U * R_z_pi_div_2 * Sigma * U.transpose();
    Vector3f T_1 = recoverVectorFromCrossProductMatrix(T_hat_1);
    Vector3f T_2 = -T_1;

    possibleTransforms.push_back(Translation3f(T_1) * R_1);
    possibleTransforms.push_back(Translation3f(T_2) * R_2);
    possibleTransforms.push_back(Translation3f(T_2) * R_1);
    possibleTransforms.push_back(Translation3f(T_1) * R_2);

    // Apply positive depth constraint
    for (const AffineTransform& transform: possibleTransforms) {
      if ( meetsPositiveDepthConstraint(transform, image1Pts[0], image2Pts[0]) ) {
        resultPose = transform;
        break;
      }
    }
    return true;
  }

} // namespace eight_point_estimator
