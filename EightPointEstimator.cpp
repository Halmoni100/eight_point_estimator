#include <Eigen/Geometry>
#include <Eigen/Jacobi>

#include "EightPointEstimator.h"

namespace eight_point_estimator {

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
    JacobiSVD svdSolver(A, ComputeFullV);

    // Make sure A has at least rank 8
    // TODO: remove hardcoded threshold
    svdSolver.setThreshold(0.01);
    if ( svdSolver.rank() < 8 )
      return false;
     
    // Recover calibration parameters from SVD of A
    Matrix<float,9,1> nonTrivialRightSingularVector = svdSolver.matrixV().block<9,1>(0,8);  
    Matrix<float,9,1> E_s = nonTrivialRightSingularVector;

    // Recover the approxmation of E from above
    Matrix3f E_approx;
    E_approx << E_s(0), E_s(3), E_s(6), E_s(1), E_s(4), E_s(7), E_s(2), E_s(5), E_s(8);
    
    // Project E_approx to the essential space
    svdSolver.compute(E_approx, ComputeFullU | ComputeFullV);
    Matrix3f U = svdSolver.matrixU();
    Matrix3f V = svdSolver.matrixV();
    if (U.determinant() != 1 || V.determinant() != 1)
    {
      svdSolver.compute(-E_approx, ComputeFullU | ComputeFullV);
      U = svdSolver.matrixU();
      V = svdSolver.matrixV();
    }
    Matrix3f E = U * DiagonalMatrix<float,3>(1,1,0) * V.transpose();

    // Calculate possible decompositions of Essential Matrix
    Matrix3f R_z_pi_div_2 = AngleAxisf(0.5*M_PI, Vector3f::UnitZ()).toRotationMatrix();
    Matrix3f R_1 = U * R_z_pi_div_2 * V.transpose();
    Matrix3f T_hat_1 = U * R_z_pi_div_2 * U.transpose();
    Matrix3f R_2 = U * R_z_pi_div_2 * V.transpose();
    Matrix3f T_hat_2 = U * -R_z_pi_div_2 * U.transpose();
    Matrix3f R_3 = U * -R_z_pi_div_2 * V.transpose();
    Matrix3f T_hat_3 = U * -R_z_pi_div_2 * U.transpose();
    Matrix3f R_4 = U * -R_z_pi_div_2 * V.transpose();
    Matrix3f T_hat_4 = U * R_z_pi_div_2 * U.transpose();
   
  }

} // namespace eight_point_estimator
