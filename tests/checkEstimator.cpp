#define BOOST_TEST_MODULE check_eight_point_calibrator
#include <boost/test/included/unit_test.hpp>

#include <iostream>
#include <Eigen/Dense>

#include "vslam_utils/helperFunctions.h"
#include "EstimatorFixture.h"
#include "EightPointEstimator.h"

using namespace Eigen;

std::vector<Vector3f> appendZToImagePoints(const std::vector<Vector2f>& imagePoints, float z_comp)
{
  std::vector<Vector3f> points3D;
  std::transform(imagePoints.begin(), imagePoints.end(), std::back_inserter(points3D),
      [z_comp](Vector3f point) -> Vector3f {
        Vector3f new3Dpoint(point(0), point(1), z_comp);
        return new3Dpoint;
      }
  );
  return points3D;
}

BOOST_FIXTURE_TEST_CASE( cube1, EstimatorFixture )
{
  AffineTransform cameraPose1 = getCameraPose1();
  AffineTransform cameraPose2 = getCameraPose2(); 
  std::vector<Vector3f> worldPoints = getCubePoints();
  std::vector<Vector3f> camera1Points = vslam::transformPoints(cameraPose1.inverse(), worldPoints);
  std::vector<Vector3f> camera2Points = vslam::transformPoints(cameraPose2.inverse(), worldPoints);
  CameraIntrinsics intrinsics = getIntrinsics();
  std::vector<Vector2f> image1Points = vslam::getImagePoints(camera1Points, intrinsics.aspectRatio, intrinsics.f_x);
  std::vector<Vector2f> image2Points = vslam::getImagePoints(camera2Points, intrinsics.aspectRatio, intrinsics.f_x);
  std::vector<Vector3f> image1Points3D = appendZToImagePoints(image1Points, intrinsics.f_x);
  std::vector<Vector3f> image2Points3D = appendZToImagePoints(image1Points, intrinsics.f_x);
  AffineTransform estimatedPoseChange;
  eight_point_estimator::estimate(image1Points3D, image2Points3D, estimatedPoseChange);
  AffineTransform realPoseChange = cameraPose2.inverse() * cameraPose1;
  BOOST_TEST(vslam::posesClose(estimatedPoseChange, realPoseChange, 0.01*M_PI, 0.01));
} 
