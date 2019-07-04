#include "EstimatorFixture.h"
#include "vslam_utils/PointVisualizer.h"

int main()
{
  EstimatorFixture fixture;
  std::vector<Vector3f> points = fixture.getCubePoints();
  AffineTransform cameraPose1 = fixture.getCameraPose1();
  AffineTransform cameraPose2 = fixture.getCameraPose2();
  CameraIntrinsics intrinsics = fixture.getIntrinsics();

  PointVisualizer ptViz;
  ptViz.visualizePoints(points, cameraPose1, intrinsics);
  ptViz.visualizePoints(points, cameraPose2, intrinsics);
}
