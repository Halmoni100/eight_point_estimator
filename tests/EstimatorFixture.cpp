#include "EstimatorFixture.h"

std::vector<Vector3f> EstimatorFixture::getCubePoints()
{
  return mCommonFixture.getCubePoints1();
}

AffineTransform EstimatorFixture::getCameraPose1()
{
  return mCommonFixture.getCameraPoseViewCenter(distance, altitude, azimuth1);
}

AffineTransform EstimatorFixture::getCameraPose2()
{
  return mCommonFixture.getCameraPoseViewCenter(distance, altitude, azimuth2);
}

CameraIntrinsics EstimatorFixture::getIntrinsics()
{
  return mCommonFixture.getStandardIntrinsics();
}
