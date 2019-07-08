#include <cstdlib>

#include "EstimatorFixture.h"

int global_rand_count = 0;

float EstimatorFixture::getRand()
{
  srand(global_rand_count);
  global_rand_count++;
  return (float) rand() / RAND_MAX;
}

Vector3f EstimatorFixture::getRandVec3()
{
  float x = getRand();
  float y = getRand();
  float z = getRand();
  return Vector3f(x,y,z);
}

std::vector<Vector3f> EstimatorFixture::getCubePoints()
{
  return mCommonFixture.getCubePoints1();
}

std::vector<Vector3f> EstimatorFixture::getRandomPoints()
{
  int numPoints = 10;
  std::vector<Vector3f> points;
  for (int i = 0; i < numPoints; i++)
    points.push_back(getRandVec3());
  return points;
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
