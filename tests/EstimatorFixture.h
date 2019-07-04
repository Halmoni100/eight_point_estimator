#pragma once

#include "vslam_utils/vslamTypes.h"
#include "vslam_utils/CommonFixture.h"

struct EstimatorFixture
{
public:
  std::vector<Vector3f> getCubePoints();
  AffineTransform getCameraPose1();
  AffineTransform getCameraPose2();
  CameraIntrinsics getIntrinsics();
private:
  CommonFixture mCommonFixture;
  const float azimuthOffset = 0.25*M_PI/4;
  const float azimuthCenter = 0.25*M_PI;
  const float azimuth1 = azimuthCenter + azimuthOffset;
  const float azimuth2 = azimuthCenter - azimuthOffset;
  const float altitude = 0.25*M_PI;
  const float distance = 10.0;
};
