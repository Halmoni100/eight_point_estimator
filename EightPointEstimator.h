#pragma once

#include <Eigen/Dense>
#include <vector>

#include "vslam_utils/vslamTypes.h"
#include "vslam_utils/CameraIntrinsics.h"

namespace eight_point_estimator {
  
  bool estimate(const std::vector<Eigen::Vector3f>& image1Pts,
                const std::vector<Eigen::Vector3f>& image2Pts,
                AffineTransform& resultPose);

} // namespace eight_point_estimator

