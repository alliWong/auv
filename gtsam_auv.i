class gtsam::Point2;
class gtsam::Pose2;
class gtsam::Vector3;

class gtsam::Point3;
class gtsam::Pose3;
class gtsam::Vector6;

class gtsam::Values;
virtual class gtsam::noiseModel::Base;
virtual class gtsam::NonlinearFactor;
virtual class gtsam::NonlinearFactorGraph;
virtual class gtsam::NoiseModelFactor : gtsam::NonlinearFactor;

// The namespace should be the same as in the c++ source code.
namespace gtsam_auv {

#include <cpp/PriorFactorPose3Z.h>
virtual class PriorFactorPose3Z : gtsam::NoiseModelFactor {
  PriorFactorPose3Z(size_t key, const double& measZ, gtsam::noiseModel::Base* noiseModel);
  Vector evaluateError(const gtsam::Pose3& p) const;
};

#include <cpp/PriorFactorVel.h>
virtual class PriorFactorVel : gtsam::NoiseModelFactor {
  PriorFactorVel(size_t key1, size_t key2, const Vector& b_velocity_, gtsam::noiseModel::Base* noiseModel);
  Vector evaluateError(const gtsam::Pose3& p, const Vector& v) const;
};


}  // namespace gtsam_auv
