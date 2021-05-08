#include "PriorFactorPose3Z.h"

namespace gtsam_auv {

	gtsam::Vector PriorFactorPose3Z::evaluateError(const gtsam::Pose3& p,
												   boost::optional<gtsam::Matrix&> H) const {
		// note that use boost optional like a pointer
		// only calculate jacobian matrix when non-null pointer exists
		if (H) *H = (gtsam::Matrix16() << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();
		
		// return error vector
		return (gtsam::Vector1() << p.z() - measZ_).finished();
	}
	
} // namespace gtsam_auv