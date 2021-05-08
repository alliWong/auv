/**
 * @file PriorFactorPose3Z.h
 * A prior factor for linear position in the z-axis measurements in navigation/global frame.
*/

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>


namespace gtsam_auv {
  
  class PriorFactorPose3Z : public gtsam::NoiseModelFactor1<gtsam::Pose3> {

  private:
    // measurement
    double measZ_;
    
    // shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<PriorFactorPose3Z> shared_ptr;

  public:
    /**
     * Constructor
     * @param poseKey    associated pose varible key
     * @param model      noise model for sensor
     * @param measZ      double z measurement
     */
    PriorFactorPose3Z(const gtsam::Key &poseKey, const double &measZ, const gtsam::SharedNoiseModel &model) :
        gtsam::NoiseModelFactor1<gtsam::Pose3>(model, poseKey), measZ_(measZ) {};

    // error function
    // @param p    the pose in Pose3
    // @param H    the optional Jacobian matrix, which use boost optional and has default null pointer
    gtsam::Vector evaluateError(const gtsam::Pose3& p, 
                                boost::optional<gtsam::Matrix&> H = boost::none) const;

    // Also need to override a second method. According to Frank Dellaert:
    // "The second is a 'clone' function that allows the factor to be copied. Under most
    // circumstances, the following code that employs the default copy constructor should
    // work fine."
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new PriorFactorPose3Z(*this)));
    }
    virtual ~PriorFactorPose3Z() = default; // trivial deconstructor
  }; // class

} // namespace gtsam_auv
