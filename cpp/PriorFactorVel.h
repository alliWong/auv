/**
 * @file PriorFactorVel.h
 * A prior factor for linear velocity measured from robot body frame.
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>


namespace gtsam_auv {

  class PriorFactorVel: public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {

  private:
    // measurement
    gtsam::Vector3 b_velocity_;

    // shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<PriorFactorVel> shared_ptr;

  public:

    /**
     * Constructor
     * @param poseKey     associated pose variable key
     * @param velKey      associated velocity variable key
     * @param model       noise model for sensor
     * @param b_velocity  body frame velocity measurement
    **/
    PriorFactorVel(const gtsam::Key &poseKey, const gtsam::Key &velKey, const gtsam::Vector3 &b_velocity, const gtsam::SharedNoiseModel &model) :
        gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>(model, poseKey, velKey), b_velocity_(b_velocity) {};

    // Error function
    // @param p    the pose in Pose3
    // @param v    the velocity in Vector
    // @param H    the optional Jacobian matrix, which use boost optional and has default null pointer
    gtsam::Vector evaluateError(const gtsam::Pose3& p,
                                const gtsam::Vector3& v, 
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none) const;

    // Also need to override a second method. According to Frank Dellaert:
    // "The second is a 'clone' function that allows the factor to be copied. Under most
    // circumstances, the following code that employs the default copy constructor should
    // work fine."
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new PriorFactorVel(*this)));
    }
    virtual ~PriorFactorVel() = default; // trivial deconstructor  
  }; // class

} // namespace gtsam