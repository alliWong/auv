#include "PriorFactorVel.h"

namespace gtsam_auv {

	gtsam::Vector PriorFactorVel::evaluateError(const gtsam::Pose3& p,
                                              const gtsam::Vector3& v, 
                                              boost::optional<gtsam::Matrix&> H1,
                                              boost::optional<gtsam::Matrix&> H2) const {
    // ---------- Error Model ---------- //
    // Error = estimated_body_vel - measured_body_vel 
    // estimated_body_vel = inv(R[B->N]) * n_vel

    // ---------- Method 1 ---------- //
    gtsam::Matrix36 Hrot__pose; // drot/dx
    gtsam::Rot3 w_R_b = p.rotation(Hrot__pose).inverse(); // inv(R[B->N]) = R[N->B]
    gtsam::Matrix33 Hvel__rot; // dvel/drot
    gtsam::Vector3 vec_b = w_R_b.unrotate(v, Hvel__rot, H2); // transform world frame velocity into body frame
    if (H1) *H1 = Hvel__rot * Hrot__pose; // derr/dx

    std::cout << "\n*****VELOCITY FACTOR EVAL*****" << std::endl;
    std::cout << "VEL: \n" << b_velocity_ << std::endl;

    // return error vector
    return (gtsam::Vector3() << vec_b - b_velocity_).finished(); // return velocity error

    // gtsam::Matrix36 Hrot__pose; // drot/dx
    // gtsam::Rot3 w_R_b = p.rotation(Hrot__pose); // inv(R[B->N]) = R[N->B]
    // gtsam::Matrix33 Hvel__rot; // dvel/drot
    // gtsam::Vector3 vec_b = w_R_b.unrotate(v, Hvel__rot, H2); // transform world frame velocity into body frame
    // if (H1) *H1 = Hvel__rot * Hrot__pose; // derr/dx

    // // return error vector
    // return (gtsam::Vector3() << vec_b - b_velocity_).finished(); // return velocity error

    // ---------- Method 2 ---------- //
    // if(H1&&H2)
    // {
    //   gtsam::Rot3 w_R_b = p.rotation(H1);
    //   gtsam::Matrix3 Hvel__rot;
    //   gtsam::Vector3 hx = w_R_b.unrotate(v,Hvel__rot,H2);
    //   (*H1) = Hvel__rot* (*H1);
    //   return (hx-b_velocity_);
    // }
    // else
    // {
    //   gtsam::Rot3 w_R_b = p.rotation();
    //   gtsam::Vector3 hx = w_R_b.unrotate(v,boost::none,H2);
    //   return (hx-b_velocity_);
    // }
	}
	
} // namespace gtsam_auv