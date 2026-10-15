#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wla_openvins {
using State = Eigen::Matrix<double, 13, 1>;
using Covariance = Eigen::Matrix<double, 12, 12>;
inline Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d s; s << 0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0; return s;
}
struct BodyState {
  Eigen::Vector3d p, v, w;
  Eigen::Quaterniond q;
  Covariance covariance;  // [p_G, rotation_G, velocity_B, angular_velocity_B]
};
// T_IB maps base-frame coordinates to IMU-frame coordinates. JPL q_GI is
// numerically Hamilton q_IG. OpenVINS attitude perturbations are expressed in I.
inline BodyState adapt(const State &x, const Covariance &P, const Eigen::Isometry3d &T_IB) {
  if (!x.allFinite() || !P.allFinite()) throw std::runtime_error("nonfinite_estimate");
  Eigen::Quaterniond q_GI(x(3),x(0),x(1),x(2));
  if (std::abs(q_GI.norm()-1.0)>0.01) throw std::runtime_error("invalid_quaternion");
  q_GI.normalize();
  const Eigen::Matrix3d R_GI=q_GI.toRotationMatrix(), R_BI=T_IB.linear().transpose();
  const Eigen::Vector3d t=T_IB.translation();
  BodyState b;
  b.p=x.segment<3>(4)+R_GI*t;
  b.q=Eigen::Quaterniond(R_GI*T_IB.linear());
  b.v=R_BI*(x.segment<3>(7)+x.segment<3>(10).cross(t));
  b.w=R_BI*x.segment<3>(10);
  Covariance J=Covariance::Zero();
  J.block<3,3>(0,0)=-R_GI*skew(t); J.block<3,3>(0,3).setIdentity();
  J.block<3,3>(3,0)=R_GI;
  J.block<3,3>(6,6)=R_BI; J.block<3,3>(6,9)=-R_BI*skew(t);
  J.block<3,3>(9,9)=R_BI;
  b.covariance=J*P*J.transpose();
  b.covariance=(0.5*(b.covariance+b.covariance.transpose())).eval();
  return b;
}
}  // namespace wla_openvins
