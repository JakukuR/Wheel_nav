// Constant-velocity target filter in a wheel-compensated local reference.
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <vector>
struct TargetKalman {
 using V=Eigen::Vector4d;using M=Eigen::Matrix4d;
 V state=V::Zero();M covariance=M::Identity();
 double stamp=-1,horizon=2.,max_std=.75;
 std::string reason="empty";
 explicit TargetKalman(double horizon_s=2.,double uncertainty_m=.75):horizon(horizon_s),max_std(uncertainty_m){
  if(!(horizon>0&&horizon<=2&&max_std>0&&max_std<=1.))throw std::invalid_argument("invalid_prediction_limits");
 }
 void reset(){stamp=-1;state.setZero();covariance.setIdentity();reason="empty";}
 void projected(double dt,V& x,M& p) const {
  M f=M::Identity();f(0,2)=dt;f(1,3)=dt;
  Eigen::Matrix<double,4,2> g;g<<dt*dt*.5,0,0,dt*dt*.5,dt,0,0,dt;
  x=f*state;p=f*covariance*f.transpose()+g*g.transpose();
 }
 bool update(double t,double x,double y){
  if(!std::isfinite(t)||!std::isfinite(x)||!std::isfinite(y)||t<=0){reason="invalid_measurement";return false;}
  if(stamp>=0&&t<=stamp){reason="replayed_measurement";return false;}
  constexpr double variance=.08*.08;
  if(stamp<0||t-stamp>horizon){state<<x,y,0,0;covariance=M::Zero();covariance.diagonal()<<variance,variance,1.,1.;stamp=t;reason="initialized";return true;}
  V predicted;M p;projected(t-stamp,predicted,p);
  Eigen::Matrix2d s=p.topLeftCorner<2,2>()+variance*Eigen::Matrix2d::Identity();
  Eigen::Vector2d innovation=Eigen::Vector2d(x,y)-predicted.head<2>();
  auto factor=s.ldlt();
  if(factor.info()!=Eigen::Success||innovation.dot(factor.solve(innovation))>9.21){reason="innovation_rejected";return false;}
  Eigen::Matrix<double,4,2> k=p.leftCols<2>()*factor.solve(Eigen::Matrix2d::Identity());
  V next=predicted+k*innovation;
  if(!next.allFinite()||next.tail<2>().norm()>3.){reason="target_speed_rejected";return false;}
  M correction=M::Identity();correction.leftCols<2>()-=k;
  covariance=correction*p*correction.transpose()+variance*k*k.transpose();
  state=next;stamp=t;reason="updated";return true;
 }
 std::vector<double> estimate(double t) const {
  double age=t-stamp;
  if(stamp<0||!std::isfinite(t)||age<0||age>horizon)return {};
  V x;M p;projected(age,x,p);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(p.topLeftCorner<2,2>());
  if(solver.info()!=Eigen::Success)return {};
  double sd=std::sqrt(std::max(0.,solver.eigenvalues().maxCoeff()));
  if(!x.allFinite()||!std::isfinite(sd)||sd>max_std)return {};
  return {x[0],x[1],x[2],x[3],age,sd};
 }
};
