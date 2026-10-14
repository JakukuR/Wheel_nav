#include <cassert>
#include <iostream>
#include "wla_openvins_navigation/adapter.hpp"
#include "wla_openvins_navigation/visual_evidence.hpp"
using namespace wla_openvins;
int main() {
  State x=State::Zero();
  Eigen::Quaterniond q(Eigen::AngleAxisd(0.7,Eigen::Vector3d(1,2,3).normalized()));
  x.head<4>()=q.coeffs();x.segment<3>(4)<<2,3,4;x.segment<3>(7)<<0.5,-0.2,0.3;x.tail<3>()<<0.2,0.4,-0.6;
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.translation()<<0.3,-0.1,0.5;
  t.linear()=Eigen::AngleAxisd(-0.4,Eigen::Vector3d::UnitY()).toRotationMatrix();
  Covariance a=Covariance::Random(),p=a*a.transpose()+Covariance::Identity();
  auto b=adapt(x,p,t);Covariance numerical;
  for(int k=0;k<12;k++) {
    const double eps=1e-6;State y=x;
    if(k<3) {Eigen::Vector3d axis=Eigen::Vector3d::Zero();axis(k)=1;
      y.head<4>()=(q*Eigen::Quaterniond(Eigen::AngleAxisd(eps,axis))).coeffs();}
    else y(k+1)+=eps;
    auto c=adapt(y,p,t);Eigen::AngleAxisd angle(c.q*b.q.conjugate());
    numerical.block<3,1>(0,k)=(c.p-b.p)/eps;
    numerical.block<3,1>(3,k)=angle.axis()*angle.angle()/eps;
    numerical.block<3,1>(6,k)=(c.v-b.v)/eps;
    numerical.block<3,1>(9,k)=(c.w-b.w)/eps;
  }
  assert((b.covariance-numerical*p*numerical.transpose()).norm()<1e-4);
  assert((b.v-t.linear().transpose()*(x.segment<3>(7)+x.tail<3>().cross(t.translation()))).norm()<1e-12);
  assert(std::abs(b.p.z())>1.);
  assert(mode(false,true,true,10.,.5,true,100,3)==INERTIAL_ONLY);
  assert(mode(false,true,true,.1,.5,true,2,3)==RECOVERING);
  assert(mode(false,true,true,.1,.5,true,3,3)==VIO_TRACKING);
  assert(mode(true,true,true,.1,.5,true,100,3)==INVALID);
  assert(mode(false,false,false,-1,.5,false,0,3)==INITIALIZING);
  double last=-1;unsigned updates=0;
  assert(record_visual_correction(0,1,1,10.,.5,last,updates));
  assert(mode(false,true,true,0.,.5,true,updates,3)==RECOVERING);
  assert(record_visual_correction(1,2,4,10.1,.5,last,updates));
  assert(record_visual_correction(2,3,7,10.2,.5,last,updates));
  assert(mode(false,true,true,.1,.5,true,updates,3)==VIO_TRACKING);
  assert(!record_visual_correction(3,3,7,10.3,.5,last,updates)); // stale count
  assert(!record_visual_correction(3,4,0,10.3,.5,last,updates));
  assert(!record_visual_correction(3,4,1,10.2,.5,last,updates)); // duplicate frame
  assert(last==10.2 && updates==3);
  assert(mode(false,true,true,.6,.5,true,updates,3)==INERTIAL_ONLY);
  assert(record_visual_correction(3,4,1,11.,.5,last,updates));
  assert(updates==1 && mode(false,true,true,0.,.5,true,updates,3)==RECOVERING);
  assert(mode(false,true,true,0.,.5,false,3,3)==RECOVERING);
  std::cout<<"PASS nonzero lever arm, rotation, full covariance finite differences, XYZ and state policy\n";
  std::cout<<"PASS sparse corrections, stale evidence, visual loss, recovery and uncertainty\n";
}
