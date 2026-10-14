#include <cassert>
#include <iostream>
#include "core/VioManager.h"
#include "state/State.h"
#include "state/Propagator.h"
#include "utils/print.h"
#include "wla_openvins_navigation/adapter.hpp"

void run(ov_msckf::VioManagerOptions o, double speed) {
  ov_msckf::VioManager vio(o);
  const double start=100, dt=o.calib_camimu_dt;
  const Eigen::Vector3d bg(.00064,.0022,.002), ba(.005,-.01,-.28);
  Eigen::Matrix<double,17,1> init=Eigen::Matrix<double,17,1>::Zero();
  init(0)=start-dt;init(4)=1;init(8)=speed;
  init.segment<3>(11)=bg+Eigen::Vector3d(.0002,0,0);
  init.segment<3>(14)=ba+Eigen::Vector3d(0,0,.005);
  vio.initialize_with_gt(init);
  size_t accepted_at_rest=0;int outputs=0;
  for(int n=-2;n<=2400;++n) {
    const double t=start+n*.005;
    const bool rotating=n>2000;
    ov_core::ImuData imu;imu.timestamp=t;
    imu.wm=bg+Eigen::Vector3d(rotating?1.0:0,0,0);
    imu.am=ba+Eigen::Vector3d(0,0,o.gravity_mag);
    vio.feed_measurement_imu(imu);
    if(n<=0)continue;
    if(n%10==0) {
      ov_core::CameraData c;c.timestamp=t-dt-.01;c.sensor_ids={0,1};
      for(int k=0;k<2;++k) {
        c.images.push_back(cv::Mat::zeros(480,640,CV_8UC1));
        c.masks.push_back(cv::Mat::zeros(480,640,CV_8UC1));
      }
      vio.feed_measurement_camera(c);
    }
    wla_openvins::State x;wla_openvins::Covariance P;
    assert(vio.get_propagator()->fast_state_propagate(vio.get_state(),t,x,P));
    assert(x.allFinite()&&P.allFinite());++outputs;
    if(n==2000) {
      accepted_at_rest=vio.zupt_update_sequence();
      if(speed==0) {
        assert(accepted_at_rest>150);
        assert(vio.initialized());
        assert((vio.get_state()->_imu->bias_g()-bg).norm()<.0002);
        assert(x.segment<3>(4).norm()<.01);
      } else assert(accepted_at_rest==0);
    }
  }
  assert(outputs==2400);
  assert(vio.msckf_update_sequence()==0);
  // One mixed transition interval is allowed; sustained rotation must not be held stationary.
  assert(vio.zupt_update_sequence()<=accepted_at_rest+1);
  std::cout<<"PASS ZUPT speed="<<speed<<", accepted while still="<<accepted_at_rest
           <<", final="<<vio.zupt_update_sequence()<<", outputs="<<outputs<<"\n";
}
int main(int argc,char**argv) {
  assert(argc==2);ov_core::Printer::setPrintLevel("ERROR");
  auto parser=std::make_shared<ov_core::YamlParser>(argv[1]);
  ov_msckf::VioManagerOptions o;o.print_and_load(parser);assert(parser->successful());
  o.use_multi_threading_subs=false;o.use_multi_threading_pubs=false;o.num_opencv_threads=1;
  o.try_zupt=true;o.zupt_only_at_beginning=false;
  o.zupt_options.chi2_multipler=1;o.zupt_noise_multiplier=10;
  o.zupt_max_velocity=.1;o.zupt_max_disparity=0;
  run(o,0);run(o,.2);
  assert(wla_openvins::mode(false,true,true,-1,.5,true,0,3,true)==wla_openvins::STATIONARY);
  assert(wla_openvins::mode(false,true,false,-1,.5,true,0,3,true)==wla_openvins::INVALID);
  assert(wla_openvins::mode(false,true,true,-1,.5,false,0,3,true)==wla_openvins::INERTIAL_ONLY);
}
