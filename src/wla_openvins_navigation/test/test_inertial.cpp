#include <cassert>
#include <iostream>
#include "core/VioManager.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "utils/print.h"
#include "wla_openvins_navigation/adapter.hpp"
#include "wla_openvins_navigation/visual_evidence.hpp"

ov_msckf::VioManagerOptions config(const std::string &path) {
  auto p=std::make_shared<ov_core::YamlParser>(path);ov_msckf::VioManagerOptions o;
  o.print_and_load(p);assert(p->successful());o.use_multi_threading_subs=false;o.use_multi_threading_pubs=false;
  o.num_opencv_threads=1;return o;
}
void propagation(ov_msckf::VioManagerOptions o,bool black) {
  ov_msckf::VioManager vio(o);
  Eigen::Matrix<double,17,1> init=Eigen::Matrix<double,17,1>::Zero();
  const double start=100,offset=o.calib_camimu_dt;init(0)=start-offset;init(4)=1;
  vio.initialize_with_gt(init);
  const Eigen::Vector3d acc(.03,-.02,.01);
  double initial_cov=-1,final_cov=0,max_error=0;int outputs=0;
  for(int n=-2;n<=4000;n++) {
    double elapsed=n*.005,t=start+elapsed;
    ov_core::ImuData imu;imu.timestamp=t;imu.wm.setZero();imu.am=acc+Eigen::Vector3d(0,0,o.gravity_mag);
    vio.feed_measurement_imu(imu);
    if(n<1)continue;
    if(black&&n%10==0) {
      ov_core::CameraData c;c.timestamp=t-offset-.01;c.sensor_ids={0,1};
      if(c.timestamp>vio.get_state()->_timestamp) {
        for(int k=0;k<2;k++){c.images.push_back(cv::Mat::zeros(480,640,CV_8UC1));c.masks.push_back(cv::Mat::zeros(480,640,CV_8UC1));}
        vio.feed_measurement_camera(c);
      }
    }
    if(t-offset-.2-vio.get_state()->_timestamp>.1)vio.propagate_without_camera(t-offset-.2);
    wla_openvins::State x;wla_openvins::Covariance p;
    assert(vio.get_propagator()->fast_state_propagate(vio.get_state(),t,x,p));
    ++outputs;assert(x.allFinite()&&p.allFinite());
    max_error=std::max(max_error,(x.segment<3>(4)-.5*acc*elapsed*elapsed).norm());
    if(initial_cov<0)initial_cov=p.block<3,3>(3,3).trace();final_cov=p.block<3,3>(3,3).trace();
    assert(vio.get_state()->_clones_IMU.size()<=size_t(o.state_options.max_clone_size));
  }
  assert(max_error<1e-5);assert(final_cov>initial_cov);assert(vio.msckf_update_sequence()==0);
  assert(outputs==4000);
  assert(vio.get_propagator()->buffered_imu_count()<400);
  std::cout<<"PASS "<<(black?"black_images":"camera_stop")<<" 20s/4000 propagated states, offset="<<offset
    <<", max XYZ numerical error="<<max_error<<", covariance="<<initial_cov<<" -> "<<final_cov
    <<", buffered IMU="<<vio.get_propagator()->buffered_imu_count()<<"\n";
}
void recovery(ov_msckf::VioManagerOptions o,bool drop) {
  ov_msckf::VioManager vio(o);const double start=200,offset=o.calib_camimu_dt;
  Eigen::Matrix<double,17,1> init=Eigen::Matrix<double,17,1>::Zero();init(0)=start-offset;init(4)=1;init(8)=.1;
  vio.initialize_with_gt(init);size_t before=0,after_gap=0;
  for(int n=-2;n<=4400;n++) {
    double t=start+n*.005;
    ov_core::ImuData imu;imu.timestamp=t;imu.wm.setZero();imu.am<<0,0,o.gravity_mag;vio.feed_measurement_imu(imu);
    if(n<1)continue;
    double elapsed=n*.005;
    if(n%10==0) {
      const double tc=t-offset-.01,ti=tc+offset;const bool lost=elapsed>=5&&elapsed<15;
      if(!lost||!drop) {
        std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> features(2);
        if(!lost)for(int k=0;k<2;k++)for(int j=0;j<120;j++) {
          Eigen::Vector3d point((j%12-5)*.25,(j/12-4)*.25,4.+(j%3)*.5);
          point-=Eigen::Vector3d(.1*(ti-start)+k*.095,0,0);
          Eigen::VectorXf uv(2);uv<<400*point.x()/point.z()+320,400*point.y()/point.z()+240;
          if(uv.x()>0&&uv.x()<640&&uv.y()>0&&uv.y()<480)features[k].emplace_back(j,uv);
        }
        if(tc>vio.get_state()->_timestamp)vio.feed_measurement_simulation(tc,{0,1},features);
      }
    }
    if(t-offset-.2-vio.get_state()->_timestamp>.1)vio.propagate_without_camera(t-offset-.2);
    wla_openvins::State x;wla_openvins::Covariance p;
    assert(vio.get_propagator()->fast_state_propagate(vio.get_state(),t,x,p));assert(x.allFinite()&&p.allFinite());
    if(n==999)before=vio.msckf_update_sequence();if(n==2999)after_gap=vio.msckf_update_sequence();
  }
  assert(before>0);assert(vio.msckf_update_sequence()>after_gap);
  std::cout<<"PASS simulated-feature recovery after 10s "<<(drop?"camera stop":"empty visual constraints")
    <<", update sequences "<<before<<" / "<<after_gap<<" / "<<vio.msckf_update_sequence()<<"\n";
}
void sparse_corrections(ov_msckf::VioManagerOptions o,bool slow_camera) {
  ov_msckf::VioManager vio(o);const double start=200,offset=o.calib_camimu_dt;
  Eigen::Matrix<double,17,1> init=Eigen::Matrix<double,17,1>::Zero();
  init(0)=start-offset;init(4)=1;init(8)=.1;vio.initialize_with_gt(init);
  double last=-1;unsigned updates=0,sparse=0,tracking=0;bool saw_loss=false,saw_recovery=false;
  // At 30 Hz the shared track lifetime fits the existing 0.5 s visual timeout.
  // At 20 Hz these deliberately identical lifetimes produce ~0.55 s gaps;
  // that is a negative timing control, even though every correction is valid.
  for(int n=-2;n<=2160;n++) {
    const double t=start+n/240.,elapsed=n/240.;
    ov_core::ImuData imu;imu.timestamp=t;imu.wm.setZero();imu.am<<0,0,o.gravity_mag;
    vio.feed_measurement_imu(imu);
    if(n<1)continue;
    if(n%(slow_camera?12:8)==0) {
      const double tc=t-offset-.01,ti=tc+offset;
      std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> features(2);
      if(elapsed<3. || elapsed>=5.)for(int k=0;k<2;k++)for(int j=0;j<7;j++) {
        Eigen::Vector3d point((j%4-1.5)*.5,(j/4-.5)*.5,4.+(j%3)*.5);
        point-=Eigen::Vector3d(.1*(ti-start)+k*.095,0,0);
        Eigen::VectorXf uv(2);uv<<400*point.x()/point.z()+320,400*point.y()/point.z()+240;
        features[k].emplace_back(j,uv);
      }
      const auto before=vio.msckf_update_sequence();
      vio.feed_measurement_simulation(tc,{0,1},features);
      const auto after=vio.msckf_update_sequence();
      const unsigned accepted=after>before?vio.last_msckf_update_count():0;
      const bool corrected=wla_openvins::record_visual_correction(before,after,accepted,ti,.5,last,updates);
      if(corrected) {assert(accepted>0 && accepted<8);++sparse;}
      const auto mode=wla_openvins::mode(false,true,true,last<0?-1:t-last,.5,true,updates,3);
      if(mode==wla_openvins::VIO_TRACKING)++tracking;
      if(elapsed>=4. && elapsed<5.) {assert(mode==wla_openvins::INERTIAL_ONLY);saw_loss=true;}
      if(elapsed>5. && mode==wla_openvins::VIO_TRACKING)saw_recovery=true;
    }
  }
  std::cerr<<"camera_hz="<<(slow_camera?20:30)<<" sparse="<<sparse<<" tracking="<<tracking<<" saw_loss="<<saw_loss
    <<" saw_recovery="<<saw_recovery<<" last="<<last<<" updates="<<updates<<"\n";
  assert(sparse>=6 && saw_loss);
  if(slow_camera) {assert(tracking==0 && !saw_recovery);}
  else {assert(tracking>0 && saw_recovery);}
  std::cout<<"PASS actual MSCKF sparse corrections (<8 features): "<<sparse
    <<", tracking frames="<<tracking<<", camera_hz="<<(slow_camera?20:30)<<"\n";
}
int main(int argc,char**argv) {
  assert(argc==2 || argc==3);ov_core::Printer::setPrintLevel("ERROR");auto o=config(argv[1]);
  if(argc==3) {assert(std::string(argv[2])=="--sparse-updates");sparse_corrections(o,false);sparse_corrections(o,true);return 0;}
  for(double offset:{0.,.007,-.007}) {o.calib_camimu_dt=offset;propagation(o,false);}
  propagation(o,true);recovery(o,false);recovery(o,true);
}
