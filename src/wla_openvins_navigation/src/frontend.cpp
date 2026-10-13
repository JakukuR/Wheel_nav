#include <chrono>
#include <deque>
#include <limits>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "core/VioManager.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "utils/print.h"
#include "wla_openvins_navigation/adapter.hpp"
#include "wla_openvins_navigation/msg/estimator_status.hpp"

using namespace std::chrono_literals;
using Status=wla_openvins_navigation::msg::EstimatorStatus;
class Frontend final : public rclcpp::Node {
 public:
  Frontend():Node("openvins_frontend") {
    const auto config=declare_parameter<std::string>("config_path", "");
    if(config.empty()) throw std::invalid_argument("config_path is required; generate a calibrated config first");
    auto parser=std::make_shared<ov_core::YamlParser>(config);
    ov_core::Printer::setPrintLevel(declare_parameter<std::string>("estimator_verbosity","WARNING"));
    params_.print_and_load(parser);
    if(!parser->successful()) throw std::invalid_argument("OpenVINS configuration incomplete");
    // Fixed calibration + MSCKF-only makes update evidence and fast covariance explicit.
    if(params_.state_options.num_cameras!=2 || !params_.use_stereo || params_.state_options.max_slam_features!=0 ||
       params_.try_zupt || params_.use_aruco || params_.use_mask || params_.downsample_cameras ||
       params_.init_options.init_dyn_use || params_.state_options.do_calib_camera_pose ||
       params_.state_options.do_calib_camera_intrinsics || params_.state_options.do_calib_camera_timeoffset ||
       params_.state_options.do_calib_imu_intrinsics)
      throw std::invalid_argument("v1 requires stereo, MSCKF only, fixed calibration and ZUPT off");
    params_.use_multi_threading_subs=false; params_.use_multi_threading_pubs=false;
    params_.num_opencv_threads=1;
    estimator_=std::make_shared<ov_msckf::VioManager>(params_);
    world_=declare_parameter<std::string>("odom_frame","openvins_trial_odom");
    body_=declare_parameter<std::string>("base_frame","openvins_trial_base");
    imu_frame_=declare_parameter<std::string>("imu_frame","gyro_link");
    left_frame_=declare_parameter<std::string>("left_frame","d455_infra1_optical_frame");
    right_frame_=declare_parameter<std::string>("right_frame","d455_infra2_optical_frame");
    left_info_frame_=declare_parameter<std::string>("left_info_frame",left_frame_);
    right_info_frame_=declare_parameter<std::string>("right_info_frame",right_frame_);
    publish_tf_=declare_parameter<bool>("publish_tf",false);
    require_info_=declare_parameter<bool>("require_camera_info",true);
    auto translation=declare_parameter<std::vector<double>>("p_base_in_imu", {0.,0.,0.});
    auto rotation=declare_parameter<std::vector<double>>("q_base_to_imu_xyzw", {0.,0.,0.,1.});
    if(translation.size()!=3 || rotation.size()!=4) throw std::invalid_argument("invalid T_imu_base size");
    Eigen::Quaterniond q(rotation[3],rotation[0],rotation[1],rotation[2]);
    T_IB_=Eigen::Isometry3d::Identity();
    T_IB_.translation()=Eigen::Map<Eigen::Vector3d>(translation.data());
    if(!q.coeffs().allFinite() || std::abs(q.norm()-1)>1e-4 || !T_IB_.translation().allFinite())
      throw std::invalid_argument("invalid T_imu_base");
    T_IB_.linear()=q.toRotationMatrix();
    imu_timeout_=positive("imu_timeout_s",0.10);
    imu_gap_=positive("max_imu_gap_s",0.10);
    visual_timeout_=positive("visual_timeout_s",0.50);
    max_delay_=positive("max_input_age_s",0.50);
    sync_tol_=positive("stereo_tolerance_s",0.003);
    propagation_lag_=positive("camera_holdback_s",0.20);
    max_accel_=positive("max_acceleration_mps2",150.);
    max_gyro_=positive("max_angular_rate_rps",30.);
    pos_limit_=positive("max_position_std_m",0.5);
    rot_limit_=positive("max_orientation_std_rad",0.3);
    vel_limit_=positive("max_velocity_std_mps",0.5);
    min_features_=declare_parameter<int>("min_accepted_features",8);
    required_updates_=declare_parameter<int>("recovery_updates",3);
    if(min_features_<1 || required_updates_<1) throw std::invalid_argument("invalid recovery thresholds");
    std::random_device rd; std::ostringstream id;
    id << std::hex << rd() << rd() << std::chrono::system_clock::now().time_since_epoch().count();
    instance_=id.str();
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("odom",10);
    raw_pub_=create_publisher<nav_msgs::msg::Odometry>("odom_imu",10);
    status_pub_=create_publisher<Status>("estimator_status",10);
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("imu_topic","/wheel/imu/data_raw"),rclcpp::SensorDataQoS().keep_last(1000),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m){ on_imu(*m); });
    left_sub_=create_subscription<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("left_topic","/r680/d455/infra1/image_rect_raw"),rclcpp::SensorDataQoS().keep_last(4),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m){ on_image(m,0); });
    right_sub_=create_subscription<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("right_topic","/r680/d455/infra2/image_rect_raw"),rclcpp::SensorDataQoS().keep_last(4),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m){ on_image(m,1); });
    for(int id=0;id<2;++id) {
      info_sub_[id]=create_subscription<sensor_msgs::msg::CameraInfo>(
        declare_parameter<std::string>(id?"right_info_topic":"left_info_topic",
          id?"/r680/d455/infra2/camera_info":"/r680/d455/infra1/camera_info"),rclcpp::SensorDataQoS(),
        [this,id](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){
          const auto &cam=params_.camera_intrinsics.at(id);const auto k=cam->get_value();
          info_valid_[id]=m->header.frame_id==(id?right_info_frame_:left_info_frame_)&&m->width==unsigned(cam->w())&&
            m->height==unsigned(cam->h())&&std::isfinite(m->p[0]+m->p[5]+m->p[2]+m->p[6])&&
            std::abs(m->p[0]-k(0))<0.05&&std::abs(m->p[5]-k(1))<0.05&&
            std::abs(m->p[2]-k(2))<0.05&&std::abs(m->p[6]-k(3))<0.05;
          info_seen_[id]=Clock::now();
        });
    }
    reset_=create_service<std_srvs::srv::Trigger>("reset",[this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res){
      estimator_=std::make_shared<ov_msckf::VioManager>(params_);
      ++generation_; fault_.clear(); last_imu_=last_image_=last_visual_=state_stamp_=-1;
      last_left_=last_right_=-1; recovery_=accepted_=0; state_valid_=false;
      images_[0].clear(); images_[1].clear(); cameras_.clear();
      res->success=true; res->message="New generation and odom frame; map reinitialization required";
    });
    timer_=create_wall_timer(50ms,[this] {
      // Image work and the watchdog share this serialized executor. A timer can
      // become ready before queued IMU callbacks after an expensive camera update.
      // Consume pending samples before classifying the sensor as disconnected.
      sensor_msgs::msg::Imu pending; rclcpp::MessageInfo info;
      for(unsigned i=0;i<200 && fault_.empty() && imu_sub_->take(pending,info);++i) on_imu(pending);
      if(estimator_->initialized() && last_imu_>0 && (now().seconds()-last_imu_>imu_timeout_ ||
          elapsed(imu_seen_)>imu_timeout_)) fail("imu_timeout_requires_reset");
      if(last_imu_>now().seconds()+0.05) fail("clock_reversed_requires_reset");
      publish_status();
    });
  }
 private:
  using Clock=std::chrono::steady_clock;
  double positive(const std::string &name,double value) {
    value=declare_parameter<double>(name,value);
    if(!std::isfinite(value)||value<=0) throw std::invalid_argument(name); return value;
  }
  static double elapsed(Clock::time_point t) {return std::chrono::duration<double>(Clock::now()-t).count();}
  static double seconds(const builtin_interfaces::msg::Time &t) {return t.sec+1e-9*t.nanosec;}
  static builtin_interfaces::msg::Time stamp(double t) {return rclcpp::Time(int64_t(std::max(0.,t)*1e9));}
  bool fresh(double t) {const double age=now().seconds()-t; return t>0 && age>=-0.05 && age<=max_delay_;}
  std::string world() const {return generation_ ? world_+"_g"+std::to_string(generation_) : world_;}
  void fail(const std::string &why) {if(fault_.empty()) RCLCPP_ERROR(get_logger(),"%s",why.c_str()); fault_=why; state_valid_=false;}
  void on_image(sensor_msgs::msg::Image::ConstSharedPtr m,int id) {
    double t=seconds(m->header.stamp); double &last=id?last_right_:last_left_;
    if(!fault_.empty()) return;
    const auto &cam=params_.camera_intrinsics.at(id);
    if((require_info_&&(!info_valid_[id]||elapsed(info_seen_[id])>2.)) ||
       !fresh(t)||t<=last||m->header.frame_id!=(id?right_frame_:left_frame_)||
       m->width!=unsigned(cam->w())||m->height!=unsigned(cam->h())||m->encoding!="mono8" ||
       m->step<m->width || m->data.size()<size_t(m->step)*m->height) {++rejected_images_;return;}
    last=t; images_[id].push_back(m);
    if(images_[id].size()>4) {images_[id].pop_front();++rejected_images_;}
    while(!images_[0].empty()&&!images_[1].empty()) {
      const double a=seconds(images_[0].front()->header.stamp), b=seconds(images_[1].front()->header.stamp);
      if(std::abs(a-b)>sync_tol_) {images_[a<b?0:1].pop_front();++rejected_images_;continue;}
      ov_core::CameraData c; c.timestamp=a; c.sensor_ids={0,1};
      for(int i=0;i<2;++i) {
        const auto &image=*images_[i].front();
        c.images.push_back(cv::Mat(image.height,image.width,CV_8UC1,
          const_cast<unsigned char *>(image.data.data()),image.step).clone());
        c.masks.push_back(cv::Mat::zeros(c.images.back().size(),CV_8UC1)); images_[i].pop_front();
      }
      cameras_.push_back(c); last_image_=a;
      if(cameras_.size()>4) {cameras_.pop_front();++rejected_images_;}
    }
  }
  void on_imu(const sensor_msgs::msg::Imu &m) {
    const double t=seconds(m.header.stamp);
    if(!fault_.empty()) return;
    ov_core::ImuData imu; imu.timestamp=t;
    imu.wm<<m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z;
    imu.am<<m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z;
    if(m.header.frame_id!=imu_frame_||!imu.wm.allFinite()||!imu.am.allFinite()||
       imu.wm.norm()>max_gyro_||imu.am.norm()>max_accel_||m.angular_velocity_covariance[0]<0||
       m.linear_acceleration_covariance[0]<0) {
      RCLCPP_ERROR(get_logger(), "Rejected IMU: age=%.6f frame=%s expected=%s accel_norm=%.6f gyro_norm=%.6f accel_cov=%.6f gyro_cov=%.6f",
        now().seconds()-t,m.header.frame_id.c_str(),imu_frame_.c_str(),imu.am.norm(),imu.wm.norm(),
        m.linear_acceleration_covariance[0],m.angular_velocity_covariance[0]);
      ++rejected_imu_;fail("invalid_imu_requires_reset");return;
    }
    if(last_imu_>=0&&t<=last_imu_) {++rejected_imu_;fail("imu_nonmonotonic_requires_reset");return;}
    if(!fresh(t)) {
      ++rejected_imu_;
      if(estimator_->initialized() || t<=0) {fail("imu_timestamp_requires_reset");return;}
      // D455 global-time conversion can settle after streaming begins. Before
      // any odom exists, discard out-of-clock data and invalidate the entire
      // initialization window. Never feed it or widen runtime age limits.
      if(last_imu_>=0 && !startup_clock_wait_) {
        estimator_=std::make_shared<ov_msckf::VioManager>(params_);
        images_[0].clear();images_[1].clear();cameras_.clear();
        last_image_=last_visual_=-1;last_left_=last_right_=-1;
      }
      startup_clock_wait_=true;
      return;
    }
    startup_clock_wait_=false;
    if(last_imu_>=0&&t-last_imu_>imu_gap_) {
      if(estimator_->initialized()) {++rejected_imu_;fail("imu_gap_requires_reset");return;}
      // Before an odometry frame exists, a sensor warm-up gap restarts only the
      // initialization window. Never bridge missing IMU with a static solution.
      estimator_=std::make_shared<ov_msckf::VioManager>(params_);
      images_[0].clear();images_[1].clear();cameras_.clear();
      last_image_=last_visual_=-1;last_left_=last_right_=-1;
    }
    last_imu_=t; imu_seen_=Clock::now();
    estimator_->feed_measurement_imu(imu);
    const double dt=params_.calib_camimu_dt;
    while(!cameras_.empty() && cameras_.front().timestamp+dt<t) {
      auto camera=cameras_.front(); cameras_.pop_front();
      if(!fresh(camera.timestamp) || (estimator_->initialized() && camera.timestamp<=estimator_->get_state()->_timestamp)) {
        ++rejected_images_;continue;
      }
      const auto sequence=estimator_->msckf_update_sequence();
      const auto camera_start=Clock::now();
      estimator_->feed_measurement_camera(camera);
      const double camera_duration=elapsed(camera_start);
      if(camera_duration>0.05) RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
        "Camera update took %.1f ms; inspect callback latency",1000.*camera_duration);
      accepted_=estimator_->msckf_update_sequence()>sequence?estimator_->last_msckf_update_count():0;
      if(accepted_>=unsigned(min_features_)) {
        double current=camera.timestamp+dt;
        recovery_=(last_visual_>=0 && current-last_visual_<=visual_timeout_)?recovery_+1:1;
        last_visual_=current;
      }
    }
    if(!estimator_->initialized()) return;
    // Advance the same posterior during camera stop, at bounded 10 Hz. Keep a
    // holdback for delayed images; late images older than this posterior are rejected.
    double target=t-dt-propagation_lag_;
    if(target-estimator_->get_state()->_timestamp>0.1) estimator_->propagate_without_camera(target);
    wla_openvins::State x; wla_openvins::Covariance P;
    if(!estimator_->get_propagator()->fast_state_propagate(estimator_->get_state(),t,x,P)) {
      state_valid_=false; return;
    }
    try {
      if((P.diagonal().array() < -1e-8).any()) throw std::runtime_error("negative_covariance");
      const auto b=wla_openvins::adapt(x,P,T_IB_);
      const auto raw=wla_openvins::adapt(x,P,Eigen::Isometry3d::Identity());
      auto odom=message(b,t,body_);
      state_stamp_=t; state_valid_=true;
      pos_std_=std::sqrt(std::max(0.,b.covariance.diagonal().segment<3>(0).maxCoeff()));
      rot_std_=std::sqrt(std::max(0.,b.covariance.diagonal().segment<3>(3).maxCoeff()));
      vel_std_=std::sqrt(std::max(0.,b.covariance.diagonal().segment<3>(6).maxCoeff()));
      raw_pub_->publish(message(raw,t,"openvins_trial_imu")); odom_pub_->publish(odom);
      if(publish_tf_) {
        geometry_msgs::msg::TransformStamped tf;tf.header=odom.header;tf.child_frame_id=body_;
        tf.transform.translation.x=b.p.x();tf.transform.translation.y=b.p.y();tf.transform.translation.z=b.p.z();
        tf.transform.rotation=odom.pose.pose.orientation;tf_->sendTransform(tf);
      }
    } catch(const std::exception &e) {fail(e.what());}
  }
  nav_msgs::msg::Odometry message(const wla_openvins::BodyState &b,double t,const std::string &child) {
    nav_msgs::msg::Odometry m; m.header.stamp=stamp(t);m.header.frame_id=world();m.child_frame_id=child;
    auto &p=m.pose.pose.position;p.x=b.p.x();p.y=b.p.y();p.z=b.p.z();
    auto &q=m.pose.pose.orientation;q.x=b.q.x();q.y=b.q.y();q.z=b.q.z();q.w=b.q.w();
    auto &v=m.twist.twist.linear;v.x=b.v.x();v.y=b.v.y();v.z=b.v.z();
    auto &w=m.twist.twist.angular;w.x=b.w.x();w.y=b.w.y();w.z=b.w.z();
    for(int r=0;r<6;r++) for(int c=0;c<6;c++) {
      m.pose.covariance[r*6+c]=b.covariance(r,c);m.twist.covariance[r*6+c]=b.covariance(r+6,c+6);
    } return m;
  }
  void publish_status() {
    const double t=now().seconds();
    Status s;s.header.stamp=now();s.header.frame_id=world();s.instance_id=instance_;s.generation=generation_;
    s.state_stamp=stamp(state_stamp_);s.last_visual_update_stamp=stamp(last_visual_);
    s.initialized=estimator_->initialized();s.state_valid=state_valid_&&state_stamp_>0&&t-state_stamp_<=imu_timeout_;
    s.image_age_s=last_image_<0?-1:t-last_image_;s.imu_age_s=last_imu_<0?-1:t-last_imu_;
    s.visual_update_age_s=last_visual_<0?-1:t-last_visual_;
    s.position_std_max=pos_std_;s.orientation_std_max=rot_std_;s.velocity_std_max=vel_std_;
    s.uncertainty_acceptable=pos_std_<=pos_limit_&&rot_std_<=rot_limit_&&vel_std_<=vel_limit_;
    s.accepted_msckf_features=accepted_;s.recovery_updates=recovery_;
    s.rejected_imu=rejected_imu_;s.rejected_images=rejected_images_;
    s.mode=wla_openvins::mode(!fault_.empty(),s.initialized,s.state_valid,s.visual_update_age_s,
      visual_timeout_,s.uncertainty_acceptable,recovery_,required_updates_);
    s.reason=!fault_.empty()?fault_:s.mode==Status::INITIALIZING?"waiting_for_static_initialization_and_excitation":
      s.mode==Status::INERTIAL_ONLY?"no_recent_accepted_visual_update":s.mode==Status::RECOVERING?
      "waiting_for_visual_consistency_and_uncertainty":"ok";
    if(fault_.empty()&&require_info_&&(!info_valid_[0]||!info_valid_[1]||
       elapsed(info_seen_[0])>2.||elapsed(info_seen_[1])>2.)) s.reason="waiting_for_matching_camera_info";
    if(fault_.empty()&&!s.initialized&&(last_imu_<0||t-last_imu_>imu_timeout_)) s.reason="waiting_for_fresh_imu";
    if(fault_.empty()&&!s.initialized&&startup_clock_wait_) s.reason="waiting_for_imu_clock_stability";
    status_pub_->publish(s);
  }
  ov_msckf::VioManagerOptions params_;std::shared_ptr<ov_msckf::VioManager> estimator_;
  Eigen::Isometry3d T_IB_;std::string world_,body_,imu_frame_,left_frame_,right_frame_,left_info_frame_,right_info_frame_,instance_,fault_;
  bool publish_tf_=false,state_valid_=false,require_info_=true,info_valid_[2]={false,false},startup_clock_wait_=false;
  double imu_timeout_,imu_gap_,visual_timeout_,max_delay_,sync_tol_,propagation_lag_,max_accel_,max_gyro_;
  double pos_limit_,rot_limit_,vel_limit_,pos_std_=1e9,rot_std_=1e9,vel_std_=1e9;
  double last_imu_=-1,last_image_=-1,last_visual_=-1,state_stamp_=-1,last_left_=-1,last_right_=-1;
  uint64_t generation_=0,rejected_imu_=0,rejected_images_=0;unsigned recovery_=0,accepted_=0;
  int min_features_,required_updates_;Clock::time_point imu_seen_{};
  Clock::time_point info_seen_[2]{};
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> images_[2];std::deque<ov_core::CameraData> cameras_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_,raw_pub_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_,right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_[2];
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<Frontend>());} // serialize all access, including initialization
  catch(const std::exception &e) {std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
