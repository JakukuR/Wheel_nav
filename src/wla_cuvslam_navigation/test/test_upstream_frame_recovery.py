"""Compile production handling blocks with fault-injected SDK results, without ROS hardware.

The generated executable runs the actual Track catch, covariance diagnostic and
Odometry cache statements from cuvslam_odometry.cpp, linked to upstream caches.
This is a focused regression harness, not a live SDK or robot acceptance test.
"""
from pathlib import Path
import sys

source = Path(sys.argv[1]).read_text()

def block(start, end):
    a = source.index(start)
    return source[a:source.index(end, a)]

track = block('        cuvslam::PoseEstimate estimate;', '        const auto elapsed')
diagnostic = block('        const bool finite_covariance', '        if (previous_pose_stamp_')
covariance = block('        // Upstream ROS Odometry uses history covariance',
                   '        std_msgs::msg::Bool current_inertial;')
lost = block('        if (!estimate.world_from_rig)', '        const auto & result')

imu_catch = block('            try {\n              tracker_->RegisterImuMeasurement', '            last_call_stamp_ = timestamp;')
query = block('        cuvslam::Odometry::State sdk_state;', '        if (feature_visualizer_)')

harness = r'''
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "isaac_ros_cuvslam/impl/pose_cache.hpp"
namespace cuvslam {
struct PoseEstimate {int64_t timestamp_ns{}; std::optional<int> world_from_rig{1};};
struct Odometry {struct State {};using Gravity=std::array<float,3>;struct ImuState {};};
}
struct Tracker {
  int calls{},imu_calls{},fail_query{};
  void GetState(cuvslam::Odometry::State&) {if(fail_query==1)throw std::runtime_error("state");}
  std::optional<cuvslam::Odometry::Gravity> GetLastGravity() {
    if(fail_query==2)throw std::runtime_error("gravity");return cuvslam::Odometry::Gravity{};
  }
  std::optional<cuvslam::Odometry::ImuState> GetImuState() {
    if(fail_query==3)throw std::runtime_error("imu state");return cuvslam::Odometry::ImuState{};
  }
  bool IsLastPoseInertialOnly() {
    if(fail_query==4)throw std::runtime_error("pose source");return false;
  }
  void RegisterImuMeasurement(int,int) {
    if (++imu_calls==1) throw std::runtime_error("injected IMU sample failure");
  }
  cuvslam::PoseEstimate Track(int, int, int, int*) {
    if (++calls == 1) throw std::runtime_error("injected SDK Track failure");
    return {};
  }
};
struct Vector {double x{},y{},z{};};
struct Twist {Vector linear,angular;};
struct Message {
  struct {std::array<double,36> covariance{};} pose;
  struct {Twist twist;std::array<double,36> covariance{};} twist;
};
struct Result {std::array<float,36> covariance_xyz_rpy{};};
struct Harness {
  nvidia::isaac_ros::visual_slam::PoseCache pose_cache_;
  nvidia::isaac_ros::visual_slam::VelocityCache velocity_cache_;
  int warnings{},diagnostics{},lost_frames_{}; int64_t covariance_stamp{};
  void diagnostic(const std::string&,const std::string&,bool) {++diagnostics;}
  void status(const std::string&) {}
  void testQueries() {
    Tracker tracker;auto tracker_=&tracker;int accepted=0;
    #define RCLCPP_WARN(...) (++warnings)
    for(int n=1;n<=5;++n) {
      tracker.fail_query=n;
__QUERY__
      ++accepted;
    }
    #undef RCLCPP_WARN
    assert(accepted==1 && warnings==4);
    warnings=0;
  }
  void testImu() {
    Tracker tracker;auto tracker_=&tracker;int accepted=0,sample=0;
    #define RCLCPP_WARN(...) (++warnings)
    for(int n=0;n<2;++n) {
__IMU__
      ++accepted;
    }
    #undef RCLCPP_WARN
    assert(accepted==1 && tracker.imu_calls==2);
    warnings=0;
  }
  void testTrack() {
    Tracker tracker;auto tracker_=&tracker;
    int images=0,frame_options=0;bool force_keyframe=false;
    int outputs=0;
    #define RCLCPP_WARN(...) (++warnings)
    for(int frame=0;frame<2;++frame) {
__TRACK__
      ++outputs;
    }
    #undef RCLCPP_WARN
    assert(tracker.calls==2 && outputs==1 && warnings==1);
  }
  Message covariance(const Result& result,double x) {
    tf2::Vector3 position(x,0,0);
    tf2::Quaternion rotation(0,0,0,1);
    cuvslam::PoseEstimate estimate;estimate.timestamp_ns=++covariance_stamp*1000000000LL;
    Message message;message.twist.twist.linear.x=x;
__DIAGNOSTIC__
__COVARIANCE__
    return message;
  }
  void testLost() {
    cuvslam::PoseEstimate estimate;estimate.world_from_rig.reset();
    for(int frame=0;frame<1;++frame) {
__LOST__
      assert(false);
    }
    std::array<double,36> covariance{};
    assert(lost_frames_==1 && !pose_cache_.GetCovariance(covariance));
  }
};
int main() {
  Harness h;h.testQueries();h.testImu();h.testTrack();
  Result bad;
  const float diag[6]={-21052.55f,-12151.67f,-19649.19f,-5495.78f,-598.71f,-573.68f};
  for(int i=0;i<6;++i)bad.covariance_xyz_rpy[i*7]=diag[i];
  for(int n=0;n<10;++n) {
    const auto m=h.covariance(bad,n);
    for(int i=0;i<36;++i) {
      const double expected=n<9 ? (i%7==0 ? 1. : 0.) : (i==0 ? 8.25 : 0.);
      assert(std::isfinite(m.pose.covariance[i]));
      assert(std::abs(m.pose.covariance[i]-expected)<1e-9);
      const double twist_expected=n<9 ? (i%7==0 ? 1. : 0.) : (i==0 ? .09 : 0.);
      assert(std::abs(m.twist.covariance[i]-twist_expected)<1e-9);
    }
  }
  assert(h.diagnostics==10);
  h.testLost();
  // A recovered first pose has zero velocity, not a delta across the loss.
  auto recovered=h.covariance(bad,100.);
  assert(recovered.twist.twist.linear.x==0.);
  recovered=h.covariance(bad,101.);
  assert(std::abs(recovered.twist.twist.linear.x-1.)<1e-9);
  Harness stationary;
  bad.covariance_xyz_rpy[0]=std::numeric_limits<float>::quiet_NaN();
  for(int i=0;i<10;++i) {
    const auto m=stationary.covariance(bad,0);
    if(i==9)for(double c:m.pose.covariance)assert(c==0.);
  }
  assert(stationary.diagnostics==10);
  std::cout<<"PASS Track exception -> next frame; captured negative/NaN SDK covariance -> history covariance; startup identity; empty-pose cache reset\n";
}
'''
for key,value in [('QUERY',query),('IMU',imu_catch),('TRACK',track),('DIAGNOSTIC',diagnostic),('COVARIANCE',covariance),('LOST',lost)]:
    harness=harness.replace('__'+key+'__',value)
Path(sys.argv[2]).write_text(harness)
