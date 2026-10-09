#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "wla_r680_navigation/degraded_odometry.hpp"
using namespace std::chrono_literals;
namespace wla=wla_r680_navigation;
class DegradedOdometry final : public rclcpp::Node {
  using Clock=std::chrono::steady_clock;
  using Odom=nav_msgs::msg::Odometry;
public:
  DegradedOdometry():Node("r680_degraded_odometry") {
    enabled_=declare_parameter<bool>("enabled",true);
    odom_frame_=declare_parameter<std::string>("odom_frame","d455_floor_odom");
    base_frame_=declare_parameter<std::string>("base_frame","r680_mapping_floor");
    imu_frame_=declare_parameter<std::string>("expected_imu_frame","gyro_link");
    raw_timeout_=parameter("raw_timeout_s",0.15);
    imu_timeout_=parameter("imu_timeout_s",0.10);
    wheel_timeout_=parameter("wheel_timeout_s",0.15);
    seconds_=parameter("max_degraded_s",2.0);
    distance_=parameter("max_degraded_travel_m",0.30);
    turn_=parameter("max_degraded_turn_rad",0.80);
    sigma_xy_=parameter("max_position_sigma_m",0.10);
    sigma_yaw_=parameter("max_yaw_sigma_rad",0.15);
    return_xy_=parameter("visual_return_translation_m",0.12);
    return_yaw_=parameter("visual_return_yaw_rad",0.15);
    max_v_=parameter("max_measured_linear_mps",1.5);
    max_w_=parameter("max_measured_angular_radps",2.5);
    return_frames_=declare_parameter<int>("visual_return_frames",3);
    if(seconds_>3 || distance_>0.5 || turn_>1.0 || return_frames_<3)
      throw std::invalid_argument("degraded trial envelope exceeded");
    auto q=declare_parameter<std::vector<double>>("base_from_imu_rotation",{0,0,0,1});
    if(q.size()!=4 || !std::all_of(q.begin(),q.end(),[](double x){return std::isfinite(x);}))
      throw std::invalid_argument("invalid IMU rotation");
    tf2::Quaternion quat(q[0],q[1],q[2],q[3]);
    if(std::abs(quat.length()-1)>0.01) throw std::invalid_argument("IMU rotation not normalized");
    imu_rotation_=tf2::Matrix3x3(quat.normalized());
    pub_=create_publisher<Odom>(declare_parameter<std::string>("output_topic","/d455_slam/odom"),10);
    health_pub_=create_publisher<std_msgs::msg::Bool>("/r680_nav/continuous_odom_healthy",10);
    degraded_pub_=create_publisher<std_msgs::msg::Bool>("/r680_nav/odom_degraded",10);
    state_pub_=create_publisher<std_msgs::msg::String>("/r680_nav/continuous_odom_status",10);
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    auto qos=rclcpp::SensorDataQoS().keep_last(5);
    raw_sub_=create_subscription<Odom>(declare_parameter<std::string>("raw_odom_topic","/r680_nav/vio_raw_odom"),qos,
      [this](Odom::ConstSharedPtr m){onRaw(*m);});
    wheel_sub_=create_subscription<Odom>(declare_parameter<std::string>("wheel_topic","/wheel/odom"),qos,
      [this](Odom::ConstSharedPtr m){
        const auto &t=m->twist.twist;
        if(freshStamp(m->header.stamp,wheel_timeout_) && std::isfinite(t.linear.x+t.angular.z) &&
          std::abs(t.linear.x)<=max_v_ && std::abs(t.angular.z)<=max_w_) {
          v_=t.linear.x; wheel_w_=t.angular.z; wheel_seen_=Clock::now();
        }else wheel_seen_={};
      });
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>(declare_parameter<std::string>("imu_topic","/wheel/imu/data_raw"),qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m){onImu(*m);});
    health_sub_=create_subscription<std_msgs::msg::Bool>("/r680_nav/vio_tracking_healthy",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m){raw_ready_=m->data; health_seen_=Clock::now();});
    inertial_sub_=create_subscription<std_msgs::msg::Bool>("/r680_nav/vio_inertial_valid",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m){inertial_valid_=m->data; inertial_seen_=Clock::now();});
    visual_sub_=create_subscription<std_msgs::msg::Bool>("/r680_nav/vio_visual_observed",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m){visual_observed_=m->data;visual_seen_=Clock::now();});
    status_sub_=create_subscription<std_msgs::msg::String>("/r680_nav/vio_status",10,
      [this](std_msgs::msg::String::ConstSharedPtr m){
        raw_status_=m->data;
        if(initialized_ && raw_status_!="tracking_inertial_ready" &&
          raw_status_!="waiting_for_stable_inertial_initialization" && raw_status_!="tracking_lost_no_output")
          block("frontend fault: "+raw_status_);
      });
    reason_sub_=create_subscription<std_msgs::msg::String>("/r680_nav/vio_health_reason",10,
      [this](std_msgs::msg::String::ConstSharedPtr m){
        if(initialized_ && (m->data.find("gravity_")!=std::string::npos ||
          m->data.find("imu_state_unavailable")!=std::string::npos ||
          m->data.find("bias_out_of_bounds")!=std::string::npos)) block("SDK inertial evidence invalid: "+m->data);
      });
    watchdog_sub_=create_subscription<std_msgs::msg::Bool>("/r680_nav/vo_watchdog_healthy",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m){watchdog_ready_=m->data; watchdog_seen_=Clock::now();});
    timer_=create_wall_timer(10ms,[this]{tick();});
  }
private:
  double parameter(const char *name,double value) {
    const auto v=declare_parameter<double>(name,value);
    if(!std::isfinite(v) || v<=0) throw std::invalid_argument(name);
    return v;
  }
  static double age(Clock::time_point t) {
    return t.time_since_epoch().count()==0 ? 1e9 : std::chrono::duration<double>(Clock::now()-t).count();
  }
  bool freshStamp(const builtin_interfaces::msg::Time &stamp,double limit) const {
    const double a=(get_clock()->now()-rclcpp::Time(stamp)).seconds();return a>=-0.02 && a<=limit;
  }
  static double yaw(const geometry_msgs::msg::Quaternion &q) {
    return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  }
  static wla::PlanarPose pose(const Odom &m) {
    return {m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation)};
  }
  bool sensorReady() const {return bias_ready_ && age(wheel_seen_)<=wheel_timeout_ && age(imu_seen_)<=imu_timeout_;}
  bool inertialReady() const {return inertial_valid_ && age(inertial_seen_)<=raw_timeout_;}
  bool rawReady() const {return age(health_seen_)<=raw_timeout_ && raw_ready_;}
  bool visualObserved() const {return visual_observed_ && age(visual_seen_)<=raw_timeout_;}
  bool rawUsable() const {return age(raw_seen_)<=raw_timeout_ &&
    (!enabled_ || visualObserved()) && (rawReady() || (provisional_return_ && inertialReady()));}
  void onImu(const sensor_msgs::msg::Imu &m) {
    const auto &g=m.angular_velocity; const auto &a=m.linear_acceleration;
    const double acceleration=std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
    const int64_t stamp=rclcpp::Time(m.header.stamp).nanoseconds();
    if(m.header.frame_id!=imu_frame_ || !freshStamp(m.header.stamp,imu_timeout_) || stamp<=imu_stamp_ ||
      !std::isfinite(g.x+g.y+g.z+acceleration) || acceleration<6 || acceleration>13 ||
      m.angular_velocity_covariance[0]<0 || std::sqrt(g.x*g.x+g.y*g.y+g.z*g.z)>max_w_) {
      imu_seen_={}; return;
    }
    imu_stamp_=stamp; imu_seen_=Clock::now();
    gyro_z_=(imu_rotation_*tf2::Vector3(g.x,g.y,g.z)).z();
    if(!bias_ready_) {
      const bool stationary=age(wheel_seen_)<=wheel_timeout_ && std::abs(v_)<0.015 && std::abs(wheel_w_)<0.03;
      if(!stationary) {bias_count_=0; bias_sum_=bias_square_=0; bias_start_={};return;}
      if(bias_count_==0) bias_start_=Clock::now();
      ++bias_count_; bias_sum_+=gyro_z_; bias_square_+=gyro_z_*gyro_z_;
      if(bias_count_>=100 && age(bias_start_)>=1.0) {
        const double mean=bias_sum_/bias_count_;
        const double variance=std::max(0.0,bias_square_/bias_count_-mean*mean);
        if(std::abs(mean)<=0.25 && variance<=0.0009) {
          bias_=mean;bias_ready_=true;RCLCPP_INFO(get_logger(),"wheel/gyro stationary yaw bias=%.6f",bias_);
        }else {bias_count_=0;bias_sum_=bias_square_=0;bias_start_={};}
      }
    }
  }
  wla::PlanarPose transformed(const Odom &m) const {
    const auto p=pose(m);const double c=std::cos(offset_.yaw),s=std::sin(offset_.yaw);
    return {offset_.x+c*p.x-s*p.y,offset_.y+s*p.x+c*p.y,wla::wrapYaw(p.yaw+offset_.yaw)};
  }
  void onRaw(const Odom &m) {
    const auto &p=m.pose.pose; const auto &q=p.orientation;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    const int64_t stamp=rclcpp::Time(m.header.stamp).nanoseconds();
    if(m.header.frame_id!=odom_frame_ || m.child_frame_id!=base_frame_ ||
      !freshStamp(m.header.stamp,raw_timeout_) || !std::isfinite(p.position.x+p.position.y+p.position.z+norm) ||
      std::abs(norm-1)>0.02 || !std::isfinite(m.twist.twist.linear.x+m.twist.twist.angular.z) ||
      !std::all_of(m.pose.covariance.begin(),m.pose.covariance.end(),[](double c){return std::isfinite(c);}) ||
      m.pose.covariance[0]<0 || m.pose.covariance[7]<0 || m.pose.covariance[35]<0) {
      if(initialized_) block("invalid or stale visual pose");
      return;
    }
    // Old/duplicate poses cannot build recovery evidence or refresh freshness.
    if(stamp<=raw_stamp_) return;
    raw_stamp_=stamp; raw_seen_=Clock::now(); raw_=m;
    if(state_==State::Degraded) {
      // Only count frames produced after entering degradation, with SDK gravity/bias/velocity evidence.
      if(stamp<=entry_stamp_ || !inertialReady() || !visualObserved()) {return_count_=0;return;}
      const double lag=std::max(0.0,(get_clock()->now()-rclcpp::Time(m.header.stamp)).seconds());
      if(!wla::consistentVisual(transformed(m),estimate_,return_xy_+std::abs(v_)*lag,
          return_yaw_+std::abs(gyro_z_-bias_)*lag)) {
        block("visual return inconsistent with wheel/IMU prediction");return;
      }
      if(++return_count_>=return_frames_) {
        // Preserve the odom gauge across the return; RTAB-Map owns global correction.
        const auto rp=pose(m);offset_.yaw=wla::wrapYaw(estimate_.yaw-rp.yaw);
        offset_.x=estimate_.x-std::cos(offset_.yaw)*rp.x+std::sin(offset_.yaw)*rp.y;
        offset_.y=estimate_.y-std::sin(offset_.yaw)*rp.x-std::cos(offset_.yaw)*rp.y;
        state_=State::Normal;provisional_return_=true;
        RCLCPP_INFO(get_logger(),"visual tracking returned: %d consistent frames, odom continuity preserved",return_count_);
      }
    }
  }
  void block(const std::string &reason) {
    if(state_!=State::Blocked) {RCLCPP_ERROR(get_logger(),"continuous odom blocked: %s",reason.c_str());reason_=reason;}
    state_=State::Blocked; recovery_since_={};provisional_return_=false;
  }
  void publish(Odom m,bool predicted) {
    if(rclcpp::Time(m.header.stamp).nanoseconds()<=published_stamp_) return;
    published_stamp_=rclcpp::Time(m.header.stamp).nanoseconds();
    const auto old=pose(m);
    m.pose.pose.position.x=estimate_.x;m.pose.pose.position.y=estimate_.y;
    tf2::Quaternion delta;delta.setRPY(0,0,wla::wrapYaw(estimate_.yaw-old.yaw));
    const auto &mq=m.pose.pose.orientation;
    auto q=(delta*tf2::Quaternion(mq.x,mq.y,mq.z,mq.w)).normalized();
    m.pose.pose.orientation.x=q.x();m.pose.pose.orientation.y=q.y();
    m.pose.pose.orientation.z=q.z();m.pose.pose.orientation.w=q.w();
    if(predicted) {
      m.twist.twist.linear.x=v_;m.twist.twist.linear.y=m.twist.twist.linear.z=0;
      m.twist.twist.angular.x=m.twist.twist.angular.y=0;m.twist.twist.angular.z=gyro_z_-bias_;
      m.pose.covariance=entry_covariance_;m.twist.covariance.fill(0);
      m.pose.covariance[0]+=std::pow(budget_.positionSigma(),2);
      m.pose.covariance[7]+=std::pow(budget_.positionSigma(),2);
      m.pose.covariance[35]+=std::pow(budget_.yawSigma(),2);
      m.pose.covariance[14]+=1;m.pose.covariance[21]+=1;m.pose.covariance[28]+=1;
      m.twist.covariance[0]=0.01;m.twist.covariance[35]=0.01;
      m.twist.covariance[7]=m.twist.covariance[14]=m.twist.covariance[21]=m.twist.covariance[28]=1;
    } else {
      // Rotate the covariance with the persistent odom gauge, including cross terms.
      const auto cov=m.pose.covariance;
      const double c=std::cos(offset_.yaw),s=std::sin(offset_.yaw);
      double r[6][6]{};for(int i=0;i<6;++i)r[i][i]=1;
      r[0][0]=r[1][1]=r[3][3]=r[4][4]=c;
      r[0][1]=r[3][4]=-s;r[1][0]=r[4][3]=s;
      m.pose.covariance.fill(0);
      for(int i=0;i<6;++i)for(int j=0;j<6;++j)for(int k=0;k<6;++k)for(int l=0;l<6;++l)
        m.pose.covariance[6*i+j]+=r[i][k]*cov[6*k+l]*r[j][l];
    }
    output_=m;pub_->publish(m);
    geometry_msgs::msg::TransformStamped t;t.header=m.header;t.child_frame_id=base_frame_;
    t.transform.translation.x=m.pose.pose.position.x;t.transform.translation.y=m.pose.pose.position.y;
    t.transform.translation.z=m.pose.pose.position.z;t.transform.rotation=m.pose.pose.orientation;
    tf_->sendTransform(t);
  }
  void tick() {
    const auto current=Clock::now();
    const double dt=last_tick_.time_since_epoch().count()==0 ? 0.01 : std::chrono::duration<double>(current-last_tick_).count();
    last_tick_=current;
    if(state_==State::Waiting) {
      // Preserve the existing warming-up TF so costmap activation and the
      // restricted initialization supervisor do not wait on each other.
      // This raw passthrough never asserts localization/normal-motion health.
      if(age(raw_seen_)<=raw_timeout_) {estimate_=pose(raw_);publish(raw_,false);}
      if(rawReady() && (!enabled_ || visualObserved()) && age(raw_seen_)<=raw_timeout_)
        {state_=State::Normal;initialized_=true;estimate_=pose(raw_);offset_={};}
    }
    if(state_==State::Normal) {
      if(rawUsable()) {
        if(rawReady()) provisional_return_=false;
        estimate_=transformed(raw_);publish(raw_,false);
      }else if(initialized_) {
        if(enabled_ && sensorReady() && watchdog_ready_ && age(watchdog_seen_)<0.25 &&
            (raw_status_=="tracking_lost_no_output" || raw_status_=="tracking_inertial_ready" ||
             raw_status_=="waiting_for_stable_inertial_initialization")) {
          state_=State::Degraded;budget_={};return_count_=0;entry_stamp_=raw_stamp_;
          entry_covariance_=output_.pose.covariance;
          const double xx=entry_covariance_[0],yy=entry_covariance_[7];
          const double xy=0.5*(entry_covariance_[1]+entry_covariance_[6]);
          entry_xy_variance_=std::max(0.0,0.5*(xx+yy)+std::hypot(0.5*(xx-yy),xy));
          entry_yaw_variance_=entry_covariance_[35];
          // Include the gap since the last trusted image, not just time after
          // detection. The current timer step below supplies the final dt.
          double catchup=std::max(0.0,(get_clock()->now()-rclcpp::Time(output_.header.stamp)).seconds()-dt);
          if(catchup>0) {
            budget_.advance(v_,gyro_z_-bias_,catchup,seconds_,distance_,turn_);
            while(catchup>1e-9) {
              const double step=std::min(catchup,0.1);
              estimate_=wla::integrateWheelGyro(estimate_,v_,gyro_z_-bias_,step);
              catchup-=step;
            }
          }
          reason_.clear();RCLCPP_WARN(get_logger(),"visual degeneration: bounded wheel/IMU odom, max %.2fs %.2fm %.2frad",seconds_,distance_,turn_);
        }else block("no valid visual pose or no trusted wheel/IMU/map anchor");
      }
    }
    if(state_==State::Degraded) {
      if(!sensorReady() || !watchdog_ready_ || age(watchdog_seen_)>=0.25 || dt<=0 || dt>0.1)
        block("wheel/IMU/watchdog stale or scheduling gap");
      else if(!budget_.advance(v_,gyro_z_-bias_,dt,seconds_,distance_,turn_) ||
          std::sqrt(entry_xy_variance_+std::pow(budget_.positionSigma(),2))>sigma_xy_ ||
          std::sqrt(entry_yaw_variance_+std::pow(budget_.yawSigma(),2))>sigma_yaw_)
        block("degraded odom time/distance/angle/uncertainty budget exhausted");
      else {
        estimate_=wla::integrateWheelGyro(estimate_,v_,gyro_z_-bias_,dt);
        auto m=output_;m.header.stamp=get_clock()->now();publish(m,true);
      }
    }
    if(state_==State::Blocked) {
      // This only resumes odom for the existing stopped/map-verified watchdog recovery.
      // It never clears that watchdog's motion latch or replays a command.
      const bool stopped=age(wheel_seen_)<=wheel_timeout_ && std::abs(v_)<0.02 && std::abs(wheel_w_)<0.04;
      if(rawReady() && (!enabled_ || visualObserved()) && age(raw_seen_)<=raw_timeout_ && stopped) {
        if(recovery_since_.time_since_epoch().count()==0) recovery_since_=current;
        if(age(recovery_since_)>=2.0) {
          state_=State::Normal;estimate_=pose(raw_);offset_={};reason_.clear();
          RCLCPP_INFO(get_logger(),"stopped VIO restored; map-verified watchdog still owns motion unlock");
        }
      }else recovery_since_={};
    }
    std_msgs::msg::Bool h,d;h.data=state_==State::Normal || state_==State::Degraded;
    d.data=state_==State::Degraded;health_pub_->publish(h);degraded_pub_->publish(d);
    if(++ticks_%10==0) {
      std_msgs::msg::String s;
      s.data=std::string(state_==State::Normal ? "visual" : state_==State::Degraded ? "wheel_imu_degraded" :
        state_==State::Blocked ? "blocked" : "waiting_initialization")+
        " elapsed="+std::to_string(budget_.elapsed)+" travel="+std::to_string(budget_.travel)+
        " turn="+std::to_string(budget_.turn)+" bias_ready="+std::to_string(bias_ready_)+
        " raw_ready="+std::to_string(rawReady())+" visual_observed="+std::to_string(visualObserved())+
        " inertial_valid="+std::to_string(inertialReady())+" reason="+reason_;
      state_pub_->publish(s);
    }
  }
  enum class State {Waiting,Normal,Degraded,Blocked};State state_{State::Waiting};
  bool enabled_{},initialized_{},raw_ready_{},inertial_valid_{},watchdog_ready_{},bias_ready_{},provisional_return_{};
  bool visual_observed_{};Clock::time_point visual_seen_{};
  double raw_timeout_{},imu_timeout_{},wheel_timeout_{},seconds_{},distance_{},turn_{},sigma_xy_{},sigma_yaw_{};
  double return_xy_{},return_yaw_{},max_v_{},max_w_{},v_{},wheel_w_{},gyro_z_{},bias_{},bias_sum_{},bias_square_{};
  double entry_xy_variance_{},entry_yaw_variance_{};
  std::array<double,36> entry_covariance_{};
  int return_frames_{},return_count_{},bias_count_{},ticks_{};
  int64_t raw_stamp_{},imu_stamp_{},published_stamp_{},entry_stamp_{};
  std::string odom_frame_,base_frame_,imu_frame_,raw_status_,reason_;
  Clock::time_point raw_seen_{},wheel_seen_{},imu_seen_{},health_seen_{},inertial_seen_{},watchdog_seen_{},last_tick_{},bias_start_{},recovery_since_{};
  tf2::Matrix3x3 imu_rotation_;wla::PlanarPose estimate_{},offset_;wla::DegradedBudget budget_;
  Odom raw_,output_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<Odom>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_,degraded_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<Odom>::SharedPtr raw_sub_,wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_sub_,inertial_sub_,visual_sub_,watchdog_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_,reason_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<DegradedOdometry>());rclcpp::shutdown();return 0;
}
