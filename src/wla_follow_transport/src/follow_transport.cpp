#include "wla_follow_transport/channel.hpp"
#include "wla_follow_transport/policy.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <wla_behavior_interfaces/srv/person_command.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <map>
#include <set>
#include <memory>

using namespace wla_follow;
using PersonCommand=wla_behavior_interfaces::srv::PersonCommand;
using String=std_msgs::msg::String;
using Image=sensor_msgs::msg::Image;
using Info=sensor_msgs::msg::CameraInfo;
using Cloud=sensor_msgs::msg::PointCloud2;
using Twist=geometry_msgs::msg::Twist;
static int64_t stamp(const builtin_interfaces::msg::Time &s) { return int64_t(s.sec)*1000000000+s.nanosec; }
static int64_t source_stamp(const Json &j) {
  if(j.contains("stamp_ns") && j["stamp_ns"].is_number_integer()) return j["stamp_ns"].get<int64_t>();
  const auto &s=j.at("stamp");
  return s.at("sec").get<int64_t>()*1000000000+s.at("nanosec").get<int64_t>();
}
template<class T> void add_message(Packet &p,const std::string &name,const T &msg) {
  rclcpp::SerializedMessage bytes; rclcpp::Serialization<T>().serialize_message(&msg,&bytes);
  const auto &b=bytes.get_rcl_serialized_message(); p.add(name,b.buffer,b.buffer_length);
}
template<class T> T read_message(const Packet &p,const std::string &name) {
  auto bytes=p.blob(name); rclcpp::SerializedMessage s(bytes.size()); auto &b=s.get_rcl_serialized_message();
  memcpy(b.buffer,bytes.data(),bytes.size()); b.buffer_length=bytes.size();
  T msg; rclcpp::Serialization<T>().deserialize_message(&s,&msg); return msg;
}

class FollowTransport: public rclcpp::Node {
 public:
  FollowTransport():Node("person_follow",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    prefix=param<std::string>("prefix","/r680/person_follow");
    base=param<std::string>("base_frame","r680_mapping_floor");
    depth_topic=param<std::string>("depth_topic","/r680/d455/aligned_depth_to_color/image_raw");
    info_topic=param<std::string>("camera_info_topic","/r680/d455/color/camera_info");
    wheel_topic=param<std::string>("wheel_topic","/wheel/odom_50hz");
    relative_topic=param<std::string>("relative_topic","/r680/perception/relative_objects");
    detector_topic=param<std::string>("detection_status_topic","/r680_nav/perception/status");
    gate.enabled=param<bool>("enable_motion",false);
    gate.max_age=param<double>("max_sensor_age_s",.30); gate.result_ttl=param<double>("result_ttl_s",.15);
    gate.map_max_age=param<double>("map_max_age_s",gate.max_age);
    gate.result_hold=param<double>("result_hold_s",gate.result_ttl);
    mask_enabled=param<bool>("target_mask_enabled",true);
    terrain_topic=param<std::string>("terrain_topic","");
    const bool reid=param<bool>("reid_enabled",false), tof=param<bool>("tof_enabled",false);
    if(param<std::string>("output_mode","mux")!="mux" || param<bool>("use_sim_time",false) ||
       !std::isfinite(gate.max_age) || !(gate.max_age>0 && gate.max_age<=.30) ||
       !std::isfinite(gate.result_ttl) || !(gate.result_ttl>0 && gate.result_ttl<=.20) ||
       !std::isfinite(gate.map_max_age) || !(gate.map_max_age>=gate.max_age && gate.map_max_age<=.50) ||
       !std::isfinite(gate.result_hold) || !(gate.result_hold>=gate.result_ttl && gate.result_hold<=gate.map_max_age))
      throw std::runtime_error("hybrid transport requires mux, system clock and bounded freshness parameters");
    tf=std::make_unique<tf2_ros::Buffer>(get_clock());
    listener=std::make_shared<tf2_ros::TransformListener>(*tf,this,false);
    preview=create_publisher<Twist>(prefix+"/cmd_vel_candidate",1);
    candidate=create_publisher<String>(prefix+"/command_candidate",1);
    status=create_publisher<String>(prefix+"/status",5);
    interest=create_publisher<String>(prefix+"/reid_request",1);
    path=create_publisher<nav_msgs::msg::Path>(prefix+"/local_path",1);
    grid=create_publisher<nav_msgs::msg::OccupancyGrid>(prefix+"/local_obstacles",1);
    cloud=create_publisher<Cloud>(prefix+"/depth_obstacles",rclcpp::SensorDataQoS());
    subscribe_string(relative_topic,"relative"); subscribe_string(detector_topic,"detector");
    subscribe_string("/r680_base/control_state","control",5);
    if(reid) subscribe_string(prefix+"/reid_features","reid");
    if(mask_enabled) subscribe_string("/r680/perception/target_mask","mask");
    subs.push_back(create_subscription<Image>(depth_topic,rclcpp::SensorDataQoS(),[this](Image::ConstSharedPtr m){ guarded([&]{
      const auto ns=stamp(m->header.stamp); const double age=(now().nanoseconds()-ns)*1e-9;
      if(ns>last_depth && age>=-.05 && age<=gate.max_age && m->width && m->height &&
         uint64_t(m->width)*m->height<=2000000 && m->data.size()<=8000000) {
        depths[ns]=m; while(depths.size()>16) depths.erase(depths.begin());
      }
    }); }));
    subs.push_back(create_subscription<Info>(info_topic,rclcpp::SensorDataQoS(),[this](Info::ConstSharedPtr m){ guarded([&]{
      infos.push_back(m); if(infos.size()>24) infos.pop_front();
    }); }));
    subs.push_back(create_subscription<nav_msgs::msg::Odometry>(wheel_topic,rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){ guarded([&]{
      const auto old=motion.generation; const auto &v=m->twist.twist;
      bool accepted=false;
      if(m->child_frame_id!=base) motion.reset();
      else accepted=motion.update(stamp(m->header.stamp)*1e-9,{v.linear.x,v.linear.y,v.angular.z},now().seconds());
      if(accepted) gate.wheel_received=monotonic();
      gate.motion_generation=motion.generation;
      const auto &p=m->pose.pose.position;const auto &q=m->pose.pose.orientation;
      wheel_diagnostic={{"stamp_ns",stamp(m->header.stamp)},{"frame_id",m->header.frame_id},{"child_frame_id",m->child_frame_id},
        {"position",{p.x,p.y,p.z}},{"orientation",{q.x,q.y,q.z,q.w}},{"twist",{v.linear.x,v.linear.y,v.angular.z}},
        {"accepted",accepted},{"received",monotonic()}};
      if(old!=motion.generation) { last_motion_sent=0; barrier("wheel_reference_changed",false); }
    }); }));
    if(!terrain_topic.empty()) {
      // Slope/step obstacle cloud from wla_terrain_analysis (base frame, z = height
      // above local ground). Forwarded as-is; Python uses it instead of its own
      // single-frame height filter.
      subs.push_back(create_subscription<Cloud>(terrain_topic,rclcpp::SensorDataQoS(),[this](Cloud::ConstSharedPtr m){ guarded([&]{
        const double age=(now().nanoseconds()-stamp(m->header.stamp))*1e-9;
        if(age>=-.05 && age<=gate.max_age && m->data.size()<=4000000) terrain=m;
      }); }));
    }
    if(tof) {
      const auto topics=param<std::vector<std::string>>("tof_topics",{"/r680/tof/front_left/points","/r680/tof/front_right/points","/r680/tof/rear_left/points","/r680/tof/rear_right/points"});
      for(const auto &topic:topics) subs.push_back(create_subscription<Cloud>(topic,rclcpp::SensorDataQoS(),[this,topic](Cloud::ConstSharedPtr m){ guarded([&]{
        const double age=(now().nanoseconds()-stamp(m->header.stamp))*1e-9;
        if(uint64_t(m->width)*m->height<=256 && m->data.size()<=65536 && age>=0 && age<=.15) tofs[topic]=m;
      }); }));
    }
    command_service=create_service<PersonCommand>(prefix+"/command",
      [this](std::shared_ptr<rmw_request_id_t> header,std::shared_ptr<PersonCommand::Request> request){
        try { command(header,request); }
        catch(const std::exception &e) {
          // Pending requests die with the transport; this caller still gets an answer.
          for(auto it=pending.begin();it!=pending.end();) it=it->second.header==header?pending.erase(it):std::next(it);
          try { reply(header,*request,false,"compute_transport_failure"); } catch(const std::exception &) {}
          fail(e.what());
        }
      });
    estop_service=create_service<std_srvs::srv::SetBool>(prefix+"/set_estop",
      [this](std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res){
        try {
          gate.estop=req->data; barrier(req->data?"local_estop":"estop_reset_requires_new_request",true);
          if(ready) checked_send({{"kind","estop"},{"data",req->data},{"generation",gate.generation}});
          res->success=true; res->message=req->data?"latched":"reset; new request required"; publish_command();
        } catch(const std::exception &e) {
          // fail() latches estop and zeroes output, so a stop request still succeeded.
          fail(e.what()); res->success=req->data; res->message="compute_transport_failure";
        }
      });
    const auto python=param<std::string>("worker_python","/usr/bin/python3");
    for(const auto &name:list_parameters({},0).names) {
      const auto p=get_parameter(name);
      switch(p.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL: config[name]=p.as_bool();break;
        case rclcpp::ParameterType::PARAMETER_INTEGER: config[name]=p.as_int();break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE: config[name]=p.as_double();break;
        case rclcpp::ParameterType::PARAMETER_STRING: config[name]=p.as_string();break;
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: config[name]=p.as_string_array();break;
        default:break;
      }
    }
    session=channel.directory.substr(channel.directory.find_last_of('/')+1);
    channel.spawn(python); started=monotonic();
    poll_timer=create_wall_timer(std::chrono::milliseconds(5),[this]{ safe_poll(); });
    input_timer=create_wall_timer(std::chrono::milliseconds(20),[this]{ if(ready) send_input(); });
    output_timer=create_wall_timer(std::chrono::milliseconds(50),[this]{ guarded([&]{ publish_command(); publish_status(); }); });
    RCLCPP_INFO(get_logger(),"C++ ROS transport + Python compute; output=mux, motion=%s, worker_pid=%d",gate.enabled?"enabled":"preview",channel.child);
  }

  int exit_code() const { return failed ? 1 : 0; }

 private:
  bool failed=false;
  Channel channel; OutputGate gate; Motion motion;
  std::string prefix,base,depth_topic,info_topic,wheel_topic,relative_topic,detector_topic,session,terrain_topic;
  Cloud::ConstSharedPtr terrain;int64_t terrain_sent=0;
  std::string terminal_state="idle",terminal_reason="waiting_for_command",active_mode;
  bool ready=false,mask_enabled=true,control_known=false;
  double started=0,last_frame_send=0,last_motion_sent=0,last_status_at=0;
  uint64_t rpc_sequence=0,output_sequence=0;
  int64_t last_depth=0,detector_status_stamp=0,disabled_stamp=0;
  Json config,latest_status,latest_preview={{"linear",0.},{"angular",0.}},wheel_diagnostic,control;
  std::map<std::string,Json> updates;
  std::set<std::string> retired_controls,retired_detectors,tombstones;
  std::map<int64_t,Image::ConstSharedPtr> depths;
  std::map<int64_t,Json> masks;
  std::deque<Info::ConstSharedPtr> infos;
  std::map<std::string,Cloud::ConstSharedPtr> tofs;
  std::map<std::string,int64_t> tof_sent;
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<tf2_ros::TransformListener> listener;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  rclcpp::Publisher<Twist>::SharedPtr preview;
  rclcpp::Publisher<String>::SharedPtr candidate,status,interest;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid;
  rclcpp::Publisher<Cloud>::SharedPtr cloud;
  rclcpp::Service<PersonCommand>::SharedPtr command_service;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_service;
  rclcpp::TimerBase::SharedPtr poll_timer,input_timer,output_timer;
  struct Pending { std::shared_ptr<rmw_request_id_t> header; std::shared_ptr<PersonCommand::Request> request; double received; uint64_t cancel_generation; };
  std::map<uint64_t,Pending> pending;

  template<class T> T param(const std::string &name,const T &value) {
    return has_parameter(name)?get_parameter(name).get_value<T>():declare_parameter<T>(name,value);
  }
  // Any fault inside a ROS callback stops the transport through fail(): zero
  // command, final status, worker shutdown and a non-zero exit code.
  template<class Body> void guarded(Body &&body) {
    if(failed) return;
    try { body(); } catch(const std::exception &e) { fail(e.what()); }
  }
  void checked_send(const Json &j) {
    if(!channel.send(j)) throw std::runtime_error("IPC backpressure/disconnect; output revoked");
  }
  void barrier(const std::string &reason,bool cancel) {
    if(cancel) {
      if(!gate.request.empty()) tombstones.insert(gate.request);
      terminal_reason=reason;
      terminal_state=(reason=="stopped_by_agent" || reason=="local_estop" || reason=="estop_reset_requires_new_request")?"stopped":
        ((reason=="command_lease_expired" || reason=="lease_or_authority_expired")?"expired":"cancelled");
      if(latest_status.is_object() && latest_status.value("request_id","")==gate.request &&
         latest_status.value("reason","")==reason && !latest_status.value("enabled",false)) {
        const auto state=latest_status.value("state","");
        if(state=="arrived" || state=="stopped" || state=="cancelled" || state=="expired") terminal_state=state;
      }
      gate.revoke();
    }
    else gate.invalidate();
    if(ready) checked_send({{"kind","barrier"},{"generation",gate.generation},{"cancel",cancel},{"reason",reason}});
  }
  void subscribe_string(const std::string &topic,const std::string &role,int depth=1) {
    subs.push_back(create_subscription<String>(topic,depth,[this,topic,role](String::ConstSharedPtr m){ guarded([&]{
      if(m->data.size()>360000) { if(role=="relative") gate.observations=false; return; }
      try {
        Json j=Json::parse(m->data);
        if(role=="control") control_update(j);
        else if(role=="relative" || role=="detector") detector_update(j,role=="relative");
        else if(role=="mask" && j.value("schema","")=="wla-target-mask/v1" && j.contains("depth_stamp_ns") && j["depth_stamp_ns"].is_number_integer()) {
          masks[j["depth_stamp_ns"].get<int64_t>()]=j; while(masks.size()>16) masks.erase(masks.begin());
        }
        updates[topic]=j;
      } catch(const Json::exception &) { if(role=="relative") gate.observations=false; }
    }); }));
  }
  void control_update(const Json &j) {
    const auto ns=j.at("stamp_ns").get<int64_t>(); const double age=(now().nanoseconds()-ns)*1e-9;
    const auto name=j.at("control_session").get<std::string>();
    const auto source=j.at("automatic_source").get<std::string>(),mode=j.at("mode").get<std::string>();
    const auto generation=j.at("source_generation").get<int64_t>(),revoke=j.at("person_revoke_generation").get<int64_t>();
    if(age<-.05 || age>.50 || name.empty() || retired_controls.count(name) || generation<0 || revoke<0 ||
       (mode!="AUTO" && mode!="MANUAL") || (source!="NAV" && source!="PERSON" && source!="OBJECT")) return;
    const bool passthrough=j.contains("velocity_passthrough") && j["velocity_passthrough"].is_boolean() && j["velocity_passthrough"].get<bool>();
    const bool permitted=!(j.contains("estop_latched") && j["estop_latched"]==true) && (passthrough || (mode=="AUTO" && source=="PERSON"));
    bool revoked=false;
    if(control_known) {
      if(control["control_session"]==name) {
        if(ns<=control["stamp_ns"].get<int64_t>() || generation<control["source_generation"].get<int64_t>() || revoke<control["person_revoke_generation"].get<int64_t>()) return;
      } else { retired_controls.insert(control["control_session"].get<std::string>()); revoked=true; }
      revoked|=control["person_revoke_generation"]!=revoke || (gate.authority && !permitted);
    }
    control=j; control_known=true; gate.authority=permitted;
    gate.authority_until=monotonic()+.50-std::max(0.,age);
    if(revoked) barrier("motion_authority_revoked",true);
  }
  void detector_update(const Json &j,bool relative) {
    if(j.at("schema")!=(relative?"wla-relative-detections/v1":"wla-relative-perception-status/v1") || !j.at("enabled").is_boolean()) return;
    const auto ns=source_stamp(j); const double age=(now().nanoseconds()-ns)*1e-9;
    const auto name=j.at("session").get<std::string>(); const auto epoch=j.at("epoch").get<int64_t>();
    if(ns<=0 || age<-.05 || age>gate.max_age || name.empty() || name.size()>128 || epoch<0 || retired_detectors.count(name) ||
       (name==gate.detector_session && epoch<gate.detector_epoch)) return;
    if(relative && (j.at("base_frame")!=base || !j.at("objects").is_array() || j["objects"].size()>64)) { gate.observations=false;return; }
    if(name!=gate.detector_session || epoch!=gate.detector_epoch) {
      if(!gate.detector_session.empty() && name!=gate.detector_session) retired_detectors.insert(gate.detector_session);
      if(gate.active) barrier("detector_generation_changed",true);
      gate.detector_session=name; gate.detector_epoch=epoch;
      gate.observations=false;gate.observation_stamp=0;detector_status_stamp=0;disabled_stamp=0;
    }
    if(relative) {
      if(ns<=gate.observation_stamp || (j["enabled"]==true && ns<=disabled_stamp)) { gate.observations=false;return; }
      gate.observation_stamp=ns; gate.observations=j["enabled"].get<bool>();
    } else { if(ns<=detector_status_stamp) return;detector_status_stamp=ns; }
    if(j["enabled"]==false) {
      disabled_stamp=ns;gate.observations=false;
      if(gate.active) barrier("detection_disabled",true);
    }
    if(retired_controls.size()>1024 || retired_detectors.size()>1024 || tombstones.size()>10000)
      throw std::runtime_error("session history exhausted; restart required");
  }
  void reply(const std::shared_ptr<rmw_request_id_t> &header,const PersonCommand::Request &req,bool accepted,const std::string &reason) {
    PersonCommand::Response res;res.accepted=accepted;res.reason=reason;res.request_id=req.request_id.empty()?gate.request:req.request_id;
    const bool owned=!gate.request.empty() && res.request_id==gate.request;
    res.mode=owned && latest_status.is_object()?latest_status.value("mode",req.mode):req.mode;
    res.target_id=owned && latest_status.is_object()?latest_status.value("target_id",req.target_id):req.target_id;
    res.enabled=accepted && gate.active && owned;
    // A rejected renewal must still report the owned task's first terminal
    // state (especially arrived), as the Python/Agent service contract does.
    res.state=owned && !gate.active && !terminal_state.empty()?terminal_state:
      (accepted?(res.enabled?"waiting":"stopped"):"rejected");
    command_service->send_response(*header,res);
  }
  void command(std::shared_ptr<rmw_request_id_t> header,std::shared_ptr<PersonCommand::Request> req) {
    const double t=monotonic();
    if(req->request_id.size()>256 || req->target_id.size()>512 || req->mode.size()>32 || req->command.size()>32) {
      reply(header,*req,false,"invalid_request");return;
    }
    if(req->command=="stop") {
      if(!req->request_id.empty() && !tombstones.count(req->request_id) && tombstones.size()>=10000) {
        reply(header,*req,false,"request_history_full_restart_required");return;
      }
      if(!req->request_id.empty()) tombstones.insert(req->request_id);
      bool pending_start=false;
      for(const auto &[id,p]:pending) {
        (void)id;
        pending_start|=p.request->command=="start" && (req->request_id.empty() || p.request->request_id==req->request_id);
      }
      if(pending_start || ((req->request_id.empty() || req->request_id==gate.request) && (gate.active || gate.request.empty())))
        barrier("stopped_by_agent",true);
      if(ready) checked_send({{"kind","rpc"},{"id",0},{"generation",gate.generation},{"command","stop"},{"request_id",req->request_id},{"mode",req->mode},{"target_id",req->target_id},{"lease_duration_s",1.}});
      reply(header,*req,true,(!req->request_id.empty() && req->request_id!=gate.request)?"request_cancelled_before_start":"stopped");publish_command();return;
    }
    if(!ready || pending.size()>=16) { reply(header,*req,false,"compute_worker_unavailable");return; }
    if((req->command=="start" || req->command=="heartbeat") &&
       (req->request_id.empty() || !std::isfinite(req->lease_duration_s) || req->lease_duration_s<.1 || req->lease_duration_s>2. ||
        gate.estop || tombstones.count(req->request_id))) {
      reply(header,*req,false,"transport_request_revoked_or_invalid");return;
    }
    if(req->command=="heartbeat" && (!gate.active || req->request_id!=gate.request || t>=gate.lease_until)) {
      reply(header,*req,false,"no_matching_active_request");return;
    }
    if(req->command=="start" && (req->request_id!=gate.request || !gate.active)) barrier("task_start_barrier",false);
    const auto id=++rpc_sequence; pending[id]={header,req,t,gate.cancel_generation};
    checked_send({{"kind","rpc"},{"id",id},{"generation",gate.generation},{"command",req->command},
      {"request_id",req->request_id},{"mode",req->mode},{"target_id",req->target_id},{"lease_duration_s",req->lease_duration_s}});
  }
  void rpc_reply(const Json &j) {
    const auto id=j.at("id").get<uint64_t>(); const auto it=pending.find(id);if(it==pending.end()) return;
    const auto p=it->second;pending.erase(it);const auto &req=*p.request;const auto &body=j.at("response");
    if(p.cancel_generation!=gate.cancel_generation || monotonic()-p.received>1. ||
       (req.command!="status" && tombstones.count(req.request_id))) { reply(p.header,req,false,"request_superseded_or_expired");return; }
    PersonCommand::Response res;res.accepted=body.at("accepted");res.reason=body.at("reason");res.request_id=body.at("request_id");
    res.state=body.at("state");res.mode=body.at("mode");res.target_id=body.at("target_id");res.enabled=body.at("enabled");
    if(res.accepted && res.enabled && (req.command=="start" || req.command=="heartbeat")) {
      const bool duplicate=req.command=="start" && gate.active && gate.request==req.request_id;
      if(!duplicate) {
        if(monotonic()>=p.received+req.lease_duration_s || (req.command=="heartbeat" && (!gate.active || monotonic()>=gate.lease_until))) {
          barrier("command_lease_expired",true);reply(p.header,req,false,"command_lease_expired");return;
        }
        gate.lease_until=p.received+req.lease_duration_s;
      }
      gate.request=res.request_id;gate.active=true;active_mode=res.mode;terminal_state.clear();terminal_reason.clear();
    }
    command_service->send_response(*p.header,res);
  }
  void send_input() {
    if(channel.input_busy) return;
    try {
      Packet p;p.metadata={{"schema","wla-follow-input/v1"},{"generation",gate.generation},{"motion_generation",motion.generation},
        {"wheel_received",gate.wheel_received},{"wheel_diagnostic",wheel_diagnostic},{"motion",Json::array()},{"events",updates},
        {"transforms",Json::object()},{"tof_topics",Json::array()}};
      for(const auto &s:motion.samples) if(s.stamp>last_motion_sent) p.metadata["motion"].push_back({s.stamp,s.pose,s.twist});
      const auto ros_ns=now().nanoseconds();int64_t chosen=0;const double t=monotonic();
      if(t-last_frame_send>=.1) {
        for(auto it=depths.rbegin();it!=depths.rend();++it) {
          const auto ns=it->first;const auto &image=it->second;
          if(ns<=last_depth || ros_ns<ns || (ros_ns-ns)*1e-9>gate.max_age) continue;
          auto mask=masks.find(ns);
          // Only object approach (mode 2) needs its own frame's segmentation (object
          // surface). Person modes take the newest depth; Python removes the person
          // by tracked position when that frame's mask has not arrived yet.
          if(gate.active && mask_enabled && active_mode=="2" &&
             (mask==masks.end() || mask->second.value("request_id","")!=gate.request)) continue;
          Info::ConstSharedPtr info;int64_t skew=250000001;
          for(const auto &i:infos) {
            const auto distance=std::abs(stamp(i->header.stamp)-ns);
            if(i->header.frame_id==image->header.frame_id && i->width==image->width && i->height==image->height && distance<skew) { info=i;skew=distance; }
          }
          if(!info) continue;
          try {
            auto tr=tf->lookupTransform(base,image->header.frame_id,rclcpp::Time(ns));
            add_message(p,"depth",*image);add_message(p,"info",*info);add_message(p,"depth_transform",tr);
            p.metadata["transforms"][image->header.frame_id+":"+std::to_string(ns)]="depth_transform";
            if(mask!=masks.end() && mask->second.value("request_id","")==gate.request)
              p.metadata["events"]["/r680/perception/target_mask"]=mask->second;
            chosen=ns;break;
          } catch(const tf2::TransformException &) { continue; }
        }
      }
      if(terrain && stamp(terrain->header.stamp)>terrain_sent && (ros_ns-stamp(terrain->header.stamp))*1e-9<=gate.max_age) {
        add_message(p,"terrain",*terrain);p.metadata["terrain_topic"]=terrain_topic;terrain_sent=stamp(terrain->header.stamp);
      }
      for(const auto &[topic,m]:tofs) {
        const auto ns=stamp(m->header.stamp);
        if(ns<=tof_sent[topic] || ros_ns<ns || (ros_ns-ns)*1e-9>.15) continue;
        try {
          auto tr=tf->lookupTransform(base,m->header.frame_id,rclcpp::Time(ns));
          const auto key="tof"+std::to_string(p.metadata["tof_topics"].size());
          add_message(p,key,*m);add_message(p,key+"_transform",tr);
          p.metadata["tof_topics"].push_back({topic,key});p.metadata["transforms"][m->header.frame_id+":"+std::to_string(ns)]=key+"_transform";
        } catch(const tf2::TransformException &) { }
      }
      const auto bytes=p.write(channel.memory);const auto seq=++channel.input_sequence;
      checked_send({{"kind","input"},{"sequence",seq},{"size",bytes}});channel.input_busy=true;
      updates.clear();if(!motion.samples.empty()) last_motion_sent=motion.samples.back().stamp;
      for(const auto &entry:p.metadata["tof_topics"]) tof_sent[entry[0].get<std::string>()]=stamp(tofs.at(entry[0].get<std::string>())->header.stamp);
      if(chosen) {
        gate.delivered(chosen);last_depth=chosen;last_frame_send=t;
        depths.erase(depths.begin(),depths.upper_bound(chosen));masks.erase(masks.begin(),masks.upper_bound(chosen));
      }
    } catch(const std::exception &e) { fail(e.what()); }
  }
  void receive(const Json &j) {
    const auto kind=j.at("kind").get<std::string>();
    if(kind=="ready") { ready=true;return; }
    if(kind=="input_ack") { if(j.at("sequence").get<uint64_t>()==channel.input_sequence) channel.input_busy=false;return; }
    if(kind=="rpc_reply") { rpc_reply(j);return; }
    if(kind=="result") {
      const auto seq=j.at("proof").at("sequence").get<uint64_t>();if(seq<=gate.sequence) return;
      if(j.at("proof").at("generation").get<uint64_t>()!=gate.generation) return;
      gate.result=Json();gate.accept(j.at("proof"),monotonic(),now().nanoseconds());
      latest_status=j.at("status");latest_preview=j.at("preview");last_status_at=monotonic();
      if(gate.active && latest_status.value("request_id","")==gate.request && latest_status.value("enabled",false)==false)
        barrier(latest_status.value("reason","python_task_terminated"),true);
      publish_command();return;
    }
    if(kind=="interest") {
      if(j.at("generation").get<uint64_t>()!=gate.generation) return;
      auto body=Json::parse(j.at("data").get<std::string>());
      if(body.value("enabled",false) && (!gate.active || body.value("request_id","")!=gate.request)) return;
      String m;m.data=body.dump();interest->publish(m);return;
    }
    if(kind=="geometry") {
      const auto seq=j.at("sequence").get<uint64_t>();
      if(seq<=output_sequence) throw std::runtime_error("replayed geometry slot");
      auto p=Packet::read(channel.memory+SLOT_BYTES,j.at("size").get<size_t>());output_sequence=seq;
      if(p.metadata.at("generation").get<uint64_t>()==gate.generation && monotonic()-p.metadata.at("generated_at").get<double>()<=gate.max_age) {
        const auto &blobs=p.metadata.at("blobs");
        if(blobs.contains("path")) path->publish(read_message<nav_msgs::msg::Path>(p,"path"));
        if(blobs.contains("grid")) grid->publish(read_message<nav_msgs::msg::OccupancyGrid>(p,"grid"));
        if(blobs.contains("cloud")) cloud->publish(read_message<Cloud>(p,"cloud"));
      }
      checked_send({{"kind","geometry_ack"},{"sequence",seq}});return;
    }
    throw std::runtime_error("unknown worker message");
  }
  void safe_poll() {
    try {
      if(!channel.alive()) { fail("Python worker exited");return; }
      if(channel.accept_peer()) checked_send({{"kind","hello"},{"protocol",1},{"config",config},{"session",session},{"generation",gate.generation}});
      if(channel.peer>=0) { Json j;for(int i=0;i<32 && channel.receive(j);++i) receive(j); }
      const double t=monotonic();
      if(!ready && t-started>30.) { fail("Python worker startup timeout");return; }
      if(gate.active && (t>=gate.lease_until || (control_known && t>=gate.authority_until))) barrier("lease_or_authority_expired",true);
      for(auto it=pending.begin();it!=pending.end();) {
        if(t-it->second.received>1.) {
          if(it->second.request->command=="start" || it->second.request->command=="heartbeat") {
            tombstones.insert(it->second.request->request_id);barrier("compute_worker_timeout",true);
          }
          reply(it->second.header,*it->second.request,false,"compute_worker_timeout");it=pending.erase(it);
        }
        else ++it;
      }
    } catch(const std::exception &e) { fail(e.what()); }
  }
  // Command candidate: on the 20 Hz timer and immediately on every result,
  // stop and estop, so the mux sees the newest gated value without waiting.
  void publish_command() {
    const auto ns=now().nanoseconds();const double t=monotonic();
    const auto reason=gate.blocker(t,ns,motion.healthy(ns*1e-9));const bool permitted=reason.empty();
    const double v=permitted?gate.result.at("linear").get<double>():0.,w=permitted?gate.result.at("angular").get<double>():0.;
    String m;m.data=Json({{"schema","wla-person-candidate/v1"},{"follow_session",session},{"request_id",gate.request},
      {"stamp_ns",ns},{"linear",v},{"angular",w}}).dump();candidate->publish(m);
    Twist pv;
    if(last_status_at>0 && t-last_status_at<gate.result_ttl) { pv.linear.x=latest_preview.value("linear",0.);pv.angular.z=latest_preview.value("angular",0.); }
    preview->publish(pv);
  }
  // Status: 20 Hz timer only (and once from fail()); per-result copies doubled the
  // rate that every Python status subscriber has to parse.
  void publish_status() {
    const auto ns=now().nanoseconds();const double t=monotonic();
    const auto reason=gate.blocker(t,ns,motion.healthy(ns*1e-9));const bool permitted=reason.empty();
    const double v=permitted?gate.result.at("linear").get<double>():0.,w=permitted?gate.result.at("angular").get<double>():0.;
    String m;
    Json s=latest_status.is_object()?latest_status:Json{{"schema","wla-person-status/v1"},{"state","idle"},{"reason","waiting_for_compute_worker"},{"enabled",false}};
    s["stamp_ns"]=ns;s["follow_session"]=session;s["transport_backend"]="cpp_ros_python_compute";
    s["transport_generation"]=gate.generation;s["transport_worker_pid"]=channel.child;s["transport_worker_age_s"]=last_status_at>0?Json(t-last_status_at):Json();
    s["motion_permitted"]=permitted;s["actual_published_command"]={{"linear_mps",v},{"angular_rps",w}};
    s["transport_blocker"]=reason;s["transport_task_active"]=gate.active;
    if(!permitted) {
      s["safety_fresh"]=false;s["safety_reason"]=reason;
      if(!s.contains("output_blockers") || !s["output_blockers"].is_array()) s["output_blockers"]=Json::array();
      s["output_blockers"].push_back(reason);
    }
    s["lease_remaining_s"]=gate.active?std::max(0.,gate.lease_until-t):0.;
    if(!gate.active) {
      s["enabled"]=false;
      if(!terminal_state.empty()) {s["state"]=terminal_state;s["reason"]=terminal_reason;}
    }
    s["terminal_arrival"]=s.value("state","")=="arrived";
    m.data=s.dump();status->publish(m);
  }
  void fail(const std::string &why) {
    if(failed) return;
    failed=true;gate.revoke();gate.estop=true;ready=false;
    terminal_state="cancelled";terminal_reason="compute_transport_failure";
    RCLCPP_ERROR(get_logger(),"Follow transport stopped: %s",why.c_str());
    try { publish_command();publish_status(); } catch(const std::exception &e) {
      RCLCPP_ERROR(get_logger(),"final follow output failed: %s",e.what());
    }
    rclcpp::shutdown();
  }
};

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);int status=0;
  try { auto node=std::make_shared<FollowTransport>();rclcpp::spin(node);status=node->exit_code();node.reset(); }
  catch(const std::exception &e) { fprintf(stderr,"follow_transport: %s\n",e.what());if(rclcpp::ok())rclcpp::shutdown();return 1; }
  if(rclcpp::ok()) rclcpp::shutdown();
  return status;
}
