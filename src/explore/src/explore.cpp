/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>
#include <explore/frontier_standoff.hpp>

#include <thread>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  progress_timeout_ = timeout;
  scan_after_goal_=declare_parameter<bool>("scan_after_goal", false);
  scan_timeout_=declare_parameter<double>("scan_timeout_s", 60.0);
  if(!std::isfinite(scan_timeout_) || scan_timeout_<25.) {
    throw std::invalid_argument("scan_timeout_s must allow a full turn at 0.30rad/s");
  }
  spin_client_=rclcpp_action::create_client<nav2_msgs::action::Spin>(this,"spin");
  scan_timer_=create_wall_timer(std::chrono::milliseconds(100),[this]() {scanTick();});
  guard_subscription_=this->create_subscription<std_msgs::msg::Bool>(
    "/home_sim/motion_guard_blocked",10,[this](std_msgs::msg::Bool::SharedPtr msg) {
      if(scan_active_) {
        if(!msg->data) {guard_blocked_=false;return;}
        const auto now=std::chrono::steady_clock::now();
        if(!guard_blocked_) {guard_blocked_since_=now;guard_blocked_=true;}
        if(std::chrono::duration<double>(now-guard_blocked_since_).count()>2.5) {
          finishScan(false,"scan_blocked");
        }
        return;
      }
      if(!msg->data || !goal_active_ || !guard_goal_handle_) {guard_blocked_=false;return;}
      const auto now=std::chrono::steady_clock::now();
      if(!guard_blocked_) {guard_blocked_since_=now;guard_blocked_=true;return;}
      if(std::chrono::duration<double>(now-guard_blocked_since_).count()>2.5) {
        RCLCPP_WARN(logger_, "Motion guard blocked own goal for 2.5s; cancel UUID and select another scan pose");
        frontier_blacklist_.push_back(prev_goal_);
        move_base_client_->async_cancel_goal(guard_goal_handle_);
        goal_cancel_pending_=true;guard_blocked_=false;
      }
    });

  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // Start exploration right away
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
  makePlan();
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.5);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.5);
  const auto green = std_msgs::msg::ColorRGBA().set__g(1.0).set__a(0.5);

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // m.lifetime defaults to 0, means lives forever
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  if(scan_active_) {return;}
  // find frontiers
  geometry_msgs::msg::Pose pose;
  try {
    pose = costmap_client_.getRobotPose();
  } catch (const tf2::TransformException& error) {
    RCLCPP_WARN(logger_, "Skipping frontier plan: %s", error.what());
    return;  // Never substitute the map origin for an unavailable robot pose.
  }
  // Keep an accepted scan goal stable while Nav2 replans its path. Grid-cell
  // jitter must not preempt every 2s or reset heading/progress controllers.
  if(goal_active_) {
    if(goal_cancel_pending_) {return;}
    const double remaining=std::hypot(prev_goal_.x-pose.position.x,prev_goal_.y-pose.position.y);
    const double heading=std::atan2(2*pose.orientation.w*pose.orientation.z,1-2*pose.orientation.z*pose.orientation.z);
    const double turn=std::abs(std::atan2(std::sin(heading-progress_yaw_),std::cos(heading-progress_yaw_)));
    if(remaining<prev_distance_-.10 || turn>.25) {
      prev_distance_=remaining;progress_yaw_=heading;last_progress_=this->now();
    }
    if(this->now()-last_progress_<=tf2::durationFromSec(progress_timeout_)) {return;}
    RCLCPP_WARN(logger_, "Scan goal made less than 0.10m progress in 30s; cancel UUID and select another pose");
    frontier_blacklist_.push_back(prev_goal_);
    if(guard_goal_handle_) {move_base_client_->async_cancel_goal(guard_goal_handle_);}
    goal_cancel_pending_=true;guard_blocked_=false;return;
  }
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }

  // Replace unknown frontier centroids with reachable, observed scan poses.
  const auto & q = pose.orientation;
  const double robot_heading = std::atan2(2*q.w*q.z,1-2*q.z*q.z);
  auto * map = costmap_client_.getCostmap();
  frontier_exploration::ScanSearchContext search_context(*map,pose.position.x,pose.position.y,robot_heading);
  frontiers.erase(std::remove_if(frontiers.begin(),frontiers.end(),
    [&](frontier_exploration::Frontier & f) {
      geometry_msgs::msg::Point scan;
      double travel=0.;
      bool selected=frontier_exploration::selectScanPose(*map,pose.position.x,pose.position.y,
                                                       robot_heading,f.middle,scan,.63,
          [this](double x,double y) {geometry_msgs::msg::Point p;p.x=x;p.y=y;return goalOnBlacklist(p);}, scan_after_goal_, &search_context, &travel);
      // A U-shaped frontier's nearest point or centroid may be unusable.
      // Try representative actual boundary points instead of abandoning it.
      if(!selected) {
        const size_t step=std::max<size_t>(1,f.points.size()/16);
        for(size_t i=0;i<f.points.size();i+=step) {
          if(frontier_exploration::selectScanPose(*map,pose.position.x,pose.position.y,
                                                robot_heading,f.points[i],scan,.63,
              [this](double x,double y) {geometry_msgs::msg::Point p;p.x=x;p.y=y;return goalOnBlacklist(p);}, scan_after_goal_, &search_context, &travel)) {
            f.middle=f.points[i];selected=true;break;
          }
        }
      }
      if(!selected) {return true;}
      f.centroid=scan;
      f.min_distance=std::hypot(scan.x-pose.position.x,scan.y-pose.position.y);
      const double bearing=std::atan2(scan.y-pose.position.y,scan.x-pose.position.x);
      const double final_heading=std::atan2(f.middle.y-scan.y,f.middle.x-scan.x);
      const double turn=std::abs(std::atan2(std::sin(final_heading-bearing),std::cos(final_heading-bearing)));
      f.cost=potential_scale_*travel + .35*turn - gain_scale_*f.size*map->getResolution();
      return false;
    }),frontiers.end());
  std::sort(frontiers.begin(),frontiers.end(),[](const auto & a,const auto & b) {return a.cost<b.cost;});
  if(frontiers.empty()) {
    RCLCPP_WARN(logger_, "No reachable observed scan pose; stopping rather than targeting unknown centroid");
    auto msg=explore_lite_msgs::msg::ExploreStatus();
    msg.status="exploration_blocked";
    status_pub_->publish(msg);exploring_timer_->cancel();return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  if (frontier == frontiers.end()) {
    RCLCPP_WARN(logger_, "All scan poses blocked; exploration remains incomplete.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = "exploration_blocked";
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }
  geometry_msgs::msg::Point target_position = frontier->centroid;
  // Hold small scan-pose changes while the same goal is active. Frontier
  // planning runs every 2s; do not keep resetting Nav2 on grid-cell jitter.
  unsigned int gx,gy;
  if(goal_active_ && std::hypot(target_position.x-prev_goal_.x,target_position.y-prev_goal_.y)<.35 &&
     map->worldToMap(prev_goal_.x,prev_goal_.y,gx,gy) && map->getCost(gx,gy)<253 &&
     frontier_exploration::scanDiskKnown(*map,prev_goal_.x,prev_goal_.y,.63)) {
    target_position=prev_goal_;
  }
  frontier->min_distance=std::hypot(target_position.x-pose.position.x,target_position.y-pose.position.y);

  // time out if we are not making any progress
  bool same_goal = same_point(prev_goal_, target_position);

  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = this->now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  if (goal_active_ &&
      (this->now() - last_progress_ >
       tf2::durationFromSec(progress_timeout_)) &&
      !resuming_) {
    frontier_blacklist_.push_back(target_position);
    RCLCPP_DEBUG(logger_, "Adding current goal to black list");
    makePlan();
    return;
  }

  // ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal && goal_active_) {
    return;
  }

  RCLCPP_DEBUG(logger_, "Sending goal to move base nav2");

  // send goal to move_base if we have something new to pursue
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  // A forward RGB-D sensor should face the selected frontier on arrival,
  // rather than always ending with a world +X heading.
  const double heading = std::atan2(frontier->middle.y - target_position.y,
                                    frontier->middle.x - target_position.x);
  goal.pose.pose.orientation.z = std::sin(heading * 0.5);
  goal.pose.pose.orientation.w = std::cos(heading * 0.5);
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  goal_active_ = true;goal_cancel_pending_=false;
  progress_yaw_=robot_heading;last_progress_=this->now();
  publishStatus("navigating_to_scan_pose");
  guard_goal_handle_.reset();guard_blocked_=false;
  const auto generation=++scan_goal_generation_;
  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
      [this,generation](const NavigationGoalHandle::SharedPtr& goal_handle) {
        if(generation!=scan_goal_generation_) {
          if(goal_handle) {move_base_client_->async_cancel_goal(goal_handle);}
          return;
        }
        if (!goal_handle) {
          RCLCPP_ERROR(logger_, "Goal was REJECTED by the action server");
          goal_active_ = false;
        } else {
          active_goal_id_ = goal_handle->get_goal_id();
          guard_goal_handle_=goal_handle;
          if(goal_cancel_pending_) {move_base_client_->async_cancel_goal(goal_handle);}
          guard_blocked_=false;
          RCLCPP_DEBUG(logger_, "Goal ACCEPTED, uuid: %s",
            rclcpp_action::to_string(active_goal_id_).c_str());
        }
      };

  send_goal_options.result_callback =
      [this,
       target_position,generation](const NavigationGoalHandle::WrappedResult& result) {
        if(generation!=scan_goal_generation_) {return;}
        reachedGoal(result, target_position);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::beginScan()
{
  if(!spin_client_->action_server_is_ready()) {publishStatus("scan_server_unavailable");return;}
  // Verify observed rotation room again at the actual arrival pose.
  try {
    const auto p=costmap_client_.getRobotPose();
    if(!frontier_exploration::scanDiskKnown(*costmap_client_.getCostmap(),p.position.x,p.position.y,.63)) {
      publishStatus("scan_space_unavailable");return;
    }
    const auto tf=tf_buffer_.lookupTransform("odom",robot_base_frame_,tf2::TimePointZero);
    const auto & q=tf.transform.rotation;
    scan_previous_yaw_=std::atan2(2*q.w*q.z,1-2*q.z*q.z);
    scan_last_stamp_=rclcpp::Time(tf.header.stamp).seconds();
    if(this->now().seconds()-scan_last_stamp_>.5) {publishStatus("scan_stale_pose");return;}
  } catch(const tf2::TransformException &) {publishStatus("scan_stale_pose");return;}
  scan_active_=true;scan_cancel_pending_=false;scan_accumulated_=0.;guard_blocked_=false;
  scan_started_=std::chrono::steady_clock::now();
  const auto generation=++scan_generation_;
  publishStatus("scanning_360");
  nav2_msgs::action::Spin::Goal goal;
  goal.target_yaw=2*M_PI;goal.time_allowance.sec=static_cast<int>(scan_timeout_);
  rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions options;
  options.goal_response_callback=[this,generation](const SpinHandle::SharedPtr & handle) {
    if(generation!=scan_generation_) {if(handle) {spin_client_->async_cancel_goal(handle);}return;}
    if(!handle) {finishScan(false,"scan_rejected",true);return;}
    spin_handle_=handle;
    if(scan_cancel_pending_) {spin_client_->async_cancel_goal(handle);}
  };
  options.result_callback=[this,generation](const SpinHandle::WrappedResult & result) {
    if(generation!=scan_generation_) {return;}
    scanTick();
    if(!scan_active_) {return;}
    const bool complete=!scan_cancel_pending_ && result.code==rclcpp_action::ResultCode::SUCCEEDED && scan_accumulated_>=2*M_PI-.15;
    finishScan(complete,complete?"scan_complete":"scan_incomplete",true);
  };
  spin_client_->async_send_goal(goal,options);
}

void Explore::scanTick()
{
  if(!scan_active_) {return;}
  if(std::chrono::duration<double>(std::chrono::steady_clock::now()-scan_started_).count()>scan_timeout_*2) {
    finishScan(false,"scan_wall_timeout");return;
  }
  try {
    const auto tf=tf_buffer_.lookupTransform("odom",robot_base_frame_,tf2::TimePointZero);
    const double stamp=rclcpp::Time(tf.header.stamp).seconds();
    const double age=this->now().seconds()-stamp;
    if(age<0 || age>.5) {finishScan(false,"scan_stale_pose");return;}
    if(stamp<=scan_last_stamp_) {return;}
    const auto & q=tf.transform.rotation;
    const double current=std::atan2(2*q.w*q.z,1-2*q.z*q.z);
    const double delta=std::atan2(std::sin(current-scan_previous_yaw_),std::cos(current-scan_previous_yaw_));
    if(std::abs(delta)>.6*(stamp-scan_last_stamp_)+.05) {finishScan(false,"scan_pose_jump");return;}
    scan_accumulated_+=delta;scan_previous_yaw_=current;scan_last_stamp_=stamp;
  } catch(const tf2::TransformException &) {finishScan(false,"scan_stale_pose");}
}

void Explore::finishScan(bool success,const std::string & reason,bool terminal)
{
  if(!scan_active_) {return;}
  if(!success && !terminal) {
    if(!scan_cancel_pending_) {
      scan_cancel_pending_=true;
      if(spin_handle_) {spin_client_->async_cancel_goal(spin_handle_);}
      publishStatus(reason);
    }
    return;  // Keep navigation ownership until the spin action is terminal.
  }
  scan_active_=false;++scan_generation_;
  if(!success && spin_handle_) {spin_client_->async_cancel_goal(spin_handle_);}
  spin_handle_.reset();guard_blocked_=false;
  RCLCPP_INFO(logger_,"360 scan result=%s measured_yaw=%.3f",reason.c_str(),scan_accumulated_);
  publishStatus(reason);
  // The planning timer selects the next viewpoint only after the scan ends.
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN;
  status_pub_->publish(status_msg);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
          status_pub_->publish(status_msg);
          RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
        }
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}
bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal) {
  // discard stale callbacks from previously preempted goals
  if (result.goal_id != active_goal_id_) {
    return;
  }

  goal_active_ = false;goal_cancel_pending_=false;guard_goal_handle_.reset();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful");
      frontier_blacklist_.push_back(frontier_goal); // observed this scan pose; try another viewpoint
      last_progress_ = this->now();
      prev_distance_ = 0;
      if(scan_after_goal_) {
        try {
          const auto actual=costmap_client_.getRobotPose();
          if(std::hypot(actual.position.x-frontier_goal.x,actual.position.y-frontier_goal.y)>.10) {
            publishStatus("arrival_verification_failed");return;
          }
        } catch(const tf2::TransformException &) {publishStatus("arrival_verification_failed");return;}
        beginScan();return;
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
#ifdef NAV2_RESULT_HAS_ERROR_CODE
      if (result.result && result.result->error_code != 0) {
        RCLCPP_DEBUG(logger_, "Goal aborted with error_code=%d (%s) — blacklisting frontier",
                     result.result->error_code,
                     result.result->error_msg.c_str());
        frontier_blacklist_.push_back(frontier_goal);
      } else {
        RCLCPP_DEBUG(logger_, "Goal aborted with error_code=0 — likely a preemption, not blacklisting");
      }
#else
      // Humble: no error_code field, blacklist unconditionally on abort
      RCLCPP_DEBUG(logger_, "Goal aborted — blacklisting frontier");
      frontier_blacklist_.push_back(frontier_goal);
#endif
      // If it was aborted probably because we've found another frontier goal,
      // so just return and don't make plan again
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");

  goal_active_ = false;
  ++scan_goal_generation_;
  if(scan_active_) {finishScan(false,"scan_canceled");}
  // Only publish paused status if manually stopped (not finished exploring)
  if (!finished_exploring) {
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    status_pub_->publish(status_msg);
  }

  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  resuming_ = true;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS;
  status_pub_->publish(status_msg);
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
