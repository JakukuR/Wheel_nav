#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Transform.h>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

namespace wla_r680_navigation {
// Separate global planning hints: bounded persistence, never erase static costs.
class TimedObstacleLayer final : public nav2_costmap_2d::Layer {
  struct Cell {double x, y, first, last; int hits;};
  std::unordered_map<int64_t, Cell> cells_;
  std::vector<Cell> rendered_;
  std::mutex mutex_;
  double ttl_{2}, gap_{0.3}, freshness_{0.5}; int hits_{2};
  bool localization_ok_{false};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_;
  double seconds() {return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();}
public:
  void onInitialize() override {
    auto node=node_.lock(); if (!node) throw std::runtime_error("expired costmap node");
    declareParameter("topic", rclcpp::ParameterValue("/r680_nav/d455/points"));
    declareParameter("persistence_s", rclcpp::ParameterValue(2.0));
    declareParameter("confirmation_hits", rclcpp::ParameterValue(2));
    declareParameter("max_observation_gap_s", rclcpp::ParameterValue(0.3));
    declareParameter("source_timeout_s", rclcpp::ParameterValue(0.5));
    ttl_=node->get_parameter(name_+".persistence_s").as_double();
    hits_=node->get_parameter(name_+".confirmation_hits").as_int();
    gap_=node->get_parameter(name_+".max_observation_gap_s").as_double();
    freshness_=node->get_parameter(name_+".source_timeout_s").as_double();
    if (!std::isfinite(ttl_+gap_+freshness_) || ttl_<=0 || gap_<=0 || freshness_<=0 || hits_<1)
      throw std::runtime_error("invalid timed obstacle parameters");
    rclcpp::SubscriptionOptions options; options.callback_group=callback_group_;
    sub_=node->create_subscription<sensor_msgs::msg::PointCloud2>(
      node->get_parameter(name_+".topic").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {observe(*cloud);}, options);
    health_=node->create_subscription<std_msgs::msg::Bool>("/r680_nav/localization_ready",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        localization_ok_=msg->data; if (!localization_ok_) cells_.clear();
      }, options);
    enabled_=true; current_=true;
  }
  void observe(const sensor_msgs::msg::PointCloud2 & c) {
    auto node=node_.lock(); if (!node) return;
    const double age=(node->now()-rclcpp::Time(c.header.stamp)).seconds();
    // Input is already segmented and spatially filtered in the floor/base frame.
    if (age< -0.1 || age>freshness_ || c.header.frame_id!="r680_mapping_floor" ||
        c.height!=1 || c.is_bigendian || c.point_step==0 ||
        c.row_step!=c.width*c.point_step || c.data.size()!=c.row_step) return;
    for (const std::string field : {"x","y","z"}) {
      auto found=std::find_if(c.fields.begin(),c.fields.end(),[&](const auto & f){return f.name==field;});
      if (found==c.fields.end() || found->datatype!=sensor_msgs::msg::PointField::FLOAT32 ||
          found->count!=1 || found->offset+4>c.point_step) return;
    }
    try {
      const auto t=tf_->lookupTransform(layered_costmap_->getGlobalFrameID(),
        c.header.frame_id,rclcpp::Time(c.header.stamp));
      tf2::Transform transform(tf2::Quaternion(t.transform.rotation.x,t.transform.rotation.y,
        t.transform.rotation.z,t.transform.rotation.w),tf2::Vector3(t.transform.translation.x,
        t.transform.translation.y,t.transform.translation.z));
      std::lock_guard<std::mutex> lock(mutex_); if (!localization_ok_) return;
      const double now=seconds(), res=layered_costmap_->getCostmap()->getResolution();
      std::unordered_set<int64_t> seen;
      sensor_msgs::PointCloud2ConstIterator<float> x(c,"x"),y(c,"y"),z(c,"z");
      for (;x!=x.end();++x,++y,++z) {
        if (!std::isfinite(*x+*y+*z) || *z<0.04 || *z>0.50 || std::hypot(*x,*y)>4.5) continue;
        const auto p=transform*tf2::Vector3(*x,*y,*z);
        if (!std::isfinite(p.x()+p.y()) || std::abs(p.x()/res)>1e8 || std::abs(p.y()/res)>1e8) continue;
        const auto ix=static_cast<int32_t>(std::floor(p.x()/res));
        const auto iy=static_cast<int32_t>(std::floor(p.y()/res));
        const int64_t key=static_cast<int64_t>((static_cast<uint64_t>(static_cast<uint32_t>(ix))<<32)|
          static_cast<uint32_t>(iy));
        if (!seen.insert(key).second) continue;
        auto it=cells_.find(key);
        if (it==cells_.end()) {
          if (cells_.size()<30000) cells_.emplace(key,Cell{p.x(),p.y(),now,now,1});
        } else {
          auto & cell=it->second;
          cell.hits=(now-cell.last>gap_ && cell.hits<hits_) ? 1 : std::min(hits_,cell.hits+1);
          cell.x=p.x();cell.y=p.y();cell.last=now;
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(logger_,*clock_,3000,"dynamic planning observation rejected: %s",error.what());
    }
  }
  void updateBounds(double,double,double,double * minx,double * miny,double * maxx,double * maxy) override {
    std::lock_guard<std::mutex> lock(mutex_);
    const double margin=layered_costmap_->getCostmap()->getResolution();
    auto touch=[&](const Cell & c){*minx=std::min(*minx,c.x-margin);*miny=std::min(*miny,c.y-margin);
      *maxx=std::max(*maxx,c.x+margin);*maxy=std::max(*maxy,c.y+margin);};
    // Include previously drawn bounds so Nav2 reconstructs expired cells from static layer.
    for (const auto & c:rendered_) touch(c);
    rendered_.clear(); const double now=seconds();
    for (auto it=cells_.begin();it!=cells_.end();) {
      if (now-it->second.last>ttl_) {it=cells_.erase(it);continue;}
      if (it->second.hits>=hits_) {rendered_.push_back(it->second);touch(it->second);}
      ++it;
    }
  }
  void updateCosts(nav2_costmap_2d::Costmap2D & master,int minx,int miny,int maxx,int maxy) override {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & c:rendered_) {
      unsigned int x,y;
      if (master.worldToMap(c.x,c.y,x,y) && static_cast<int>(x)>=minx && static_cast<int>(x)<maxx &&
          static_cast<int>(y)>=miny && static_cast<int>(y)<maxy)
        master.setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  void reset() override {std::lock_guard<std::mutex> lock(mutex_);cells_.clear();current_=true;}
  bool isClearable() override {return true;}
};
}
PLUGINLIB_EXPORT_CLASS(wla_r680_navigation::TimedObstacleLayer,nav2_costmap_2d::Layer)
