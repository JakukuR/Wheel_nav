#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <thread>
#include <stdexcept>
using namespace std::chrono_literals;
void check(bool x,const char * why){if(!x)throw std::runtime_error(why);}
class StaticFixture : public nav2_costmap_2d::Layer {
  bool initial=true;
public:
  void onInitialize() override{enabled_=true;current_=true;}
  bool isClearable() override{return false;}
  void reset() override{}
  void updateBounds(double,double,double,double * a,double * b,double * c,double * d)override {
    if(initial){*a=-2.5;*b=-2.5;*c=2.49;*d=2.49;initial=false;}
  }
  void updateCosts(nav2_costmap_2d::Costmap2D & m,int a,int b,int c,int d)override {
    for(int y=b;y<d;++y)for(int x=a;x<c;++x)m.setCost(x,y,0);
    unsigned int x,y;m.worldToMap(0,0,x,y);
    if(static_cast<int>(x)>=a && static_cast<int>(x)<c && static_cast<int>(y)>=b && static_cast<int>(y)<d)m.setCost(x,y,254);
  }
};
int main(int argc,char ** argv){
  rclcpp::init(argc,argv);
  auto costnode=std::make_shared<nav2_util::LifecycleNode>("test_timed_layer","",rclcpp::NodeOptions());
  costnode->declare_parameter("dynamic.persistence_s",0.3);
  costnode->declare_parameter("dynamic.confirmation_hits",2);
  auto publisher=std::make_shared<rclcpp::Node>("test_timed_layer_inputs");
  auto cloudpub=publisher->create_publisher<sensor_msgs::msg::PointCloud2>("/r680_nav/d455/points",10);
  auto healthpub=publisher->create_publisher<std_msgs::msg::Bool>("/r680_nav/localization_ready",10);
  nav2_costmap_2d::LayeredCostmap map("map",false,false);map.resizeMap(100,100,0.05,-2.5,-2.5);
  tf2_ros::Buffer buffer(costnode->get_clock());
  geometry_msgs::msg::TransformStamped transform;transform.header.frame_id="map";
  transform.child_frame_id="r680_mapping_floor";transform.transform.rotation.w=1;
  buffer.setTransform(transform,"fixture",true);
  auto group=costnode->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto fixed=std::make_shared<StaticFixture>();fixed->initialize(&map,"fixed",&buffer,costnode,group);map.addPlugin(fixed);
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader("nav2_costmap_2d","nav2_costmap_2d::Layer");
  auto dynamic=loader.createSharedInstance("wla_r680_navigation::TimedObstacleLayer");
  dynamic->initialize(&map,"dynamic",&buffer,costnode,group);map.addPlugin(dynamic);
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(costnode->get_node_base_interface());executor.add_node(publisher);
  auto spin=[&](int millis){for(int i=0;i<millis/10;++i){executor.spin_some();std::this_thread::sleep_for(10ms);}};
  auto health=[&](bool value){std_msgs::msg::Bool h;h.data=value;healthpub->publish(h);spin(80);};
  auto observe=[&](bool stale){sensor_msgs::msg::PointCloud2 c;c.header.frame_id="r680_mapping_floor";
    c.header.stamp=publisher->now()-rclcpp::Duration::from_seconds(stale?2:0);
    sensor_msgs::PointCloud2Modifier modifier(c);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float> x(c,"x"),y(c,"y"),z(c,"z");*x=1;*y=0;*z=0.2;
    cloudpub->publish(c);spin(80);map.updateMap(0,0,0);};
  auto cost=[&](double wx,double wy){unsigned int x,y;check(map.getCostmap()->worldToMap(wx,wy,x,y),"map bounds");return map.getCostmap()->getCost(x,y);};
  spin(200);health(true);observe(false);check(cost(1,0)==0,"single frame noise was marked");
  observe(false);check(cost(1,0)==254,"confirmed obstacle not in planner map");
  spin(400);map.updateMap(0,0,0);check(cost(1,0)==0,"expired obstacle not removed");
  check(cost(0,0)==254,"static obstacle erased by expiration");
  observe(true);observe(true);check(cost(1,0)==0,"stale cloud marked");
  observe(false);observe(false);check(cost(1,0)==254,"fresh cloud did not remark");
  health(false);map.updateMap(0,0,0);check(cost(1,0)==0,"old generation obstacles survived unhealthy localization");
  check(cost(0,0)==254,"static obstacle erased by reset");
  rclcpp::shutdown();
}
