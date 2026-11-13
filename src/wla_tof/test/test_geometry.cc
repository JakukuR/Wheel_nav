#include "wla_tof/geometry.h"
#include "wla_tof/protocol.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace wla::tof;
void Require(bool c,const char* why) {if(!c) throw std::runtime_error(why);}
int main() {
 try {
  nlohmann::json model;std::ifstream(GEOMETRY_FIXTURE)>>model;RobotGeometry geometry(model);
  std::map<std::string,double> joints;for(const auto& j:model.at("joints")) joints.emplace(j.at("name"),0.);
  auto zero=geometry.evaluate(joints,Eigen::Quaterniond::Identity(),"both",.05);
  Require(zero.frames.size()==18,"all eight joints, seven sensors, IMU and two wheels");
  Require(zero.ground.available && zero.ground.offset>0,"wheel bottom defines floor");
  Require(std::abs(HeightAboveGround(zero.ground,-zero.ground.offset*Eigen::Vector3d::UnitZ()))<1e-10,"ground point zero height");
  Require(std::abs(HeightAboveGround(zero.ground,Eigen::Vector3d(0,0,.1-zero.ground.offset))-.1)<1e-10,"obstacle height");
  Require(!geometry.evaluate(joints,Eigen::Quaterniond::Identity(),"air",.05).ground.available,"airborne is not ground");
  Require(!geometry.evaluate(joints,Eigen::Quaterniond::Identity(),"unknown",.05).ground.available,"unknown is not ground");
  const Eigen::Quaterniond q(Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitY()));
  auto pitch=geometry.evaluate(joints,q,"both",.05);Require(pitch.ground.available,"pitch ground");
  Require((pitch.ground.normal-q.conjugate()*Eigen::Vector3d::UnitZ()).norm()<1e-12,"IMU changes plane normal");
  joints["knee_L_Joint"]=1.;auto bent=geometry.evaluate(joints,q,"both",1.);
  Require(std::abs(bent.ground.wheelHeights[0]-pitch.ground.wheelHeights[0])>.01,"live knee changes height");
  Require(std::abs(bent.ground.wheelHeights[1]-pitch.ground.wheelHeights[1])<1e-12,"left knee does not move right wheel");
  Require(!geometry.evaluate(joints,q,"both",.001).ground.available,"wheel height disagreement rejected");
  joints.erase("hip_L_Joint");bool rejected=false;try{(void)geometry.evaluate(joints,q,"both",.05);}catch(const std::exception&){rejected=true;}Require(rejected,"missing joint rejected");
  Frame f;f.targets[0]=f.targets[1]=f.targets[2]=1;f.status[0]=f.status[1]=f.status[2]=5;
  f.distance[0]=149;f.distance[1]=150;f.distance[2]=4001;
  auto ps=PreviewPoints(f,45.);Require(ps.size()==1 && ps[0].zone==1,"0.15m inclusive minimum and 4m maximum");
  std::cout<<"joint_ground_geometry: passed\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
