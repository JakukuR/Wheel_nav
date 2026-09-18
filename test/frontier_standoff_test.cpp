#include "explore/frontier_standoff.hpp"
#include <iostream>
#include <chrono>
int main()
{
  nav2_costmap_2d::Costmap2D map(120,120,.05,-3,-3,0);
  // A straight frontier. Candidate must retreat into known space and retain
  // a complete known scan disk; frontier centroid itself is unknown.
  for(unsigned int x=80;x<120;++x) for(unsigned int y=0;y<120;++y) map.setCost(x,y,255);
  geometry_msgs::msg::Point border,goal;border.x=1.025;border.y=0;
  if(!frontier_exploration::selectScanPose(map,0,0,0,border,goal,.63,{},true) ||
     !frontier_exploration::scanDiskKnown(map,goal.x,goal.y,.63) || goal.x>.35) {return 1;}
  unsigned int x,y;map.worldToMap(goal.x+.2,goal.y+.1,x,y);map.setCost(x,y,254);
  if(frontier_exploration::scanDiskKnown(map,goal.x,goal.y,.63)) {return 2;}
  map.setCost(x,y,255);
  if(frontier_exploration::scanDiskKnown(map,goal.x,goal.y,.63)) {return 3;}
  // Disconnected free area cannot provide a falsely reachable goal.
  for(unsigned int x=0;x<120;++x) for(unsigned int y=0;y<120;++y) map.setCost(x,y,255);
  if(frontier_exploration::selectScanPose(map,0,0,0,border,goal)) {return 4;}
  // Current-body unknown from camera near clip must not make all nearby
  // observed free cells disappear from the candidate search.
  for(unsigned int x=0;x<100;++x) for(unsigned int y=0;y<120;++y) map.setCost(x,y,0);
  map.worldToMap(0,0,x,y);map.setCost(x,y,255);border.x=2.025;
  if(!frontier_exploration::selectScanPose(map,0,0,0,border,goal) || map.getCost(x,y)!=255) {return 5;}
  const auto rejected=goal;
  if(!frontier_exploration::selectScanPose(map,0,0,0,border,goal,.63,
      [&](double gx,double gy) {return std::abs(gx-rejected.x)<.25 && std::abs(gy-rejected.y)<.25;}) ||
     (std::abs(goal.x-rejected.x)<.25 && std::abs(goal.y-rejected.y)<.25)) {return 6;}
  // An inside corner of an unknown region: the marker is not a valid goal,
  // but an observed position with room to turn must remain selectable.
  for(unsigned int x=0;x<120;++x) for(unsigned int y=0;y<120;++y) {
    map.setCost(x,y,(x>=80 && y>=80)?255:0);
  }
  border.x=1.025;border.y=1.025;
  if(!frontier_exploration::selectScanPose(map,0,0,0,border,goal) ||
     !frontier_exploration::scanDiskKnown(map,goal.x,goal.y,.63)) {return 7;}
  // Saved-house geometry around the bedroom door: the actual observation
  // rectangle has a 0.627m corner radius and must retain a known scan pose
  // between the coffee table, bed and wall.
  nav2_costmap_2d::Costmap2D house(180,140,.05,-1,-1,0);
  auto fill=[&](double x0,double x1,double y0,double y1,unsigned char cost) {
    for(double wx=x0+.025;wx<x1;wx+=.05) for(double wy=y0+.025;wy<y1;wy+=.05) {
      unsigned int mx,my;if(house.worldToMap(wx,wy,mx,my)) {house.setCost(mx,my,cost);}
    }
  };
  fill(-1,.7,3.45,3.55,254);fill(1.7,3.5,3.45,3.55,254);
  fill(1.1,1.9,1.725,2.275,254);fill(1.25,3.05,3.65,5.55,255);
  fill(-.75,.85,3.875,5.725,254);
  border.x=1.25;border.y=3.65;
  if(!frontier_exploration::selectScanPose(house,6.1,4.88,M_PI_2,border,goal,.63) ||
     !frontier_exploration::scanDiskKnown(house,goal.x,goal.y,.63)) {return 8;}
  // Cache lifetime is one immutable map cycle. Repeated candidates must not
  // re-evaluate disks, and a new cycle must observe newly inserted obstacles.
  frontier_exploration::ScanSearchContext cached(house,6.1,4.88,M_PI_2);
  double travel=0.;
  if(!frontier_exploration::selectScanPose(house,6.1,4.88,M_PI_2,border,goal,.63,{},false,&cached,&travel) || travel<=0) {return 9;}
  const auto checks=cached.disk_checks;
  for(int n=0;n<30;++n) {
    if(!frontier_exploration::selectScanPose(house,6.1,4.88,M_PI_2,border,goal,.63,{},false,&cached)) {return 10;}
  }
  if(cached.disk_checks!=checks) {return 11;}
  auto start=std::chrono::steady_clock::now();
  for(int n=0;n<100;++n) {frontier_exploration::selectScanPose(house,6.1,4.88,M_PI_2,border,goal,.63);}
  auto middle=std::chrono::steady_clock::now();
  for(int n=0;n<100;++n) {frontier_exploration::selectScanPose(house,6.1,4.88,M_PI_2,border,goal,.63,{},false,&cached);}
  auto end=std::chrono::steady_clock::now();
  std::cout<<"100 queries uncached_ms="<<std::chrono::duration<double,std::milli>(middle-start).count()
           <<" cached_ms="<<std::chrono::duration<double,std::milli>(end-middle).count()<<" disk_checks="<<checks<<"\n";
  house.worldToMap(goal.x,goal.y,x,y);house.setCost(x,y,254);
  frontier_exploration::ScanSearchContext updated(house,6.1,4.88,M_PI_2);
  if(frontier_exploration::selectScanPose(house,6.1,4.88,M_PI_2,border,goal,.63,{},true,&updated) &&
     !frontier_exploration::scanDiskKnown(house,goal.x,goal.y,.63)) {return 12;}
  std::cout<<"Standoff regression passed: known disk, interior obstacle/unknown, unreachable map\n";
}
