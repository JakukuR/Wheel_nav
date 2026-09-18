// Local explore_lite policy. No simulator truth or costmap modifications.
#pragma once
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <queue>
#include <vector>
#include <cmath>
#include <limits>
#include <functional>
#include <memory>
#include <algorithm>

namespace frontier_exploration
{
inline bool scanRectangleKnown(const nav2_costmap_2d::Costmap2D & map,
  double x,double y,double heading)
{
  const double res=map.getResolution(),margin=res/std::sqrt(2.0);
  const double c=std::cos(heading),s=std::sin(heading);
  const double radius=std::hypot(.57+margin,.26+margin);
  const int ix=std::floor((x-map.getOriginX())/res),iy=std::floor((y-map.getOriginY())/res);
  const int cells=std::ceil(radius/res)+1;
  for(int mx=ix-cells;mx<=ix+cells;++mx) for(int my=iy-cells;my<=iy+cells;++my) {
    const double dx=map.getOriginX()+(mx+.5)*res-x,dy=map.getOriginY()+(my+.5)*res-y;
    const double bx=c*dx+s*dy,by=-s*dx+c*dy;
    if(bx<-.27-margin || bx>.57+margin || std::abs(by)>.26+margin) {continue;}
    if(mx<0 || my<0 || mx>=static_cast<int>(map.getSizeInCellsX()) ||
       my>=static_cast<int>(map.getSizeInCellsY()) || map.getCost(mx,my)>=254) {return false;}
  }
  return true;
}
inline bool scanDiskKnown(const nav2_costmap_2d::Costmap2D & map,
  double x,double y,double radius)
{
  const double r=radius+map.getResolution()/std::sqrt(2.0);
  const int lo_x=std::floor((x-r-map.getOriginX())/map.getResolution());
  const int hi_x=std::floor((x+r-map.getOriginX())/map.getResolution());
  const int lo_y=std::floor((y-r-map.getOriginY())/map.getResolution());
  const int hi_y=std::floor((y+r-map.getOriginY())/map.getResolution());
  for(int mx=lo_x;mx<=hi_x;++mx) for(int my=lo_y;my<=hi_y;++my) {
    const double wx=map.getOriginX()+(mx+.5)*map.getResolution();
    const double wy=map.getOriginY()+(my+.5)*map.getResolution();
    if(std::hypot(wx-x,wy-y)>r) {continue;}
    if(mx<0 || my<0 || mx>=static_cast<int>(map.getSizeInCellsX()) ||
       my>=static_cast<int>(map.getSizeInCellsY()) || map.getCost(mx,my)>=254) {return false;}
  }
  return true;
}
// Lifetime is exactly one makePlan call: map changes never reuse old reachability.
struct ScanSearchContext
{
  std::vector<int> distance;
  std::vector<signed char> disk;
  size_t disk_checks=0;
  double disk_radius=-1.;
  ScanSearchContext(const nav2_costmap_2d::Costmap2D & map,
                    double robot_x,double robot_y,double heading)
  {
  unsigned int sx,sy;
  if(!map.worldToMap(robot_x,robot_y,sx,sy) ||
     map.getSizeInCellsX()*map.getSizeInCellsY()>100000) {return;}
  const int width=map.getSizeInCellsX(),height=map.getSizeInCellsY();
  if(map.getCost(sx,sy)==255) {
    // The forward RGB-D camera leaves the current body/near-clip area unknown.
    // Seed the candidate search from an ACTUALLY OBSERVED nearby forward cell;
    // do not paint the gap free or infer that the intervening motion is safe.
    // The planner, controller and unchanged outlet gate own the actual motion.
    double nearest=.65;
    bool found=false;
    const int initial_x=sx,initial_y=sy;
    for(int y=std::max(0,initial_y-14);y<std::min(height,initial_y+15);++y) {
      for(int x=std::max(0,initial_x-14);x<std::min(width,initial_x+15);++x) {
        if(map.getCost(x,y)>=253) {continue;}
        double wx,wy;map.mapToWorld(x,y,wx,wy);
        const double dx=wx-robot_x,dy=wy-robot_y,d=std::hypot(dx,dy);
        if(d<nearest && std::cos(heading)*dx+std::sin(heading)*dy>0 &&
           std::abs(-std::sin(heading)*dx+std::cos(heading)*dy)<.3) {
          sx=x;sy=y;nearest=d;found=true;
        }
      }
    }
    if(!found) {return;}
  }
  if(map.getCost(sx,sy)>=253) {return;}
  distance.assign(width*height,-1);
  std::queue<int> queue;queue.push(sy*width+sx);distance[sy*width+sx]=0;
  while(!queue.empty()) {
    const int idx=queue.front();queue.pop();const int x=idx%width,y=idx/width;
    constexpr int moves[4][2]={{-1,0},{1,0},{0,-1},{0,1}};
    for(const auto & delta:moves) {
      const int nx=x+delta[0],ny=y+delta[1];
      if(nx>=0 && ny>=0 && nx<width && ny<height && distance[ny*width+nx]<0 && map.getCost(nx,ny)<253) {
        distance[ny*width+nx]=distance[idx]+1;queue.push(ny*width+nx);
      }
    }
  }
    disk.assign(distance.size(),-1);
  }
};
inline bool selectScanPose(const nav2_costmap_2d::Costmap2D & map,
  double robot_x,double robot_y,double heading,const geometry_msgs::msg::Point & frontier,
  geometry_msgs::msg::Point & goal,double radius=.63,
  const std::function<bool(double,double)> & blocked={}, bool require_full_scan=false,
  ScanSearchContext * shared=nullptr, double * path_distance=nullptr)
{
  std::unique_ptr<ScanSearchContext> owned;
  if(!shared) {owned=std::make_unique<ScanSearchContext>(map,robot_x,robot_y,heading);shared=owned.get();}
  if(shared->distance.empty()) {return false;}
  if(shared->disk_radius!=radius) {
    std::fill(shared->disk.begin(),shared->disk.end(),-1);
    shared->disk_radius=radius;
  }
  const int width=map.getSizeInCellsX(),height=map.getSizeInCellsY();
  const auto & distance=shared->distance;
  double best=std::numeric_limits<double>::infinity();
  const int cx=std::floor((frontier.x-map.getOriginX())/map.getResolution());
  const int cy=std::floor((frontier.y-map.getOriginY())/map.getResolution());
  const int reach=std::ceil(1.4/map.getResolution())+1;
  for(int y=std::max(0,cy-reach);y<std::min(height,cy+reach+1);++y) {
    for(int x=std::max(0,cx-reach);x<std::min(width,cx+reach+1);++x) {
    const int idx=y*width+x;
    if(distance[idx]<0) {continue;}
    double wx,wy;map.mapToWorld(x,y,wx,wy);
    const double border_distance=std::hypot(wx-frontier.x,wy-frontier.y);
    if(border_distance>=radius+.05 && border_distance<=1.4 && map.getCost(x,y)<253 &&
       (!blocked || !blocked(wx,wy))) {
      const double bearing=std::atan2(wy-robot_y,wx-robot_x);
      const double turn=std::abs(std::atan2(std::sin(bearing-heading),std::cos(bearing-heading)));
      const double goal_heading=std::atan2(frontier.y-wy,frontier.x-wx);
      const double final_turn=std::atan2(std::sin(goal_heading-bearing),std::cos(goal_heading-bearing));
      const double displacement=std::hypot(wx-robot_x,wy-robot_y);
      const double scan_turn=std::abs(std::atan2(std::sin(goal_heading-heading),std::cos(goal_heading-heading)));
      const double score=.5*distance[idx]*map.getResolution()+.35*turn+.35*std::abs(final_turn)+1.2*border_distance;
      bool observed=false;
      if(score<best && (displacement>.25 || scan_turn>.35)) {
        if(shared->disk[idx]<0) {
          shared->disk[idx]=scanDiskKnown(map,wx,wy,radius)?1:0;
          ++shared->disk_checks;
        }
        observed=shared->disk[idx]==1;
        // A forward approach with a small final turn does not require a
        // complete 360-degree disk. Verify every intermediate orientation;
        // larger turns still require the fully observed scan disk.
        if(!require_full_scan && !observed && turn<.8 && std::abs(final_turn)<.7) {
          observed=true;
          for(int j=0;j<=14;++j) {
            if(!scanRectangleKnown(map,wx,wy,bearing+final_turn*j/14)) {observed=false;break;}
          }
        }
      }
      if(observed) {
        // Camera observation ray must not go through a known wall or an
        // unrelated unknown band before reaching the selected frontier.
        const int samples=std::ceil(border_distance/(map.getResolution()/2));
        bool visible=true;
        for(int j=1;j<samples-2;++j) {
          unsigned int mx,my;const double t=static_cast<double>(j)/samples;
          if(!map.worldToMap(wx+t*(frontier.x-wx),wy+t*(frontier.y-wy),mx,my) ||
             map.getCost(mx,my)>=254) {visible=false;break;}
        }
        if(visible) {
          best=score;goal.x=wx;goal.y=wy;goal.z=0;
          if(path_distance) {*path_distance=distance[idx]*map.getResolution();}
        }
      }
    }
    }
  }
  return std::isfinite(best);
}
}
