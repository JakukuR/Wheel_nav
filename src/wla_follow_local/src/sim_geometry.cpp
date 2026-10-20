// Real depth front-end only. Simulation planner equations are unchanged.
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <array>
#include "isolated_clusters.hpp"
namespace py = pybind11;
using Array=py::array_t<double,py::array::c_style|py::array::forcecast>;
py::array_t<double> deproject(const Array& depth,const Array& k,const Array& rot,
 const Array& translation,double lo,double hi,int stride) {
 if(depth.ndim()!=2||depth.size()>2000000||k.ndim()!=1||k.size()!=4||
    rot.ndim()!=2||rot.shape(0)!=3||rot.shape(1)!=3||translation.ndim()!=1||translation.size()!=3||
    stride<1||stride>16||!std::isfinite(lo)||!std::isfinite(hi)||lo<=0||hi<=lo||hi>10)
  throw std::invalid_argument("invalid_depth_geometry");
 const auto d=depth.unchecked<2>();const auto q=k.unchecked<1>();
 const auto r=rot.unchecked<2>();const auto t=translation.unchecked<1>();
 for(int i=0;i<4;i++)if(!std::isfinite(q(i)))throw std::invalid_argument("invalid_intrinsics");
 if(q(0)<=0||q(1)<=0)throw std::invalid_argument("invalid_focal_length");
 for(int i=0;i<3;i++){
  if(!std::isfinite(t(i)))throw std::invalid_argument("invalid_translation");
  for(int j=0;j<3;j++)if(!std::isfinite(r(i,j)))throw std::invalid_argument("invalid_rotation");
 }
 std::vector<std::array<double,3>> points;
 points.reserve(((depth.shape(0)+stride-1)/stride)*((depth.shape(1)+stride-1)/stride));
 {
  py::gil_scoped_release release;
  for(py::ssize_t v=0;v<depth.shape(0);v+=stride)for(py::ssize_t u=0;u<depth.shape(1);u+=stride){
   double z=d(v,u);if(!std::isfinite(z)||z<lo||z>hi)continue;
   double x=(u-q(2))*z/q(0),y=(v-q(3))*z/q(1);
   points.push_back({r(0,0)*x+r(0,1)*y+r(0,2)*z+t(0),
                     r(1,0)*x+r(1,1)*y+r(1,2)*z+t(1),
                     r(2,0)*x+r(2,1)*y+r(2,2)*z+t(2)});
  }
 }
 py::array_t<double> out({static_cast<py::ssize_t>(points.size()),py::ssize_t(3)});
 auto a=out.mutable_unchecked<2>();
 for(size_t i=0;i<points.size();i++)for(int j=0;j<3;j++)a(i,j)=points[i][j];
 return out;
}

// Same filtering policy as the local depth_to_points display. Runs in the
// follower process, with GIL released; no external cloud synchronization.
py::tuple filter_points(const Array & input) {
 if(input.ndim()!=2 || input.shape(1)!=3 || input.shape(0)>2000000)
  throw std::invalid_argument("invalid_filter_points");
 struct Point {float x,y,z;};
 std::vector<Point> clearing, obstacles;
 const auto a=input.unchecked<2>();
 {
  py::gil_scoped_release release;
  clearing.reserve(input.shape(0));
  for(py::ssize_t i=0;i<input.shape(0);++i) {
   // Reject nonfinite/out-of-contract inputs before integer cell conversion.
   if(!std::isfinite(a(i,0)) || !std::isfinite(a(i,1)) || !std::isfinite(a(i,2)) ||
      std::abs(a(i,0))>100. || std::abs(a(i,1))>100.) continue;
   Point point{static_cast<float>(a(i,0)),static_cast<float>(a(i,1)),static_cast<float>(a(i,2))};
   if(point.z>=-.08 && point.z<=.50) clearing.push_back(point);
  }
  using wla_r680_navigation::detail::xy_key;
  std::unordered_map<uint64_t,std::vector<float>> samples;
  samples.reserve(clearing.size()/8+1);
  for(const auto & point:clearing) {
   if(point.z<-.08 || point.z>.12) continue;
   const auto x=static_cast<int32_t>(std::floor(point.x/.20));
   const auto y=static_cast<int32_t>(std::floor(point.y/.20));
   for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy)
    samples[xy_key(x+dx,y+dy)].push_back(point.z);
  }
  std::unordered_map<uint64_t,float> ground;
  ground.reserve(samples.size());
  for(auto & entry:samples) {
   auto & zs=entry.second;
   if(zs.size()<5) continue;
   const auto index=static_cast<size_t>(.20*static_cast<double>(zs.size()-1));
   std::nth_element(zs.begin(),zs.begin()+index,zs.end());
   ground.emplace(entry.first,zs[index]);
  }
  std::vector<Point> candidates;
  candidates.reserve(clearing.size());
  for(const auto & point:clearing) {
   const auto x=static_cast<int32_t>(std::floor(point.x/.20));
   const auto y=static_cast<int32_t>(std::floor(point.y/.20));
   const auto it=ground.find(xy_key(x,y));
   if(it!=ground.end() ? point.z-it->second>=.05 : point.z>=.08)
    candidates.push_back(point);
  }
  obstacles=wla_r680_navigation::remove_isolated_clusters(candidates,.05,3);
 }
 auto pack=[](const std::vector<Point> & points) {
  py::array_t<double> out({static_cast<py::ssize_t>(points.size()),py::ssize_t(3)});
  auto dst=out.mutable_unchecked<2>();
  for(size_t i=0;i<points.size();++i){dst(i,0)=points[i].x;dst(i,1)=points[i].y;dst(i,2)=points[i].z;}
  return out;
 };
 return py::make_tuple(pack(obstacles),pack(clearing));
}
PYBIND11_MODULE(_wla_sim_geometry,m){m.def("deproject",&deproject);m.def("filter_points",&filter_points);}
