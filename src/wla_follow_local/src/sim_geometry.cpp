// Real depth front-end only. Simulation planner equations are unchanged.
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <array>
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
PYBIND11_MODULE(_wla_sim_geometry,m){m.def("deproject",&deproject);}
