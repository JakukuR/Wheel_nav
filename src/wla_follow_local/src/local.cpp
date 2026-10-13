// Local flat-floor planning only. No Nav2, global pose, actuator publisher,
// terrain traversal guarantee or unobserved side/rear footprint certification.
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>
namespace py = pybind11;
using Point = std::pair<double,double>;
constexpr double pi=3.14159265358979323846;
constexpr double inf=std::numeric_limits<double>::infinity();
using FloatArray=py::array_t<float,py::array::c_style|py::array::forcecast>;
using MaskArray=py::array_t<uint8_t,py::array::c_style|py::array::forcecast>;
struct Scene {
 int n=0; double resolution=.1,radius=.7;
 std::vector<uint8_t> observed,occupied,free;
 int center() const {return n/2;}
 std::pair<int,int> cell(double x,double y) const {
  return {int(std::nearbyint(y/resolution))+center(),int(std::nearbyint(x/resolution))+center()};
 }
 Point point(std::pair<int,int> a) const {return {(a.second-center())*resolution,(a.first-center())*resolution};}
 bool clear(double x,double y) const {
  auto [i,j]=cell(x,y);return i>=0&&j>=0&&i<n&&j<n&&free[i*n+j];
 }
 bool segment_clear(Point a,Point b) const {
  int count=std::max(1,int(std::ceil(std::hypot(a.first-b.first,a.second-b.second)/(resolution*.4))));
  for(int t=0;t<=count;t++) if(!clear(a.first+(b.first-a.first)*t/count,a.second+(b.second-a.second)*t/count)) return false;
  return true;
 }
 bool command_clear(double v,double w,double horizon=.8) const {
  if(!std::isfinite(v)||!std::isfinite(w)||v<0||!std::isfinite(horizon)||horizon<0) return false;
  for(int i=0;i<17;i++) {
   double t=horizon*i/16.,x=std::abs(w)<1e-6?v*t:v*std::sin(w*t)/w;
   double y=std::abs(w)<1e-6?0.:v*(1-std::cos(w*t))/w;
   if(!clear(x,y))return false;
  }return true;
 }
 void inflate() {
  free=observed;int p=int(std::ceil(radius/resolution));
  for(int i=0;i<n;i++)for(int j=0;j<n;j++)if(occupied[i*n+j])
   for(int dy=-p;dy<=p;dy++)for(int dx=-p;dx<=p;dx++)
    if(dx*dx+dy*dy<=p*p&&i+dy>=0&&i+dy<n&&j+dx>=0&&j+dx<n)free[(i+dy)*n+j+dx]=0;
 }
 py::array_t<bool> mask(const std::vector<uint8_t>& data) const {
  py::array_t<bool> out({n,n});auto a=out.mutable_unchecked<2>();
  for(int i=0;i<n;i++)for(int j=0;j<n;j++)a(i,j)=data[i*n+j]!=0;
  return out;
 }
};
Scene inflate_scene(const MaskArray& known,const MaskArray& occupied,double resolution,double radius) {
 if(known.ndim()!=2||known.shape(0)!=known.shape(1)||known.shape(0)<3||known.shape(0)>201||known.shape(0)%2!=1||
    occupied.ndim()!=2||occupied.shape(0)!=known.shape(0)||occupied.shape(1)!=known.shape(1)||
    !std::isfinite(resolution)||resolution<.05||resolution>.15||!std::isfinite(radius)||radius<.3||radius>1.5)
  throw std::invalid_argument("invalid_local_scene_configuration");
 Scene s;s.n=known.shape(0);s.resolution=resolution;s.radius=radius;
 s.observed.assign(known.data(),known.data()+known.size());s.occupied.assign(occupied.data(),occupied.data()+occupied.size());
 {py::gil_scoped_release release;s.inflate();}return s;
}
void ray(Scene& s,int x0,int y0,int x1,int y1) {
 int dx=std::abs(x1-x0),sx=x0<x1?1:-1,dy=-std::abs(y1-y0),sy=y0<y1?1:-1,err=dx+dy;
 for(;;){
  if(x0>=0&&y0>=0&&x0<s.n&&y0<s.n)s.observed[y0*s.n+x0]=1;
  if(x0==x1&&y0==y1)break;
  int e=2*err;if(e>=dy){err+=dy;x0+=sx;}if(e<=dx){err+=dx;y0+=sy;}
 }
}
Scene scene_from_points(const FloatArray& points,Point origin,double resolution,double extent,double radius,double hmin,double hmax) {
 if(points.ndim()!=2||points.shape(1)!=3||points.shape(0)>2000000||!(resolution>=.05&&resolution<=.15&&extent>=1&&extent<=5&&radius>=.3&&radius<=1.5&&hmin>0&&hmin<hmax&&hmax<=1.5)||!std::isfinite(origin.first)||!std::isfinite(origin.second)||std::hypot(origin.first,origin.second)>extent)
  throw std::invalid_argument("invalid_local_scene_configuration");
 Scene s;s.n=2*int(std::ceil(extent/resolution))+1;s.radius=radius;s.resolution=resolution;
 s.observed.assign(s.n*s.n,0);s.occupied=s.observed;
 constexpr int bins=720;std::array<double,bins> far{},near;near.fill(inf);
 auto data=points.unchecked<2>();
 py::gil_scoped_release release;
 for(py::ssize_t i=0;i<data.shape(0);i++) {
  double x=data(i,0),y=data(i,1),z=data(i,2);
  if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z))continue;
  // Match float32 depth boundary semantics, including exactly 1 cm and 50 cm.
  bool obstacle=(z>=float(hmin)&&z<=float(hmax))||z<-.10;
  double d=std::hypot(x-origin.first,y-origin.second);
  int b=int(std::nearbyint((std::atan2(y-origin.second,x-origin.first)+pi)*bins/(2*pi)))%bins;
  if(z<=float(hmax))far[b]=std::max(far[b],d);
  if(obstacle){near[b]=std::min(near[b],d);auto [iy,ix]=s.cell(x,y);if(iy>=0&&ix>=0&&iy<s.n&&ix<s.n)s.occupied[iy*s.n+ix]=1;}
 }
 auto [sy,sx]=s.cell(origin.first,origin.second);
 for(int b=0;b<bins;b++) {
  double d=std::min({far[b],near[b],near[(b+1)%bins],near[(b+bins-1)%bins],extent*2});
  if(d<=0)continue;
  double angle=b*2*pi/bins-pi;auto [ey,ex]=s.cell(origin.first+d*std::cos(angle),origin.second+d*std::sin(angle));ray(s,sx,sy,ex,ey);
 }
 for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++)if(dx*dx+dy*dy<=1)s.observed[(s.center()+dy)*s.n+s.center()+dx]=1;
 s.inflate();return s;
}
struct Plan {std::vector<Point> path;std::string reason="waiting_for_local_scene";int side=0;};
struct Planner {
 int side=0;Plan last;
 void reset(){side=0;last=Plan();}
 Plan plan(const Scene& s,double yaw,double distance,double stand_off) {
  if(!std::isfinite(yaw)||!std::isfinite(distance)||!std::isfinite(stand_off)||distance<=0||stand_off<=0||!s.clear(0,0))return last={{},"footprint_blocked_or_unobserved",side};
  double hx=std::cos(yaw),hy=std::sin(yaw),tx=hx*distance,ty=hy*distance;
  double travel=std::max(0.,distance-stand_off),reach=std::min(travel,s.center()*s.resolution-s.radius-.2);
  Point goal={hx*reach,hy*reach};
  if(travel<=.10){side=0;return last={{{0,0}},"within_standoff",0};}
  if(s.segment_clear({0,0},goal)){side=0;return last={{{0,0},goal},"direct_local_path",0};}
  int start=s.center()*s.n+s.center(),best=-1;double best_score=inf;
  std::vector<double> costs(s.n*s.n,inf);std::vector<int> parents(s.n*s.n,-1);costs[start]=0;
  using Entry=std::pair<double,int>;std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;queue.push({0,start});
  std::array<Entry,3> options={Entry{inf,-1},Entry{inf,-1},Entry{inf,-1}};
  while(!queue.empty()) {
   auto [cost,cell]=queue.top();queue.pop();if(cost!=costs[cell])continue;
   int iy=cell/s.n,ix=cell%s.n;auto [x,y]=s.point({iy,ix});double d=std::hypot(x-tx,y-ty);
   if(distance-d>=std::min(.20,travel*.5)&&d>=stand_off-.05) {
    double score=6*std::hypot(x-goal.first,y-goal.second)+cost,lateral=y*hx-x*hy;
    int sd=lateral>.10?1:lateral<-.10?-1:0;
    if(score<options[sd+1].first)options[sd+1]={score,cell};
    if(score<best_score){best=cell;best_score=score;}
   }
   for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++) {
    int oy=iy+dy,ox=ix+dx;if((dx==0&&dy==0)||oy<0||ox<0||oy>=s.n||ox>=s.n)continue;
    int other=oy*s.n+ox;if(!s.free[other]||(dy&&dx&&(!s.free[(iy+dy)*s.n+ix]||!s.free[iy*s.n+ix+dx])))continue;
    auto [px,py]=s.point({oy,ox});if(std::hypot(px-tx,py-ty)<s.radius+.30)continue;
    double lateral=py*hx-px*hy,penalty=side&&lateral*side<-.10?.2:0.;
    double candidate=cost+s.resolution*std::hypot(dy,dx)*(1+penalty);
    if(candidate<costs[other]){costs[other]=candidate;parents[other]=cell;queue.push({candidate,other});}
   }
  }
  if(side&&options[side+1].second>=0&&options[side+1].first<=best_score+2)best=options[side+1].second;
  if(best<0)return last={{},"no_observed_local_detour",side};
  std::vector<Point> path;for(int c=best;c!=-1;c=parents[c]){path.push_back(s.point({c/s.n,c%s.n}));if(c==start)break;}
  std::reverse(path.begin(),path.end());double peak=0;
  for(auto [x,y]:path){double l=y*hx-x*hy;if(std::abs(l)>std::abs(peak))peak=l;}
  if(std::abs(peak)>.10)side=peak>0?1:-1;
  return last={path,"local_detour",side};
 }
 Point command(const Scene& s,double yaw,double distance,double stand_off,double max_v,double max_w) {
  if(!(max_v>0&&max_v<=.2&&max_w>0&&max_w<=.45))throw std::invalid_argument("invalid_control_limits");
  auto p=plan(s,yaw,distance,stand_off);if(p.path.empty())return {0,0};
  if(p.path.size()==1)return {0,std::abs(yaw)>.08?std::clamp(1.2*yaw,-max_w,max_w):0};
  Point carrot=p.path[1];for(size_t i=1;i<p.path.size();i++){carrot=p.path[i];if(std::hypot(carrot.first,carrot.second)>=.45)break;}
  double angle=std::atan2(carrot.second,carrot.first),w=std::clamp(1.4*angle,-max_w,max_w);
  double v=std::min(max_v,.45*std::max(0.,distance-stand_off))*std::max(0.,std::cos(angle));
  if(std::abs(angle)>=.55)v=0;
  if(!s.command_clear(v,w)){v=0;if(!s.command_clear(v,w))w=0;}return {v,w};
 }
};
PYBIND11_MODULE(_wla_follow_local,m) {
 py::class_<Scene>(m,"LocalScene")
  .def_property_readonly("observed",[](const Scene&s){return s.mask(s.observed);})
  .def_property_readonly("occupied",[](const Scene&s){return s.mask(s.occupied);})
  .def_property_readonly("free",[](const Scene&s){return s.mask(s.free);})
  .def_readonly("resolution",&Scene::resolution).def_readonly("radius",&Scene::radius)
  .def_property_readonly("center",&Scene::center).def("cell",&Scene::cell).def("point",&Scene::point)
  .def("clear",&Scene::clear).def("segment_clear",&Scene::segment_clear)
  .def("command_clear",&Scene::command_clear,py::arg("v"),py::arg("w"),py::arg("horizon")=.8);
 py::class_<Plan>(m,"LocalPlan").def_readonly("path",&Plan::path).def_readonly("reason",&Plan::reason).def_readonly("side",&Plan::side);
 py::class_<Planner>(m,"LocalAvoidance").def(py::init<>()).def("reset",&Planner::reset)
  .def_readonly("side",&Planner::side).def_readonly("last",&Planner::last)
  .def("plan",&Planner::plan,py::call_guard<py::gil_scoped_release>())
  .def("command",&Planner::command,py::call_guard<py::gil_scoped_release>());
 m.def("scene_from_points",&scene_from_points,py::arg("points"),py::arg("origin")=Point{0,0},py::kw_only(),py::arg("resolution")=.10,py::arg("extent")=3.,py::arg("radius")=.70,py::arg("height_min")=.01,py::arg("height_max")=.50);
 m.def("inflate_scene",&inflate_scene,py::arg("observed"),py::arg("occupied"),py::arg("resolution"),py::arg("radius"));
}
