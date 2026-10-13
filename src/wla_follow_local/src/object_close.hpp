// Surface-clearance terminal control; selected target is never erased.
struct ObjectCloseResult {double v=0,w=0,gap=inf;bool valid=false,arrived=false;std::string reason="surface_unavailable";};
bool footprint_clear(const FloatArray& points,double v,double w,double front,double rear,double half_width,double margin,double horizon=1.) {
 auto p=points.unchecked<2>();
 int steps=std::max(20,int(std::ceil(horizon/.025)));
 for(int step=0;step<=steps;step++) {
  double t=horizon*step/steps,angle=w*t,c=std::cos(angle),s=std::sin(angle);
  double dx=std::abs(w)<1e-6?v*t:v*std::sin(angle)/w,dy=std::abs(w)<1e-6?0:v*(1-std::cos(angle))/w;
  for(py::ssize_t i=0;i<p.shape(0);i++) {
   double px=p(i,0),py_=p(i,1),z=p(i,2);
   if(!std::isfinite(px)||!std::isfinite(py_)||!std::isfinite(z)||!((z>=.01&&z<=.50)||z<-.10))continue;
   double x=c*(px-dx)+s*(py_-dy),y=-s*(px-dx)+c*(py_-dy);
   if(x>=-rear-margin&&x<=front+margin&&std::abs(y)<=half_width+margin)return false;
  }
 }return true;
}
ObjectCloseResult close_object(const FloatArray& points,const FloatArray& target,double yaw,double front,double rear,double width,double desired,double maximum,double gain,double measured_v,double measured_w,double reaction,double deceleration,double turn_limit) {
 if(points.ndim()!=2||points.shape(1)!=3||target.ndim()!=2||target.shape(1)!=3||
    points.shape(0)>2000000||target.shape(0)>2000000||!std::isfinite(yaw)||std::abs(yaw)>pi||
    !std::isfinite(maximum)||!std::isfinite(gain)||!std::isfinite(measured_v)||!std::isfinite(measured_w)||
    !std::isfinite(reaction)||!std::isfinite(deceleration)||!std::isfinite(turn_limit)||
    !(maximum>=0&&maximum<=.20&&gain>0&&gain<=2&&std::abs(measured_v)<=1.5&&std::abs(measured_w)<=2&&
      reaction>=.1&&reaction<=2&&deceleration>=.05&&deceleration<=2&&turn_limit>0&&turn_limit<=.45)||
    !(front>=.1&&front<=1&&rear>=.1&&rear<=1&&width>=.1&&width<=1&&desired>=.15&&desired<=.5))
  throw std::invalid_argument("invalid_object_close_configuration");
 auto p=target.unchecked<2>();ObjectCloseResult out;int count=0;
 py::gil_scoped_release release;
 for(py::ssize_t i=0;i<p.shape(0);i++) {
  double x=p(i,0),y=p(i,1),z=p(i,2);
  if(std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z)&&x>0&&z>=.01&&z<=2.5){out.gap=std::min(out.gap,x-front);count++;}
 }
 if(count<4)return out;
 out.valid=true;
 if(!footprint_clear(points,0,0,front,rear,width,.10)||out.gap<.10){out.reason="close_emergency_obstacle";return out;}
 double corridor=inf;auto all=points.unchecked<2>();
 for(py::ssize_t i=0;i<all.shape(0);i++)if(std::isfinite(all(i,0))&&std::isfinite(all(i,1))&&std::isfinite(all(i,2))&&all(i,0)>front&&std::abs(all(i,1))<=width+.03&&((all(i,2)>=.01&&all(i,2)<=.50)||all(i,2)<-.10))corridor=std::min(corridor,double(all(i,0))-front);
 bool foreground=corridor<out.gap-.04;
 out.arrived=std::abs(measured_v)<=.015&&std::abs(measured_w)<=.03&&std::abs(yaw)<=.08&&out.gap>=desired-.02&&out.gap<=desired+.02&&!foreground;
 if(out.arrived){out.reason="object_clearance_reached";return out;}
 // User requested smooth convergence through 0.05 m/s to zero, no velocity floor.
 double remaining=std::max(0.,std::min(out.gap,corridor)-desired);
 // Configured response delay/deceleration are assumptions pending chassis tests.
 double brake_limit=std::sqrt(std::pow(deceleration*reaction,2)+2*deceleration*remaining)-deceleration*reaction;
 out.v=std::min({maximum,gain*remaining,std::max(0.,brake_limit)});
 double actual=std::max(0.,measured_v);
 bool braking=actual>.015 && actual*reaction+actual*actual/(2*deceleration)>=remaining;
 if(braking)out.v=0;
 if(std::abs(yaw)>.10)out.v=0;
 out.w=std::abs(yaw)>.04?std::clamp(1.2*yaw,-turn_limit,turn_limit):0.;
 if(braking)out.w=0;
 out.reason=braking?"close_braking":foreground?"foreground_obstacle":"close_surface_control";
 // Retain the original 1 s sweep, extend if the braking envelope is longer.
 double horizon=std::max(1.,reaction+std::max(std::abs(measured_v),out.v)/deceleration);
 if(!footprint_clear(points,measured_v,measured_w,front,rear,width,.10,horizon)){
  out.v=out.w=0;out.reason="measured_motion_stop_margin";return out;
 }
 if(!footprint_clear(points,out.v,out.w,front,rear,width,.10,horizon)){
  out.v=0;if(!footprint_clear(points,0,out.w,front,rear,width,.10,horizon))out.w=0;
  out.reason="close_path_blocked";
 }
 return out;
}
