#include <cassert>
#include <cmath>
#include "wla_r680_navigation/degraded_odometry.hpp"
using namespace wla_r680_navigation;
int main() {
  auto p=integrateWheelGyro({},0.2,0,0.05);
  assert(std::abs(p.x-0.01)<1e-10 && p.y==0);
  p={}; for(int i=0;i<20;++i) p=integrateWheelGyro(p,0.2,0.5,0.05);
  assert(std::abs(p.x-0.4*std::sin(0.5))<1e-10);
  assert(std::abs(p.y-0.4*(1-std::cos(0.5)))<1e-10);
  p=integrateWheelGyro({},-0.2,0,0.05); assert(p.x<0);
  DegradedBudget b;
  for(int i=0;i<19;++i) assert(b.advance(0.1,0.2,0.1,2,0.3,0.8));
  assert(!b.advance(0.1,0.2,0.1,2,0.3,0.8));
  b={}; assert(!b.advance(4,0,0.1,2,0.3,0.8));
  b={}; assert(!b.advance(0,9,0.1,2,0.3,0.8));
  b={}; assert(b.advance(0,0,0.1,2,0.3,0.8)); assert(b.positionSigma()>0.02);
  assert(consistentVisual({0.01,0.02,0.02},{},0.12,0.15));
  assert(!consistentVisual({1,0,0},{},0.12,0.15));
  assert(!consistentVisual({0,0,1},{},0.12,0.15));
  assert(std::abs(degradedCommandScale(0.3,0.6,0.15,0.3)-0.5)<1e-10);
  assert(degradedCommandScale(0,0,0.15,0.3)==1);
  assert(degradedCommandScale(NAN,0,0.15,0.3)==0);
}
