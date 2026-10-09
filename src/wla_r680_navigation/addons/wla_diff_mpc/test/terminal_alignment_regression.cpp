#include "wla_diff_mpc/terminal_alignment.hpp"
#include <stdexcept>
#include <iostream>
using namespace wla_diff_mpc;
void check(bool value,const char * reason) {if(!value)throw std::runtime_error(reason);}
int main() {
  TerminalAlignment s;double w=0;
  auto tick=[&](double t,double error,double v,double rate,bool fresh=true,double distance=.1) {
    return s.update(distance,error,v,rate,fresh,t,.2,.18,w);
  };
  check(!tick(0,.6,0,0,true,1),"terminal took over away from goal");
  check(tick(0,.6,.1,.3) && w==0,"must stop first");
  tick(1,.6,0,0,false);check(w==0,"stale feedback allowed rotation");
  tick(2,.6,0,0);tick(2.2,.6,0,0);check(w==0,"did not confirm stable stopping");
  tick(2.4,.6,0,0);tick(2.45,.6,0,0);
  check(w>0 && w<=.22,"no bounded yaw alignment after settling");
  tick(2.5,.3,0,.3);check(w==0 && s.phase()==TerminalAlignment::Phase::Brake,
    "did not brake using measured angular speed");
  tick(2.6,-.3,0,.2);check(w==0,"reversed before settling");
  tick(3,-.3,0,0);tick(3.4,-.3,0,0);tick(3.45,-.3,0,0);
  check(w<0,"did not correct overshoot after stopping");
  tick(4,.1,0,-.03);check(w==0 && s.phase()==TerminalAlignment::Phase::Hold,
    "did not hold zero within yaw tolerance");
  for(int k=0;k<20;++k) {tick(4+k*.1,(k%2 ? .17 : -.17),0,0);check(w==0,"chased tolerance noise");}
  tick(7,.1,0,0,false);check(w==0,"hold failed during missing feedback");
  tick(8,.1,0,0,true,.25);tick(8.4,.1,0,0,true,.25);
  check(s.phase()==TerminalAlignment::Phase::Track,"hold outside XY tolerance must reapproach after settling");
  s.reset();tick(0,.5,.1,0);tick(1,.5,0,0,true,.25);tick(1.4,.5,0,0,true,.25);
  check(s.phase()==TerminalAlignment::Phase::Track,"must reapproach if stopping left XY tolerance");
  s.reset();tick(0,.5,0,0);tick(.4,.5,0,0);tick(.5,.5,0,0);
  tick(.6,.5,0,.1,false);check(w==0,"alignment continued with stale velocity");
  tick(.65,.5,0,0);tick(.8,.5,0,0);check(w==0,"resumed without stopping dwell after feedback gap");
  // Recorded terminal cycle: align inside 0.20m, a small map/pose correction
  // moves the apparent distance to 0.216m while yaw is still about 0.6rad off.
  // A stop must not re-enable tracking and its opposite-direction steering.
  for(double direction:{-1.0,1.0}) {
    s.reset();tick(0,direction*.9,0,0,true,.18);tick(.4,direction*.9,0,0,true,.18);
    tick(.45,direction*.9,0,0,true,.18);check(w*direction>0,"alignment did not start");
    tick(.5,direction*.6,.04,direction*.1,true,.216);
    check(s.phase()==TerminalAlignment::Phase::Stopping && w==0,"motion must still request stopping");
    tick(.6,direction*.6,0,0,true,.216);tick(1,direction*.6,0,0,true,.216);
    check(s.phase()==TerminalAlignment::Phase::Align,"small correction reset alignment to tracking");
    for(int k=0;k<20;++k) {
      tick(1.05+k*.05,direction*.6,0,0,true,k%2 ? .19 : .216);
      check(s.phase()==TerminalAlignment::Phase::Align && w*direction>0,
        "distance jitter reversed alignment or selected tracking");
    }
    tick(2.1,direction*.6,0,0,true,.29);tick(2.2,direction*.6,0,0,true,.29);
    tick(2.6,direction*.6,0,0,true,.29);
    check(s.phase()==TerminalAlignment::Phase::Track && w==0,"large correction must stop before reacquiring position");
  }
  // A delayed first-order chassis: output arrives 0.4s later, body response
  // takes another 0.5s. Verify convergence and zero holds for both yaw signs.
  for(double initial:{.65,-.65,3.12,-3.12}) {
    s.reset();double yaw=0,rate=0;double queue[8]={};int zero_tail=0,flips=0;double last_nonzero=0;
    for(int k=0;k<2400;++k) {
      double error=std::remainder(initial-yaw,2*3.14159265358979323846);
      tick(k*.05,error,0,rate);
      if (w*last_nonzero<0) ++flips;
      if (w!=0) last_nonzero=w;
      const double delayed=queue[k%8];queue[k%8]=w;
      rate+=(delayed-rate)*(.05/.5);yaw+=rate*.05;
      if (std::abs(error)<=.18 && std::abs(rate)<.04 && w==0) ++zero_tail; else zero_tail=0;
      if (zero_tail>30) break;
    }
    check(zero_tail>30,"delayed chassis did not settle within goal tolerance");
    check(flips<=2,"delayed chassis repeatedly reversed");
  }
  std::cout<<"PASS: stopping, feedback freshness, angular braking, no early reversal, zero hold, delayed chassis convergence\n";
}
