#include <cassert>
#include "wla_r680_navigation/map_localization_evidence.hpp"
using wla_r680_navigation::MapLocalizationEvidence;
int main() {
  MapLocalizationEvidence e; e.reset(10.);
  auto ready=[&](double now,bool tf=true,bool odom=true,bool vio=true,double grace=0.) {
    return e.update(now,.4,tf,odom,vio,grace);
  };
  e.match(10.1,10.1,.4);assert(!ready(10.1)); // event alone
  e.pose(10.3,10.3,.4);assert(!ready(10.3)); // different backend update
  e.match(10.3,10.3,.4);assert(ready(10.3)); // reverse callback order
  e.pose(10.5,10.5,.4);assert(ready(10.5)); // no perpetual loops required
  assert(!ready(11.)); // source age expiry clears the session
  e.pose(11.1,11.1,.4);assert(!ready(11.1));
  e.match(10.3,11.1,.4);assert(!ready(11.1)); // delayed pre-loss match
  e.match(11.1,11.1,.4);assert(ready(11.1));
  assert(!ready(11.2,false)); // lost TF
  e.pose(11.3,11.3,.4);assert(!ready(11.3));
  e.match(11.3,11.3,.4);assert(!ready(11.3,true,true,false)); // VIO required for admission
  assert(ready(11.3));
  assert(ready(11.8,true,true,false,2.)); // existing bounded continuity only
  assert(!ready(13.8,true,true,false,2.)); // bounded grace expires
  e.match(14.,14.,.4);e.pose(14.,14.,.4);assert(ready(14.));
  assert(!ready(14.1,true,false)); // continuity failure clears evidence
  e.match(14.,14.2,.4);e.pose(14.2,14.2,.4);assert(!ready(14.2));
  e.match(14.2,14.2,.4);assert(ready(14.2));
  e.reset(15.);e.pose(15.1,15.1,.4);assert(!ready(15.1)); // reset/manual prior
  e.match(16.,15.1,.4);assert(!ready(15.1)); // future evidence

  // Retain an early startup match through a long cuVSLAM warmup, but never
  // treat it as sufficient localization or permit it across a reset.
  e.reset(20., true);
  e.match(19.,20.1,.4); e.pose(20.1,20.1,.4);
  assert(!ready(20.1)); // old evidence rejected even during initial admission
  e.match(20.2,20.2,.4);
  assert(!ready(20.2,true,true,false)); // no VIO yet
  assert(!ready(80.)); // the old pose is not enough after initialization
  e.pose(80.,80.,.4);
  assert(!ready(80.,false));
  assert(!ready(80.,true,false));
  assert(!ready(80.,true,true,false));
  assert(ready(80.)); // fresh pose/TF/odom plus the retained initial match
  assert(!e.retainsInitialMatch());
  assert(!ready(81.)); // expiry revokes admission
  e.pose(81.1,81.1,.4); assert(!ready(81.1));
  e.match(81.1,81.1,.4); assert(ready(81.1));
  e.reset(90.,true); e.match(90.1,90.1,.4);
  e.reset(91.); e.pose(91.1,91.1,.4); assert(!ready(91.1));
  // admit=false withholds a new confirmation but never revokes an existing one.
  e.reset(100.); e.match(100.1,100.1,.4); e.pose(100.1,100.1,.4);
  assert(!e.update(100.1,.4,true,true,true,0.,false));
  assert(e.update(100.1,.4,true,true,true,0.,true));
  assert(e.update(100.2,.4,true,true,true,0.,false));
}
