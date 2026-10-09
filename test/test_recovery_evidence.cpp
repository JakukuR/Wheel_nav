#include "wla_r680_navigation/recovery_evidence.hpp"
#include <limits>
#include <stdexcept>
using namespace wla_r680_navigation;
void check(bool ok) {if(!ok) throw std::runtime_error("recovery evidence regression");}
int main() {
  RecoveryEvidence e;
  check(e.add(1000000000,{1,2,3.13},0.08,0.1));
  check(!e.add(1000000000,{1,2,3.13},0.08,0.1) && e.count()==1);
  check(e.add(2000000000,{1.01,2,-3.13},0.08,0.1) && e.count()==2);
  check(e.add(3000000000,{1.02,2,-3.12},0.08,0.1) && e.count()==3);
  check(e.add(4000000000,{2,2,-3.12},0.08,0.1) && e.count()==1);
  check(!e.add(5000000000,{std::numeric_limits<double>::quiet_NaN(),2,0},0.08,0.1));
  check(e.add(8000000000,{2,2,-3.12},0.08,0.1) && e.count()==1);
  e.reset();check(e.count()==0);
}
