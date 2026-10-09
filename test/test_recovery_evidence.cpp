#include "wla_r680_navigation/recovery_evidence.hpp"
#include <limits>
#include <stdexcept>
using namespace wla_r680_navigation;
void check(bool ok) {if(!ok) throw std::runtime_error("recovery evidence regression");}
int main() {
  RecoveryEvidence e;
  check(!e.verified());
  check(!e.accept(0,{1,2,0}) && !e.verified());
  check(!e.accept(1000000000,{std::numeric_limits<double>::quiet_NaN(),2,0}) && !e.verified());
  check(e.accept(1000000000,{1,2,3.13}) && e.verified());
  check(!e.accept(1000000000,{1,2,3.13}) && e.verified());
  check(!e.accept(500000000,{1,2,3.13}) && e.verified());
  check(!e.accept(2000000000,{1,std::numeric_limits<double>::infinity(),0}) && e.verified());
  e.reset();check(!e.verified());
  check(e.accept(3000000000,{5,4,1.57}) && e.verified());
}
