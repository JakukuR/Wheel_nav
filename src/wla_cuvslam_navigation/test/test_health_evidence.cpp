#include "health_evidence.hpp"
#include <stdexcept>
using namespace wla_vio;
void check(bool v) {if(!v)throw std::runtime_error("SDK initialization evidence regression");}
int main() {
  HealthEvidence e;
  check(!e.valid());
  check(e.reason()=="gravity_unavailable|imu_state_unavailable");
  e={true,true,true,true,true};
  check(e.valid() && e.reason()=="tracking_inertial_ready");
  // One missing SDK result closes readiness; the next valid result recovers
  // immediately, without a velocity sample count or additional time window.
  e.gravity_available=false;check(!e.valid());
  e.gravity_available=true;check(e.valid());
  e.gyro_bias_valid=false;e.accel_bias_valid=false;
  check(!e.valid() && e.reason()=="gyro_bias_nonfinite|accel_bias_nonfinite");
  e={true,false,true,true,true};check(e.reason()=="gravity_nonfinite");
  e={true,true,false,false,false};check(e.reason()=="imu_state_unavailable");
}
