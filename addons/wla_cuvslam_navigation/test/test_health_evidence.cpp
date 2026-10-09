#include "health_evidence.hpp"
#include <stdexcept>
using namespace wla_vio;
void check(bool v) {if(!v)throw std::runtime_error("health evidence regression");}
int main() {
  HealthEvidence e;
  check(!e.valid());
  check(e.reason(false)=="gravity_unavailable|imu_state_unavailable|velocity_interval_invalid");
  e={true,true,true,true,true,true};
  check(e.valid() && e.reason(false)=="inertial_stability_window");
  check(e.reason(true)=="tracking_inertial_ready");
  e.gyro_bias_valid=false;e.accel_bias_valid=false;
  check(!e.valid() && e.reason(false)=="gyro_bias_out_of_bounds|accel_bias_out_of_bounds");
  e={true,false,true,true,true,false};
  check(e.reason(false)=="gravity_out_of_bounds|velocity_interval_invalid");
  e={true,true,false,false,false,true};
  check(e.reason(false)=="imu_state_unavailable");
}
