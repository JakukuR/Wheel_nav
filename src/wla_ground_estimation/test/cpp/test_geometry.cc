#include "wla_ground/geometry.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace wla::ground;
namespace {
void Check(bool value, const char* reason) { if (!value) throw std::runtime_error(reason); }
void Near(double actual, double expected) { Check(std::abs(actual-expected) < 1e-10, "numeric_mismatch"); }
template<class F> void Reject(F function) { bool rejected = false; try { function(); } catch (const std::invalid_argument&) { rejected = true; } Check(rejected, "invalid_input_accepted"); }
}
int main() {
  try {
    const auto identity = Eigen::Quaterniond::Identity();
    const std::array<Contact, 2> grounded{Contact::Ground, Contact::Ground};
    Wheel wheel{Eigen::Vector3d(0, .1, -.2), Eigen::Vector3d::UnitY(), .04, .015};
    Near(CylinderSupport(wheel, Eigen::Vector3d::UnitZ()), .04);
    Near(CylinderSupport(wheel, Eigen::Vector3d::UnitY()), .015);
    Near(CylinderSupport(wheel, Eigen::Vector3d(0, .6, .8)), .04*.8+.015*.6);
    auto pair = std::array<Wheel, 2>{wheel, wheel}; pair[1].center.y() = -.1;
    auto flat = EstimateFlatGround(pair, identity, grounded, .03);
    Check(flat.hasEstimate && flat.contactKnown, "ground_contact_missing"); Near(flat.plane.offset, .24);
    auto unknown = EstimateFlatGround(pair, identity, {Contact::Unknown, Contact::Unknown}, .03);
    Check(unknown.hasEstimate && !unknown.contactKnown, "unknown_contact_claimed_valid");
    Check(!EstimateFlatGround(pair, identity, {Contact::Air, Contact::Air}, .03).hasEstimate, "flight_height_invented");
    pair[1].center.z() = -.30;
    Check(!EstimateFlatGround(pair, identity, grounded, .03).hasEstimate, "contact_disagreement_accepted");
    auto single = EstimateFlatGround(pair, identity, {Contact::Ground, Contact::Air}, .03);
    Check(single.hasEstimate && single.contactKnown, "single_contact_lost"); Near(single.plane.offset, .24);
    const Eigen::Quaterniond attitude(FromRpy(Eigen::Vector3d(.3, -.4, .8)));
    for (int side = 0; side < 2; ++side) {
      pair[side].axis = attitude.conjugate()*Eigen::Vector3d::UnitY();
      pair[side].center = attitude.conjugate()*Eigen::Vector3d(.1, side == 0 ? .1 : -.1, .04-.31);
    }
    flat = EstimateFlatGround(pair, attitude, grounded, .03); Near(flat.plane.offset, .31);
    Near(VerticalHeight(flat.plane, attitude), .31);
    Eigen::Isometry3d mount = Eigen::Isometry3d::Identity();
    mount.translation() = Eigen::Vector3d(.06, -.02, .08); mount.linear() = FromRpy(Eigen::Vector3d(.1, .2, .3));
    const auto sensor_plane = TransformPlane(flat.plane, mount);
    const Eigen::Vector3d body_point = attitude.conjugate()*Eigen::Vector3d(.2, .3, -.31);
    Near(sensor_plane.normal.dot(mount.inverse()*body_point)+sensor_plane.offset, 0.0);
    std::vector<Eigen::Vector3d> points;
    const Eigen::Vector3d normal = Eigen::Vector3d(.1, -.2, 1).normalized();
    for (double x : {-.2, .2}) for (double y : {-.1, .1})
      points.emplace_back(x, y, (-.3-normal.x()*x-normal.y()*y)/normal.z());
    auto fit = FitGroundPlane(points, Eigen::Vector3d::UnitZ(), .005, .6);
    Near((fit.normal-normal).norm(), 0.0); Near(fit.offset, .3);
    points.back().z() += .2;
    Reject([&]() { (void)FitGroundPlane(points, Eigen::Vector3d::UnitZ(), .005, .6); });
    Reject([&]() { (void)FitGroundPlane({{0,0,-.2},{.1,0,-.2},{.2,0,-.2}}, Eigen::Vector3d::UnitZ(), .01, .6); });
    Reject([&]() { (void)CheckedQuaternion(Eigen::Quaterniond(0,0,0,0)); });
    Reject([&]() { auto bad = wheel; bad.radius = -1; (void)CylinderSupport(bad, Eigen::Vector3d::UnitZ()); });
    Reject([&]() { (void)FromRpy(Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),0,0)); });
    Eigen::Isometry3d body_from_model = Eigen::Isometry3d::Identity(); body_from_model.translation() = Eigen::Vector3d(.02,0,-.0005);
    WheelModel model(std::string(TEST_DATA_DIR)+"/kinematic.urdf", {"wheel_L_Link","wheel_R_Link"}, body_from_model);
    std::map<std::string,double> joints{{"hip_L",0},{"hip_R",0},{"wheel_L",0},{"wheel_R",0}};
    auto evaluated = model.evaluate(joints);
    Near(evaluated[0].center.x(), .02); Near(evaluated[0].center.y(), .11); Near(evaluated[0].center.z(), -.2005);
    Near(EstimateFlatGround(evaluated, identity, grounded, .03).plane.offset, .2405);
    joints["wheel_L"] = 2.1; auto spun = model.evaluate(joints);
    Near((spun[0].center-evaluated[0].center).norm(), 0.0);
    joints["hip_L"] = 1.5707963267948966; evaluated = model.evaluate(joints);
    Near(evaluated[0].center.x(), -.18); Near(evaluated[0].center.z(), -.0005);
    joints.erase("hip_L"); Reject([&]() { (void)model.evaluate(joints); });
    std::cout << "geometry: support, contact, roll/pitch, plane, FK, root shift, spin and invalid-input checks passed\n";
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
