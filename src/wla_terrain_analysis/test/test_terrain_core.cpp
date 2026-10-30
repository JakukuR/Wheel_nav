// Synthetic D455 frames ray-cast into height fields; no ROS.
#include "wla_terrain_analysis/terrain_core.hpp"
#include <cstdio>
#include <functional>
#include <random>
#include <stdexcept>
#include <string>

using namespace wla_terrain;
using Height = std::function<double(double, double)>;

static void check(bool ok, const std::string &what) {
  if (!ok) throw std::runtime_error(what);
}

// Camera 0.184 m ahead, 0.50 m up, pitched down 8.85 deg (car mount); 640x480,
// fx=fy=385, stride 4, range 0.15-5 m; depth noise sigma = noise * range^2
// (D455 spec: <2% at 4 m, i.e. about 0.005).
static std::vector<Point> frame(const Height &h, const Pose &pose, std::mt19937 &rng, double noise = .005) {
  std::normal_distribution<double> n(0., 1.);
  const auto r = rotation(pose);
  const double pitch = 8.85 * M_PI / 180., cx = .184, cz = .50;
  std::vector<Point> out;
  for (int v = 0; v < 480; v += 4) for (int u = 0; u < 640; u += 4) {
    // optical ray -> body (x fwd, y left, z up), then camera pitch down
    const double ox = (u - 320) / 385., oy = (v - 240) / 385.;
    double bx = 1, by = -ox, bz = -oy;
    const double tx = bx * std::cos(pitch) + bz * std::sin(pitch), tz = -bx * std::sin(pitch) + bz * std::cos(pitch);
    bx = tx; bz = tz;
    const double norm = std::sqrt(bx * bx + by * by + bz * bz);
    const Point o = to_odom(pose, r, {cx, 0, cz});
    const Point e = to_odom(pose, r, {cx + bx / norm, by / norm, cz + bz / norm});
    const double dx = e.x - o.x, dy = e.y - o.y, dz = e.z - o.z;
    for (double s = .15; s <= 5.; s += .01) {
      const double x = o.x + dx * s, y = o.y + dy * s, z = o.z + dz * s;
      if (z <= h(x, y)) {
        const double ds = s * (1. + noise * s * n(rng));
        out.push_back({o.x + dx * ds, o.y + dy * ds, o.z + dz * ds});
        break;
      }
    }
  }
  return out;
}

struct Counts { int ground = 0, blocked = 0, steep = 0, step = 0, obstacle = 0, unknown = 0; };
static Counts region(const Output &o, const Params &p, double x0, double x1, double y0, double y1) {
  Counts c;
  for (const auto &cell : o.cells) {
    const double x = (cell.ix + .5) * p.planar_cell, y = (cell.iy + .5) * p.planar_cell;
    if (x < x0 || x > x1 || y < y0 || y > y1) continue;
    switch (cell.cls) {
      case CellClass::Ground: ++c.ground; break;
      case CellClass::Steep: ++c.steep; ++c.blocked; break;
      case CellClass::Step: ++c.step; ++c.blocked; break;
      case CellClass::Obstacle: ++c.obstacle; ++c.blocked; break;
      default: ++c.unknown;
    }
  }
  return c;
}
static int obstacles_in(const Output &o, double x0, double x1, double y0, double y1) {
  int n = 0;
  for (const auto &q : o.obstacles) if (q[0] >= x0 && q[0] <= x1 && q[1] >= y0 && q[1] <= y1) ++n;
  return n;
}

static Output run(const Height &h, int frames = 3, double noise = .005, unsigned seed = 1, TerrainAnalyzer *keep = nullptr, double t0 = 0) {
  std::mt19937 rng(seed);
  TerrainAnalyzer local;
  TerrainAnalyzer &a = keep ? *keep : local;
  Pose pose; Output o;
  for (int k = 0; k < frames; ++k) { a.add_frame(frame(h, pose, rng, noise), pose, t0 + .1 * k); o = a.analyze(pose, t0 + .1 * k); }
  return o;
}

static Height ramp(double deg) {  // flat until x=1.5, then rising along +x
  const double g = std::tan(deg * M_PI / 180.);
  return [g](double x, double) { return x > 1.5 ? (x - 1.5) * g : 0.; };
}
static Height step(double hgt) { return [hgt](double x, double) { return x > 1.8 ? hgt : 0.; }; }

int main() {
  try {
    Params p;
    // Flat floor: nothing blocked.
    {auto o = run([](double, double) { return 0.; });
     auto c = region(o, p, 1.0, 3.0, -1.0, 1.0);
     std::printf("flat      ground %d blocked %d unknown %d obstacles %zu\n", c.ground, c.blocked, c.unknown, o.obstacles.size());
     check(c.ground > 100 && c.blocked == 0 && o.obstacles.empty(), "flat floor blocked");}
    for (double deg : {10., 15., 20., 25., 30.}) {
      auto o = run(ramp(deg));
      auto on = region(o, p, 1.9, 2.8, -.8, .8), edge = region(o, p, 1.3, 1.8, -.8, .8);
      std::printf("ramp %2.0f  on: ground %d steep %d step %d obst %d | start: ground %d blocked %d | obstacle pts on ramp %d\n",
                  deg, on.ground, on.steep, on.step, on.obstacle, edge.ground, edge.blocked, obstacles_in(o, 1.9, 2.8, -.8, .8));
      if (deg <= 20.) check(on.blocked * 10 <= on.ground && obstacles_in(o, 1.3, 2.8, -.8, .8) <= 3, "traversable ramp blocked");
      else check(on.steep * 2 >= on.ground, "steep ramp not blocked");
    }
    for (double hgt : {.03, .04, .06, .08, -.06, -.10}) {
      auto o = run(step(hgt));
      auto at = region(o, p, 1.65, 1.95, -.8, .8);
      std::printf("step %+.2f at edge: ground %d step %d obst %d | obstacle pts %d\n", hgt, at.ground, at.step, at.obstacle, obstacles_in(o, 1.6, 2.0, -.8, .8));
      if (std::abs(hgt) <= .04) check(obstacles_in(o, 1.6, 2.0, -.8, .8) <= 3, "small step blocked");
      else check(obstacles_in(o, 1.6, 2.0, -.8, .8) >= 5, "high step / drop not blocked");
    }
    // A 0.4 m x 0.4 m x 0.2 m box at x=2.
    {auto o = run([](double x, double y) { return (x > 2. && x < 2.4 && std::abs(y) < .2) ? .2 : 0.; });
     const int box = obstacles_in(o, 1.95, 2.45, -.25, .25), around = obstacles_in(o, 1.0, 1.85, -1., 1.);
     std::printf("box       obstacle pts on box %d, before box %d\n", box, around);
     check(box >= 10 && around == 0, "box not isolated");}
    // One noisy frame with a spurious 15 cm blob: not confirmed.
    {TerrainAnalyzer a; std::mt19937 rng(3); Pose pose;
     auto flat = [](double, double) { return 0.; };
     a.add_frame(frame(flat, pose, rng), pose, 0); a.analyze(pose, 0);
     auto noisy = frame(flat, pose, rng);
     for (int i = 0; i < 12; ++i) noisy.push_back({2.2 + .01 * i, .1, .15});
     a.add_frame(noisy, pose, .1); auto o = a.analyze(pose, .1);
     std::printf("noise     single-frame blob obstacle pts %d\n", obstacles_in(o, 2.1, 2.4, 0, .2));
     check(obstacles_in(o, 2.1, 2.4, 0, .2) == 0, "single-frame noise published");
     a.add_frame(frame(flat, pose, rng), pose, .2); o = a.analyze(pose, .2);
     check(obstacles_in(o, 2.1, 2.4, 0, .2) == 0, "cleared blob remained");}
    // A box that is removed (person walked away) is cleared once seen again.
    {TerrainAnalyzer a;
     auto box = [](double x, double y) { return (x > 2. && x < 2.4 && std::abs(y) < .2) ? .3 : 0.; };
     run(box, 3, .005, 5, &a);
     // Seen again for 0.4 s (replace_age 0.35 s): the box points are gone.
     auto o = run([](double, double) { return 0.; }, 5, .005, 6, &a, 1.);
     std::printf("moved     obstacle pts after the box left %d\n", obstacles_in(o, 1.9, 2.5, -.3, .3));
     check(obstacles_in(o, 1.9, 2.5, -.3, .3) == 0, "moved object left a ghost");}
    // Robot already pitched on a 20 deg ramp (pose from the IMU): ramp ahead is ground.
    {std::mt19937 rng(9); TerrainAnalyzer a; const double g = std::tan(20. * M_PI / 180.);
     auto slope = [g](double x, double) { return x * g; };
     Pose pose; pose.pitch = -20. * M_PI / 180.;  // nose up (REP-103)
     Output o; for (int k = 0; k < 3; ++k) { a.add_frame(frame(slope, pose, rng), pose, .1 * k); o = a.analyze(pose, .1 * k); }
     auto c = region(o, p, 1.0, 2.5, -.8, .8);
     std::printf("on ramp   ground %d blocked %d obstacle pts %zu\n", c.ground, c.blocked, o.obstacles.size());
     check(c.blocked * 10 <= c.ground, "ramp ahead blocked while climbing");}
    std::puts("terrain core checks passed");
  } catch (const std::exception &e) { std::fprintf(stderr, "FAIL: %s\n", e.what()); return 1; }
  return 0;
}
