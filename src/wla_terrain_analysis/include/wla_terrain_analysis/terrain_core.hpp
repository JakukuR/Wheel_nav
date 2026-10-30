// Terrain (slope / step) analysis for a forward depth camera.
//
// Ported from the CMU terrain_analysis package (Ji Zhang, Carnegie Mellon
// University, BSD licence per its package.xml; autonomous_exploration_
// development_environment, jazzy branch): scans registered in a gravity-aligned
// odom frame are accumulated with time decay, and a ground height is estimated
// per planar cell from a low z quantile. Changes for this robot:
//  * ground is a robust local plane per cell, so ramps are not obstacles and a
//    slope angle and a step (residual discontinuity) are available;
//  * a cell observed in the current frame keeps only current points
//    (moved objects and old noise are replaced, not decayed for seconds);
//  * an obstacle cell is reported after it is seen in min_confirm frames;
//  * drops (ground below the local plane) are reported like steps when
//    consider_drop is on (off by default, as CMU considerDrop on its Diablo);
//  * a cell blocked by its ground alone (no points above it) needs blocked
//    neighbours (CMU pools each cell's 3x3 neighbourhood and its planner needs
//    pointPerPathThre = 2 points on a path).
// No ROS dependency: the ROS nodes and the unit tests share this core.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace wla_terrain {

struct Params {
  double planar_cell = .10;      // m, ground/slope cell (CMU uses 0.2 for lidar)
  double point_voxel = .05;      // m, per-cell downsampling (CMU scanVoxelSize)
  double ground_quantile = .25;  // low z quantile of a cell = its ground sample (CMU 0.25)
  int min_cell_points = 4;       // points for a cell ground sample (CMU minBlockPointNum)
  double fit_radius = .30;       // m, neighbour cells used for the local plane
  int min_fit_cells = 4;
  double fit_outlier = .02;      // m, residual excluding a cell from the plane (< max_step/2,
                                 // so the two levels of a step do not average into one plane)
  double obstacle_height = .05;  // m above local ground: obstacle (step limit)
  double vehicle_height = .50;   // m above local ground: ignored (overhead)
  double max_slope_deg = 20.;    // climbable slope (robot spec)
  double slope_margin_deg = 3.;  // estimate noise at the limit; blocked above max+margin
  double max_step = .05;         // m, ground discontinuity / drop limit
  double step_span = .30;        // m, distance between the two planes compared for a step
  double drop_shadow = .60;      // m, longest unseen gap behind an edge checked for a drop
  double replace_age = .35;      // s, an observed cell keeps only points this recent
  double decay_time = 2.;        // s, unobserved points beyond keep_radius
  double keep_radius = 2.5;      // m, unobserved points kept (camera blind zone)
  double map_radius = 5.;        // m, analysis window around the robot
  double terrain_range = 3.5;    // m, slope/step/drop judged only this close (D455 depth
                                 // error ~2% at 4 m would exceed max_step); obstacles beyond
  int min_confirm = 2;           // frames an obstacle cell must be seen in
  bool consider_drop = false;    // judge ground below the local plane: sunk cells, drops and
                                 // steps down (CMU considerDrop; its Diablo runs with it off)
  int min_residual_points = 8;   // points for a raised-cell verdict (CMU minBlockPointNum 10)
  int min_blocked_neighbours = 2;  // of 8, for a cell blocked by its ground alone (steep,
                                   // step, raised); an edge pair alone has one each
};

struct Pose {  // gravity-aligned odom frame, ZYX Euler
  double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
};

struct Point { double x, y, z; };

enum class CellClass : uint8_t { Unknown = 0, Ground, Obstacle, Steep, Step };

struct CellResult {
  int64_t ix = 0, iy = 0;
  CellClass cls = CellClass::Unknown;
  double ground = 0;      // plane height at the cell centre (odom z)
  double raw = 0;         // the cell's own ground sample (z quantile)
  double slope_deg = 0;
  double step = 0;        // largest neighbour discontinuity / drop
  bool observed = false;  // seen in the latest frame
  // Why blocked (diagnostics): 1 sunk, 2 raised, 3 step between planes, 4 drop
  // after an unseen gap, 5 points above ground; 0 none / steep slope.
  uint8_t reason = 0;
};

struct Output {
  // Points with height above local ground in [obstacle_height, vehicle_height),
  // from confirmed obstacle cells. Odom frame; h = height above ground.
  std::vector<std::array<double, 4>> obstacles;  // x, y, z, h
  std::vector<std::array<double, 4>> terrain;    // every analysed point, h may be < 0
  std::vector<CellResult> cells;                 // analysed cells
};

// Rotation body->odom for ZYX Euler (yaw, pitch, roll).
inline std::array<double, 9> rotation(const Pose &p) {
  const double cr = std::cos(p.roll), sr = std::sin(p.roll), cp = std::cos(p.pitch),
               sp = std::sin(p.pitch), cy = std::cos(p.yaw), sy = std::sin(p.yaw);
  return {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
          sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
          -sp,     cp * sr,                cp * cr};
}

inline Point to_odom(const Pose &pose, const std::array<double, 9> &r, const Point &b) {
  return {r[0] * b.x + r[1] * b.y + r[2] * b.z + pose.x,
          r[3] * b.x + r[4] * b.y + r[5] * b.z + pose.y,
          r[6] * b.x + r[7] * b.y + r[8] * b.z + pose.z};
}

class TerrainAnalyzer {
 public:
  explicit TerrainAnalyzer(Params p = {}) : p_(p) {}
  const Params &params() const { return p_; }

  // Odom-frame points of one depth frame taken at time t with the robot at pose.
  void add_frame(const std::vector<Point> &points, const Pose &pose, double t);
  // Classify cells and points around the robot (call after add_frame).
  Output analyze(const Pose &pose, double t);
  void clear() { cells_.clear(); }
  size_t stored_points() const;

 private:
  struct Stored { double x, y, z, t; };
  struct Cell {
    std::vector<Stored> points;
    double last_seen = -1e18;
    int confirm = 0;          // consecutive frames classified as obstacle
    bool observed = false;    // touched by the latest frame
    bool was_obstacle = false;
  };
  static int64_t key(int64_t ix, int64_t iy) { return (ix << 32) ^ (iy & 0xffffffff); }
  int64_t cell_index(double v) const { return static_cast<int64_t>(std::floor(v / p_.planar_cell)); }

  Params p_;
  std::unordered_map<int64_t, Cell> cells_;
  std::unordered_map<int64_t, std::pair<int64_t, int64_t>> coords_;
};

}  // namespace wla_terrain
