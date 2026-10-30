#include "wla_terrain_analysis/terrain_core.hpp"

#include <algorithm>
#include <unordered_set>

namespace wla_terrain {
namespace {

struct Plane { double a = 0, b = 0, c = 0; bool valid = false; };  // z = a*dx + b*dy + c

// Least-squares plane through (dx, dy, z) samples centred on the fitted cell.
Plane fit(const std::vector<std::array<double, 3>> &s, const std::vector<char> &use) {
  double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, n = 0, sxz = 0, syz = 0, sz = 0;
  for (size_t i = 0; i < s.size(); ++i) {
    if (!use[i]) continue;
    const auto &q = s[i];
    sxx += q[0] * q[0]; sxy += q[0] * q[1]; syy += q[1] * q[1];
    sx += q[0]; sy += q[1]; n += 1; sxz += q[0] * q[2]; syz += q[1] * q[2]; sz += q[2];
  }
  Plane p;
  if (n < 3) return p;
  // Solve [sxx sxy sx; sxy syy sy; sx sy n] [a b c]' = [sxz syz sz]'
  const double m[3][4] = {{sxx, sxy, sx, sxz}, {sxy, syy, sy, syz}, {sx, sy, n, sz}};
  double a[3][4];
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 4; ++j) a[i][j] = m[i][j];
  for (int col = 0; col < 3; ++col) {
    int piv = col;
    for (int r = col + 1; r < 3; ++r) if (std::abs(a[r][col]) > std::abs(a[piv][col])) piv = r;
    if (std::abs(a[piv][col]) < 1e-9) return p;  // collinear samples
    for (int j = 0; j < 4; ++j) std::swap(a[col][j], a[piv][j]);
    for (int r = 0; r < 3; ++r) {
      if (r == col) continue;
      const double f = a[r][col] / a[col][col];
      for (int j = col; j < 4; ++j) a[r][j] -= f * a[col][j];
    }
  }
  p.a = a[0][3] / a[0][0]; p.b = a[1][3] / a[1][1]; p.c = a[2][3] / a[2][2]; p.valid = true;
  return p;
}

}  // namespace

size_t TerrainAnalyzer::stored_points() const {
  size_t n = 0;
  for (const auto &kv : cells_) n += kv.second.points.size();
  return n;
}

void TerrainAnalyzer::add_frame(const std::vector<Point> &points, const Pose &pose, double t) {
  for (auto &kv : cells_) kv.second.observed = false;
  // Group this frame's points by planar cell, one point per small voxel.
  std::unordered_map<int64_t, std::vector<Stored>> fresh;
  std::unordered_set<int64_t> voxels;
  const double r2 = (p_.map_radius * 1.5) * (p_.map_radius * 1.5);
  for (const auto &q : points) {
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) continue;
    const double dx = q.x - pose.x, dy = q.y - pose.y;
    if (dx * dx + dy * dy > r2) continue;
    const auto vx = static_cast<int64_t>(std::floor(q.x / p_.point_voxel));
    const auto vy = static_cast<int64_t>(std::floor(q.y / p_.point_voxel));
    const auto vz = static_cast<int64_t>(std::floor(q.z / p_.point_voxel));
    const int64_t vkey = (vx * 73856093) ^ (vy * 19349663) ^ (vz * 83492791);
    if (!voxels.insert(vkey).second) continue;
    const int64_t ix = cell_index(q.x), iy = cell_index(q.y);
    fresh[key(ix, iy)].push_back({q.x, q.y, q.z, t});
    coords_[key(ix, iy)] = {ix, iy};
  }
  for (auto &kv : fresh) {
    auto &cell = cells_[kv.first];
    if (static_cast<int>(kv.second.size()) >= std::max(2, p_.min_cell_points / 2)) {
      // Seen now: points older than replace_age go (moved objects, old noise);
      // the last few frames stay, since one frame samples a far cell sparsely.
      // CMU instead decays all points, which keeps a ghost for decayTime.
      cell.points.erase(std::remove_if(cell.points.begin(), cell.points.end(),
        [&](const Stored &s) { return t - s.t > p_.replace_age; }), cell.points.end());
      cell.observed = true;
      cell.last_seen = t;
    }
    cell.points.insert(cell.points.end(), kv.second.begin(), kv.second.end());
    if (cell.points.size() > 96) cell.points.erase(cell.points.begin(), cell.points.end() - 96);
  }
  // Decay unobserved points beyond the keep radius (CMU decayTime / noDecayDis).
  const double keep2 = p_.keep_radius * p_.keep_radius;
  for (auto it = cells_.begin(); it != cells_.end();) {
    auto &cell = it->second;
    const auto &c = coords_[it->first];
    const double cx = (c.first + .5) * p_.planar_cell - pose.x, cy = (c.second + .5) * p_.planar_cell - pose.y;
    const double d2 = cx * cx + cy * cy;
    if (d2 > r2) { coords_.erase(it->first); it = cells_.erase(it); continue; }
    if (!cell.observed && d2 > keep2) {
      cell.points.erase(std::remove_if(cell.points.begin(), cell.points.end(),
        [&](const Stored &s) { return t - s.t > p_.decay_time; }), cell.points.end());
    }
    if (cell.points.empty() && !cell.was_obstacle) { coords_.erase(it->first); it = cells_.erase(it); continue; }
    ++it;
  }
}

Output TerrainAnalyzer::analyze(const Pose &pose, double t) {
  (void)t;
  Output out;
  const double cell = p_.planar_cell, r2 = p_.map_radius * p_.map_radius;
  // 1. Ground sample per cell: low z quantile of its own points.
  std::unordered_map<int64_t, double> raw;
  std::vector<int64_t> keys;
  for (auto &kv : cells_) {
    const auto &c = coords_[kv.first];
    const double cx = (c.first + .5) * cell - pose.x, cy = (c.second + .5) * cell - pose.y;
    if (cx * cx + cy * cy > r2) continue;
    keys.push_back(kv.first);
    const auto &pts = kv.second.points;
    if (static_cast<int>(pts.size()) < p_.min_cell_points) continue;
    std::vector<double> zs; zs.reserve(pts.size());
    for (const auto &s : pts) zs.push_back(s.z);
    const auto k = static_cast<size_t>(p_.ground_quantile * (zs.size() - 1));
    std::nth_element(zs.begin(), zs.begin() + k, zs.end());
    raw[kv.first] = zs[k];
  }
  const int reach = static_cast<int>(std::ceil(p_.fit_radius / cell));
  const double seed_slope = std::tan(10. * M_PI / 180.);
  // 2. Robust local plane per cell with a ground sample.
  struct Fit { Plane plane; double residual = 0; bool consistent = false; };
  std::unordered_map<int64_t, Fit> fits;
  for (const auto &kv : raw) {
    const auto &c = coords_[kv.first];
    std::vector<std::array<double, 3>> s;
    for (int dx = -reach; dx <= reach; ++dx) for (int dy = -reach; dy <= reach; ++dy) {
      if ((dx * dx + dy * dy) * cell * cell > p_.fit_radius * p_.fit_radius) continue;
      const auto it = raw.find(key(c.first + dx, c.second + dy));
      if (it != raw.end()) s.push_back({dx * cell, dy * cell, it->second});
    }
    Fit f;
    // Seed with the cell's own level (a gentle rise allowed): on a step the far
    // level is left out instead of being averaged in; a ramp re-enters below.
    std::vector<char> use(s.size(), 0);
    for (size_t i = 0; i < s.size(); ++i)
      use[i] = std::abs(s[i][2] - kv.second) <= p_.fit_outlier + std::hypot(s[i][0], s[i][1]) * seed_slope;
    if (std::count(use.begin(), use.end(), 1) >= p_.min_fit_cells) {
      for (int iter = 0; iter < 3; ++iter) {
        f.plane = fit(s, use);
        if (!f.plane.valid) break;
        bool changed = false;
        for (size_t i = 0; i < s.size(); ++i) {
          const bool in = std::abs(s[i][2] - (f.plane.a * s[i][0] + f.plane.b * s[i][1] + f.plane.c)) <= p_.fit_outlier;
          if (static_cast<char>(in) != use[i]) { use[i] = in; changed = true; }
        }
        if (std::count(use.begin(), use.end(), 1) < p_.min_fit_cells) { f.plane.valid = false; break; }
        if (!changed) break;
      }
    }
    if (!f.plane.valid) {
      // Too few neighbours: a level plane at the median neighbour ground.
      std::vector<double> zs; for (const auto &q : s) zs.push_back(q[2]);
      std::nth_element(zs.begin(), zs.begin() + zs.size() / 2, zs.end());
      f.plane = Plane{0, 0, zs[zs.size() / 2], true};
    }
    f.residual = kv.second - f.plane.c;
    f.consistent = std::abs(f.residual) <= p_.fit_outlier;
    fits[kv.first] = f;
  }
  // 3. Steps. Two cells step_span apart each get a plane fitted only from cells
  // on their own side (so the two levels of a step never mix). Across a ramp
  // kink those planes cross somewhere between the cells; at a step they stay
  // apart by its height. The smallest gap along the segment is the step.
  auto side_plane = [&](int64_t cx, int64_t cy, int dx, int dy, int sign) {
    std::vector<std::array<double, 3>> s;
    for (int ox = -reach; ox <= reach; ++ox) for (int oy = -reach; oy <= reach; ++oy) {
      if ((ox * ox + oy * oy) * cell * cell > p_.fit_radius * p_.fit_radius) continue;
      if (sign * (ox * dx + oy * dy) > 0) continue;  // keep own side (and the cross line)
      const auto it = raw.find(key(cx + ox, cy + oy));
      if (it != raw.end()) s.push_back({ox * cell, oy * cell, it->second});
    }
    Plane pl;
    if (static_cast<int>(s.size()) < p_.min_fit_cells) return pl;
    std::vector<char> use(s.size(), 1);
    for (int iter = 0; iter < 3; ++iter) {
      pl = fit(s, use);
      if (!pl.valid) return pl;
      bool changed = false;
      for (size_t i = 0; i < s.size(); ++i) {
        const bool in = std::abs(s[i][2] - (pl.a * s[i][0] + pl.b * s[i][1] + pl.c)) <= p_.fit_outlier;
        if (static_cast<char>(in) != use[i]) { use[i] = in; changed = true; }
      }
      if (std::count(use.begin(), use.end(), 1) < p_.min_fit_cells) { pl.valid = false; return pl; }
      if (!changed) break;
    }
    return pl;
  };
  std::unordered_set<int64_t> step_cells;
  const int span = std::max(2, static_cast<int>(std::lround(p_.step_span / cell)));
  for (const auto &kv : raw) {
    const auto &c = coords_[kv.first];
    for (const auto &d : {std::pair<int, int>{1, 0}, {0, 1}}) {
      const int64_t nx = c.first + d.first * span, ny = c.second + d.second * span;
      if (!raw.count(key(nx, ny))) continue;
      const Plane a = side_plane(c.first, c.second, d.first, d.second, +1);   // behind c
      const Plane b = side_plane(nx, ny, d.first, d.second, -1);              // beyond n
      if (!a.valid || !b.valid) continue;
      double gap = 1e9, prev = 0, at_gap = 0;
      bool crossed = false;
      for (int i = 0; i <= span; ++i) {
        const double ux = i * d.first * cell, uy = i * d.second * cell;   // from c
        const double za = a.a * ux + a.b * uy + a.c;
        const double zb = b.a * (ux - span * d.first * cell) + b.b * (uy - span * d.second * cell) + b.c;
        const double diff = za - zb;
        if (i > 0 && diff * prev <= 0) crossed = true;
        prev = diff;
        if (std::abs(diff) < gap) { gap = std::abs(diff); at_gap = diff; }
      }
      if (crossed || gap <= p_.max_step) continue;
      if (!p_.consider_drop) {
        // Rises only: skip when the side farther from the robot is the lower one.
        const double dc = std::hypot((c.first + .5) * cell - pose.x, (c.second + .5) * cell - pose.y);
        const double dn = std::hypot((nx + .5) * cell - pose.x, (ny + .5) * cell - pose.y);
        if ((dn > dc) == (at_gap > 0)) continue;
      }
      // The edge lies between the two fitted cells: mark only the inner cells,
      // not the whole span (that painted a 0.3 m band in front of every edge).
      for (int i = 1; i < span; ++i)
        step_cells.insert(key(c.first + d.first * i, c.second + d.second * i));
    }
  }
  // Drops: a forward camera cannot see the lower floor just past an edge, so it
  // leaves a shadow (no data). A ground cell followed, after unseen cells, by
  // ground lower by more than max_step is a drop edge. A descending ramp seen
  // from too far is self-occluded the same way and stays blocked until visible.
  const int shadow = std::max(2, static_cast<int>(std::lround(p_.drop_shadow / cell)));
  std::unordered_set<int64_t> drop_cells;
  for (const auto &kv : fits) {
    if (!p_.consider_drop || !kv.second.consistent) continue;
    const auto &c = coords_[kv.first];
    for (const auto &d : {std::pair<int, int>{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
      int k = 1;
      while (k <= shadow && !raw.count(key(c.first + d.first * k, c.second + d.second * k))) ++k;
      if (k < 2 || k > shadow) continue;  // no unseen gap, or nothing beyond it
      const auto nk = key(c.first + d.first * k, c.second + d.second * k);
      if (kv.second.plane.c - raw[nk] > p_.max_step) { step_cells.insert(kv.first); drop_cells.insert(kv.first); }
    }
  }
  auto surrounded = [&](int64_t cx, int64_t cy) {
    for (const auto &d : {std::pair<int, int>{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
      bool any = false;
      for (int k = 1; k <= 2 && !any; ++k) any = raw.count(key(cx + d.first * k, cy + d.second * k)) > 0;
      if (!any) return false;
    }
    return true;
  };
  // 4. Classify cells and points.
  struct Verdict { int64_t k; CellResult res; bool blocked; };
  std::vector<Verdict> verdicts;
  for (const auto k : keys) {
    auto &state = cells_[k];
    const auto &c = coords_[k];
    CellResult res; res.ix = c.first; res.iy = c.second; res.observed = state.observed;
    const auto fi = fits.find(k);
    bool blocked = false;
    if (fi != fits.end()) {
      const auto &f = fi->second;
      res.ground = f.plane.c;
      res.raw = raw[k];
      res.slope_deg = std::atan(std::hypot(f.plane.a, f.plane.b)) * 180. / M_PI;
      double jump = 0;
      if (f.consistent) {
        for (const auto &d : {std::pair<int, int>{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
          const auto nf = fits.find(key(c.first + d.first, c.second + d.second));
          if (nf == fits.end() || !nf->second.consistent) continue;
          jump = std::max(jump, std::abs(raw[key(c.first + d.first, c.second + d.second)] - raw[k]));
        }
      }
      res.step = std::max(jump, std::abs(f.residual));
      const double rx = (c.first + .5) * cell - pose.x, ry = (c.second + .5) * cell - pose.y;
      const bool resolvable = rx * rx + ry * ry <= p_.terrain_range * p_.terrain_range;
      // A residual is one cell against its neighbours: like slope and step it needs
      // depth noise below max_step, and enough points that the quantile is not one return.
      const bool sampled = static_cast<int>(state.points.size()) >= p_.min_residual_points;
      if (p_.consider_drop && resolvable && f.residual < -p_.max_step) { res.cls = CellClass::Step; res.reason = 1; }  // sunk
      else if (resolvable && sampled && f.residual > p_.max_step) { res.cls = CellClass::Step; res.reason = 2; }     // raised edge / box top
      // Slope, step and drop only where depth noise is below max_step.
      else if (!resolvable) res.cls = CellClass::Ground;
      else if (res.slope_deg > p_.max_slope_deg + p_.slope_margin_deg && surrounded(c.first, c.second))
        res.cls = CellClass::Steep;  // only with ground data all round: one-sided fits at the
                                     // edge of the data (far range, partly seen cells) tilt
      else if (step_cells.count(k)) { res.cls = CellClass::Step; res.reason = drop_cells.count(k) ? 4 : 3; }
      else res.cls = CellClass::Ground;
      int high = 0;
      for (const auto &s : state.points) {
        const double h = s.z - (f.plane.a * (s.x - (c.first + .5) * cell) + f.plane.b * (s.y - (c.second + .5) * cell) + f.plane.c);
        out.terrain.push_back({s.x, s.y, s.z, h});
        if (h >= p_.obstacle_height && h < p_.vehicle_height) ++high;
      }
      // Depth noise grows with range squared; an obstacle must lift a share of the
      // cell, not a couple of stray returns.
      if (res.cls == CellClass::Ground && high >= 3 && high >= .3 * state.points.size()) { res.cls = CellClass::Obstacle; res.reason = 5; }
      blocked = res.cls != CellClass::Ground;
    } else if (!state.points.empty()) {
      // Too few points for a ground sample: judge points against neighbours' plane.
      const Plane *plane = nullptr;
      for (int dx = -1; dx <= 1 && !plane; ++dx) for (int dy = -1; dy <= 1 && !plane; ++dy) {
        const auto nf = fits.find(key(c.first + dx, c.second + dy));
        if (nf != fits.end()) plane = &nf->second.plane;
      }
      if (plane) {
        int high = 0;
        for (const auto &s : state.points) {
          const double h = s.z - (plane->a * (s.x - (c.first + .5) * cell) + plane->b * (s.y - (c.second + .5) * cell) + plane->c);
          out.terrain.push_back({s.x, s.y, s.z, h});
          if (h >= p_.obstacle_height && h < p_.vehicle_height) ++high;
        }
        res.ground = plane->c;
        const bool lifted = high >= 3 && high >= .3 * state.points.size();
        res.cls = lifted ? CellClass::Obstacle : CellClass::Ground;
        blocked = lifted;
      }
    }
    verdicts.push_back({k, res, blocked});
  }
  // 5. A cell blocked by its ground alone (one-cell residual, step pair, tilted
  // fit) on its own is noise; real edges and slopes span several cells. Drop
  // edges are exempt: they already need an unseen gap and lower ground behind,
  // and a forward camera samples them sparsely along the edge.
  std::unordered_set<int64_t> blocked_keys;
  for (const auto &v : verdicts) if (v.blocked) blocked_keys.insert(v.k);
  for (auto &v : verdicts) {
    if (!v.blocked || v.res.cls == CellClass::Obstacle || v.res.reason == 4) continue;
    int support = 0;
    for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy)
      if ((dx || dy) && blocked_keys.count(key(v.res.ix + dx, v.res.iy + dy))) ++support;
    if (support < p_.min_blocked_neighbours) { v.blocked = false; v.res.cls = CellClass::Ground; v.res.reason = 0; }
  }
  // 6. Temporal confirmation; an unobserved cell keeps its last verdict.
  for (auto &v : verdicts) {
    const auto k = v.k;
    auto &state = cells_[k];
    const auto &c = coords_[k];
    auto &res = v.res;
    const bool blocked = v.blocked;
    const auto fi = fits.find(k);
    if (state.observed) state.confirm = blocked ? state.confirm + 1 : 0;
    state.was_obstacle = state.confirm >= p_.min_confirm;
    if (state.was_obstacle) {
      const Plane *plane = fi != fits.end() ? &fi->second.plane : nullptr;
      bool any = false;
      for (const auto &s : state.points) {
        const double base = plane ? plane->a * (s.x - (c.first + .5) * cell) + plane->b * (s.y - (c.second + .5) * cell) + plane->c : res.ground;
        const double h = s.z - base;
        if (h >= p_.obstacle_height && h < p_.vehicle_height) { out.obstacles.push_back({s.x, s.y, s.z, h}); any = true; }
      }
      if (!any) {
        // Steep ground, a step or a drop: the cell itself is not traversable.
        out.obstacles.push_back({(c.first + .5) * cell, (c.second + .5) * cell, res.ground, .2});
      }
    } else if (res.cls != CellClass::Unknown) {
      // Not (yet) confirmed: reported as ground; a single noisy frame never blocks.
      res.cls = CellClass::Ground;
    }
    out.cells.push_back(res);
  }
  return out;
}

}  // namespace wla_terrain
