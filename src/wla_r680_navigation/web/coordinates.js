// A local Mercator plane anchored at (0,0); one ROS metre = one rendered metre.
// Using the equator avoids latitude-dependent height/XY scale discrepancies.
const R = 6378137, D = 180 / Math.PI;
export function toLngLat(x, y) {
  return [x / R * D, Math.atan(Math.sinh(y / R)) * D];
}
export function toRos(lng, lat) {
  return [lng / D * R, Math.asinh(Math.tan(lat / D)) * R];
}
export function mapCorners(m) {
  const c = Math.cos(m.origin_yaw || 0), s = Math.sin(m.origin_yaw || 0);
  const w = m.width * m.resolution, h = m.height * m.resolution;
  return [[0,h],[w,h],[w,0],[0,0]].map(([x,y]) =>
    toLngLat(m.origin_x + c*x - s*y, m.origin_y + s*x + c*y));
}
export function headingDegrees(yaw) { return 90 - yaw * D; }
