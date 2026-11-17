#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include <cstdint>
namespace wla::uvc {
// Camera optical frame: x right, y down, z forward. A fresh grid for EACH frame.
struct FieldGrid {
  int nx=60,ny=40,nz=60;
  float resolution=.05F,x0=-1.5F,y0=-1.F,z0=0.F,maxDistance=1.F;
  std::vector<uint8_t> state; // 0 unknown, 1 observed free, 2 surface voxel
  std::vector<float> distance; // signed voxel-centre distance; NaN for unknown
  size_t index(int x,int y,int z) const {return (size_t(z)*ny+y)*nx+x;}
};
class SnapshotField {
 public:
  SnapshotField();
  const FieldGrid& update(const cv::Mat& depthMm,const cv::Matx33d& k);
 private:
  void transform(std::vector<float>& values);
  FieldGrid grid_;
  std::vector<float> occupied_,free_,line_,output_,boundary_;
  std::vector<int> sites_;
};
}
