#include "wla_uvc_depth/core.h"
#include <yaml-cpp/yaml.h>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace wla::uvc {
namespace {
cv::Mat Matrix(const YAML::Node& node,int rows,int cols) {
  if(!node.IsSequence()||static_cast<int>(node.size())!=rows) throw std::runtime_error("invalid_matrix_rows");
  cv::Mat value(rows,cols,CV_64F);
  for(int r=0;r<rows;++r) {
    if(!node[r].IsSequence()||static_cast<int>(node[r].size())!=cols) throw std::runtime_error("invalid_matrix_columns");
    for(int c=0;c<cols;++c) value.at<double>(r,c)=node[r][c].as<double>();
  }
  if(!cv::checkRange(value)) throw std::runtime_error("nonfinite_matrix");
  return value;
}
void Rigid(const cv::Mat& t) {
  if(t.rows!=4||t.cols!=4||t.type()!=CV_64F||!cv::checkRange(t))throw std::runtime_error("invalid_rigid_transform");
  const auto r=t(cv::Rect(0,0,3,3));
  if(cv::norm(r*r.t()-cv::Mat::eye(3,3,CV_64F))>1e-5||std::abs(cv::determinant(r)-1.)>1e-5||
     cv::norm(t.row(3)-(cv::Mat_<double>(1,4)<<0,0,0,1))>1e-8)throw std::runtime_error("invalid_rigid_transform");
}
void CheckImage(const cv::Mat& image,cv::Size size) {
  if(image.size()!=size||(image.type()!=CV_8UC1&&image.type()!=CV_8UC3))throw std::runtime_error("raw_image_size_or_encoding_changed");
}
}
Calibration LoadCalibration(const std::string& path) {
  const auto data=YAML::LoadFile(path);
  if(data["schema"].as<std::string>()!="wla-uvc-rig/v1"||data["units"].as<std::string>()!="m")throw std::runtime_error("rig_schema_or_units");
  Calibration result;
  const std::array<std::string,3> names={"left","right","rgb"};
  for(int i=0;i<3;++i) {
    const auto n=data["cameras"][names[i]];auto& c=result.cameras[i];
    c.size={n["width"].as<int>(),n["height"].as<int>()};c.k=Matrix(n["k"],3,3);
    if(n["model"].as<std::string>()!="pinhole-radtan"||!n["d"].IsSequence()||(n["d"].size()!=5&&n["d"].size()!=8))throw std::runtime_error("invalid_camera_model");
    c.d=cv::Mat(1,static_cast<int>(n["d"].size()),CV_64F);
    for(int j=0;j<c.d.cols;++j)c.d.at<double>(0,j)=n["d"][j].as<double>();
    if(!cv::checkRange(c.d)||c.k.at<double>(0,0)<=0||c.k.at<double>(1,1)<=0||
      cv::norm(c.k.row(2)-(cv::Mat_<double>(1,3)<<0,0,1))>1e-8||
      std::abs(c.k.at<double>(0,1))+std::abs(c.k.at<double>(1,0))>1e-8)throw std::runtime_error("invalid_camera_intrinsics");
  }
  const auto size=result.cameras[0].size;
  if((size!=cv::Size(1280,720)&&size!=cv::Size(720,1280))||result.cameras[1].size!=size||result.cameras[2].size!=cv::Size(1920,1080))throw std::runtime_error("calibration_resolution_mismatch");
  result.rightFromLeft=Matrix(data["T_right_left"],4,4);result.rgbFromLeft=Matrix(data["T_rgb_left"],4,4);
  Rigid(result.rightFromLeft);Rigid(result.rgbFromLeft);
  if(cv::norm(result.rightFromLeft(cv::Rect(3,0,1,3)))<1e-4)throw std::runtime_error("missing_stereo_baseline");
  return result;
}
cv::Mat CropMatrix(cv::Size output) {
  if(output.width<=0||output.height<=0)throw std::runtime_error("invalid_crop_size");
  const int rw=std::min(1920,1080*output.width/output.height);
  const int rh=std::min(1080,1920*output.height/output.width);
  const int x=(1920-rw)/2,y=(1080-rh)/2;
  const double sx=static_cast<double>(output.width)/rw,sy=static_cast<double>(output.height)/rh;
  return (cv::Mat_<double>(3,3)<<sx,0,sx*(.5-x)-.5,0,sy,sy*(.5-y)-.5,0,0,1);
}
cv::Mat RawColor(const cv::Mat& image,cv::Size output) {
  CheckImage(image,{1920,1080});
  const int rw=std::min(1920,1080*output.width/output.height),rh=std::min(1080,1920*output.height/output.width);
  cv::Mat result;cv::resize(image(cv::Rect((1920-rw)/2,(1080-rh)/2,rw,rh)),result,output,0,0,cv::INTER_AREA);return result;
}
cv::Mat ProjectDepth(const cv::Mat& points,const cv::Mat& k,const cv::Mat& transform,cv::Size output,double near_m,double far_m) {
  if((points.type()!=CV_32FC3&&points.type()!=CV_64FC3)||k.rows!=3||k.cols!=3||k.type()!=CV_64F||output.width<=0||output.height<=0||
     !std::isfinite(near_m)||!std::isfinite(far_m)||near_m<=0||far_m<=near_m||far_m>65.)throw std::runtime_error("invalid_depth_projection");
  Rigid(transform);
  const cv::Matx44d t(transform);const cv::Matx33d kk(k);
  cv::Mat zbuffer(output,CV_32F,cv::Scalar(std::numeric_limits<float>::infinity()));
  for(int r=0;r<points.rows;++r)for(int c=0;c<points.cols;++c) {
    const cv::Vec3d v=points.type()==CV_64FC3?points.at<cv::Vec3d>(r,c):cv::Vec3d(points.at<cv::Vec3f>(r,c));
    if(!std::isfinite(v[0])||!std::isfinite(v[1])||!std::isfinite(v[2]))continue;
    const auto p=t*cv::Vec4d(v[0],v[1],v[2],1.);const double z=p[2];
    if(z<near_m||z>far_m)continue;
    const auto uv=kk*cv::Vec3d(p[0],p[1],z);
    const double x=std::nearbyint(uv[0]/uv[2]),y=std::nearbyint(uv[1]/uv[2]);
    if(!std::isfinite(x)||!std::isfinite(y)||x<0||y<0||x>=output.width||y>=output.height)continue;
    auto& cell=zbuffer.at<float>(static_cast<int>(y),static_cast<int>(x));cell=std::min(cell,static_cast<float>(z));
  }
  cv::Mat depth(output,CV_16U,cv::Scalar(0));
  for(int r=0;r<output.height;++r)for(int c=0;c<output.width;++c) {
    const auto z=zbuffer.at<float>(r,c);if(std::isfinite(z))depth.at<uint16_t>(r,c)=static_cast<uint16_t>(std::nearbyint(z*1000.F));
  }
  return depth;
}
Core::Core(Calibration calibration,Options options):m_calibration(std::move(calibration)),m_options(std::move(options)) {
  if((m_options.rgbMode!="crop"&&m_options.rgbMode!="full")||(m_options.algorithm!="sgbm"&&m_options.algorithm!="bm")||
    m_options.disparities<16||m_options.disparities>=640||m_options.disparities%16||m_options.block<5||m_options.block>51||m_options.block%2==0||
    !std::isfinite(m_options.nearM)||!std::isfinite(m_options.farM)||m_options.nearM<=0||m_options.farM<=m_options.nearM||m_options.farM>65.)throw std::runtime_error("invalid_matcher_or_range");
  m_output=m_options.rgbMode=="crop"?cv::Size(640,480):cv::Size(640,360);
  const auto& cl=m_calibration.cameras[0];const auto& cr=m_calibration.cameras[1];const auto& cc=m_calibration.cameras[2];
  const auto& t=m_calibration.rightFromLeft;Rigid(t);Rigid(m_calibration.rgbFromLeft);
  cv::stereoRectify(cl.k,cl.d,cr.k,cr.d,cl.size,t(cv::Rect(0,0,3,3)),t(cv::Rect(3,0,1,3)),m_r1,m_r2,m_p1,m_p2,m_q,cv::CALIB_ZERO_DISPARITY,0.,{640,360});
  if(m_p2.at<double>(0,3)>=0||std::abs(m_p2.at<double>(1,3))>1e-5)throw std::runtime_error("requires_horizontal_positive_disparity_check_eye_order");
  cv::initUndistortRectifyMap(cl.k,cl.d,m_r1,m_p1,{640,360},CV_32FC1,m_lx,m_ly);
  cv::initUndistortRectifyMap(cr.k,cr.d,m_r2,m_p2,{640,360},CV_32FC1,m_rx,m_ry);
  m_colorK=CropMatrix(m_output)*cc.k;
  cv::initUndistortRectifyMap(cc.k,cc.d,cv::Mat::eye(3,3,CV_64F),m_colorK,m_output,CV_32FC1,m_cx,m_cy);
  if(m_options.algorithm=="sgbm")m_matcher=cv::StereoSGBM::create(0,m_options.disparities,m_options.block,8*m_options.block*m_options.block,
    32*m_options.block*m_options.block,1,0,12,100,2,cv::StereoSGBM::MODE_SGBM);
  else {
    auto bm=cv::StereoBM::create(m_options.disparities,m_options.block);bm->setUniquenessRatio(12);bm->setSpeckleWindowSize(100);bm->setSpeckleRange(2);bm->setDisp12MaxDiff(1);m_matcher=bm;
  }
}
cv::Mat Core::color(const cv::Mat& rgb) const {
  CheckImage(rgb,{1920,1080});cv::Mat result;cv::remap(rgb,result,m_cx,m_cy,cv::INTER_LINEAR);return result;
}
Result Core::depth(const cv::Mat& left,const cv::Mat& right) {
  CheckImage(left,stereoSize());CheckImage(right,stereoSize());Result result;
  cv::remap(left,result.left,m_lx,m_ly,cv::INTER_LINEAR);cv::remap(right,result.right,m_rx,m_ry,cv::INTER_LINEAR);
  if(result.left.channels()==3)cv::cvtColor(result.left,result.left,cv::COLOR_BGR2GRAY);
  if(result.right.channels()==3)cv::cvtColor(result.right,result.right,cv::COLOR_BGR2GRAY);
  cv::Mat raw;m_matcher->compute(result.left,result.right,raw);raw.convertTo(result.disparity,CV_32F,1./16.);
  cv::Mat rectified_points,points;cv::reprojectImageTo3D(result.disparity,rectified_points,m_q);
  rectified_points.convertTo(points,CV_64FC3);
  const cv::Mat inverse_matrix=m_r1.t();
  const cv::Matx33d inverse_rotation(inverse_matrix);
  for(int r=0;r<points.rows;++r)for(int c=0;c<points.cols;++c) {
    auto& p=points.at<cv::Vec3d>(r,c);
    if(result.disparity.at<float>(r,c)<=0||!std::isfinite(p[2])||p[2]<=0)p=cv::Vec3d(std::numeric_limits<double>::quiet_NaN(),0,0);
    else p=inverse_rotation*p;
  }
  result.depth=ProjectDepth(points,m_colorK,m_calibration.rgbFromLeft,m_output,m_options.nearM,m_options.farM);return result;
}
}
