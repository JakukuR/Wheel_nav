#include "wla_uvc_depth/core.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace wla::uvc;
void Require(bool condition,const char* reason) {if(!condition)throw std::runtime_error(reason);}
Calibration Synthetic() {
  Calibration c;
  for(int i=0;i<3;++i) {
    c.cameras[i].size=i==2?cv::Size(1920,1080):cv::Size(1280,720);
    c.cameras[i].k=i==2?(cv::Mat_<double>(3,3)<<1000,0,960,0,1000,540,0,0,1):(cv::Mat_<double>(3,3)<<800,0,640,0,800,360,0,0,1);
    c.cameras[i].d=cv::Mat::zeros(1,5,CV_64F);
  }
  c.rightFromLeft=cv::Mat::eye(4,4,CV_64F);c.rightFromLeft.at<double>(0,3)=-.1;c.rgbFromLeft=cv::Mat::eye(4,4,CV_64F);return c;
}
int main() {
  try {
    const auto a=CropMatrix({640,480});const auto k=Synthetic().cameras[2].k;const cv::Mat transformed=a*k;
    Require(std::abs(transformed.at<double>(0,2)-((960-240+.5)*4./9.-.5))<1e-10,"crop pixel-center cx");
    Require(std::abs(transformed.at<double>(1,2)-((540+.5)*4./9.-.5))<1e-10,"crop pixel-center cy");
    cv::Mat points(1,5,CV_32FC3);points.at<cv::Vec3f>(0,0)={0,0,2};points.at<cv::Vec3f>(0,1)={0,0,1};points.at<cv::Vec3f>(0,2)={0,0,.14F};
    points.at<cv::Vec3f>(0,3)={std::numeric_limits<float>::quiet_NaN(),0,1};points.at<cv::Vec3f>(0,4)={0,0,6};
    const cv::Mat small_k=(cv::Mat_<double>(3,3)<<100,0,50,0,100,50,0,0,1);
    auto depth=ProjectDepth(points,small_k,cv::Mat::eye(4,4,CV_64F),{100,100});
    Require(depth.at<uint16_t>(50,50)==1000&&cv::countNonZero(depth)==1,"z buffer/near/far/nonfinite/units");
    cv::Mat t=cv::Mat::eye(4,4,CV_64F);t.at<double>(2,3)=1;
    const cv::Mat one(1,1,CV_32FC3,cv::Scalar(0,0,1));depth=ProjectDepth(one,small_k,t,{100,100});
    Require(depth.at<uint16_t>(50,50)==2000,"RGB optical transform");
    auto invalid=Synthetic();invalid.rightFromLeft.at<double>(0,3)=.1;bool rejected=false;
    try{Core core(invalid);}catch(const std::exception&){rejected=true;}Require(rejected,"reject swapped positive-disparity order");
    Options native;native.algorithm="none";native.rgbMode="full";Core network_geometry(Synthetic(),native);
    cv::Mat disparity(360,640,CV_32F,cv::Scalar(20));disparity.at<float>(0,0)=0;disparity.at<float>(0,1)=std::numeric_limits<float>::quiet_NaN();
    const auto metric=network_geometry.pointsFromDisparity(disparity);
    Require(std::abs(metric.at<cv::Vec3f>(180,320)[2]-2.F)<.01F,"network disparity to 2m without RGB");
    Require(!std::isfinite(metric.at<cv::Vec3f>(0,0)[0])&&!std::isfinite(metric.at<cv::Vec3f>(0,1)[0]),"invalid network disparity rejected");
    bool disabled=false;try{network_geometry.depth(cv::Mat(720,1280,CV_8U),cv::Mat(720,1280,CV_8U));}catch(const std::exception&){disabled=true;}
    Require(disabled,"none algorithm cannot execute classical matcher");
    const auto real=LoadCalibration(FIXTURE);Core measured(real);Require(measured.outputSize()==cv::Size(640,480),"measured rig supported");
    cv::Mat left(720,1280,CV_8U);cv::RNG rng(7);rng.fill(left,cv::RNG::UNIFORM,0,256);
    cv::Mat right=cv::Mat::zeros(left.size(),left.type());left(cv::Rect(40,0,1240,720)).copyTo(right(cv::Rect(0,0,1240,720)));
    for(const auto& algorithm:{"sgbm","bm"}) {
      Options options;options.algorithm=algorithm;Core core(Synthetic(),options);auto result=core.depth(left,right);
      std::vector<uint16_t> values;
      for(int r=0;r<result.depth.rows;++r)for(int c=0;c<result.depth.cols;++c)if(auto z=result.depth.at<uint16_t>(r,c))values.push_back(z);
      Require(values.size()>5000,"synthetic plane valid matches");std::nth_element(values.begin(),values.begin()+values.size()/2,values.end());
      Require(std::abs(static_cast<int>(values[values.size()/2])-2000)<=50,"BM/SGBM metric 2m plane");
      std::cout<<algorithm<<" plane valid="<<values.size()<<" median_mm="<<values[values.size()/2]<<'\n';
    }
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
  std::cout<<"geometry and matching checks passed\n";return 0;
}
