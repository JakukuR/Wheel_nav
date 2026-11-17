#include "wla_uvc_depth/stereo_engine.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
void Require(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
int main(){try{
  const cv::Mat bgr(360,640,CV_8UC3,cv::Scalar(0,127,255));
  for(auto size:{cv::Size(320,192),cv::Size(640,384)}){
    auto input=wla::uvc::StereoInput(bgr,size);const size_t plane=size.area();
    Require(input.size()==3*plane,"CHW padded size");
    Require(std::abs(input[0]-(1.F-.485F)/.229F)<1e-6,"RGB channel order and 0..1 normalization");
    Require(std::abs(input[plane]-(127.F/255-.456F)/.224F)<1e-6,"green channel normalization");
    Require(std::abs(input[2*plane]-(-.406F)/.225F)<1e-6,"blue channel normalization");
    Require(input[0]==input[plane-1],"replicated pad preserves boundary content");
  }
  bool rejected=false;try{auto bad=wla::uvc::StereoInput(cv::Mat(480,640,CV_8UC3));(void)bad;}catch(const std::exception&){rejected=true;}
  Require(rejected,"uncalibrated input dimensions rejected");
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}std::cout<<"stereo input contract passed\n";return 0;}
