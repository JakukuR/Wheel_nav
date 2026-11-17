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
  for(const std::string key:{"light_ess","ess","las2_s","las2_m","lightstereo_s"}){
    const auto model=wla::uvc::ModelSettings(key);const auto input=wla::uvc::StereoInput(bgr,model);const size_t plane=model.network.area();
    Require(input.size()==plane*3,"model CHW shape");
    if(key=="light_ess"||key=="ess"){
      Require(std::abs(input[0]-1.F)<1e-6&&std::abs(input[2*plane]+1.F)<1e-6,"ESS RGB minus-one-one normalization");
    }else if(key=="las2_s"||key=="las2_m")Require(input[0]==255.F&&input[2*plane]==0.F,"LAS2 raw RGB normalization");
    cv::Mat output(model.network,CV_32F,cv::Scalar(8.F*model.content.width/640.F));
    cv::Mat confidence(model.network,CV_32F,cv::Scalar(1));
    const auto d=wla::uvc::StereoOutput(output,confidence,model);
    Require(d.size()==cv::Size(640,360)&&std::abs(d.at<float>(180,320)-8.F)<1e-5,"disparity resize uses horizontal pixel scale");
    confidence.setTo(.1F);output.setTo(12.F);
    const auto masked=wla::uvc::StereoOutput(output,confidence,model);
    if(!model.confidence.empty())Require(cv::countNonZero(masked)==0,"ESS low-confidence depth remains invalid");
    confidence.setTo(1.F);output.setTo(0.F);
    Require(cv::countNonZero(wla::uvc::StereoOutput(output,confidence,model))==0,"zero disparity remains invalid");
  }
  const auto small=wla::uvc::ModelSettings("las2_s",{384,288});
  Require(small.content==cv::Size(384,216)&&small.padTop==36&&small.padBottom==36,"16:9 content with symmetric padding");
  cv::Mat nativeImage(216,384,CV_8UC3,cv::Scalar(0,127,255));
  auto smallInput=wla::uvc::StereoInput(nativeImage,small);
  Require(smallInput.size()==384*288*3&&smallInput[0]==255.F,"native LAS2 input accepted");
  cv::Mat smallOutput(288,384,CV_32F,cv::Scalar(12));smallOutput.rowRange(0,36).setTo(999);smallOutput.rowRange(252,288).setTo(999);
  const auto nativeDisp=wla::uvc::StereoOutput(smallOutput,{},small,{384,216});
  Require(nativeDisp.size()==cv::Size(384,216)&&cv::norm(nativeDisp-12,cv::NORM_INF)==0,"padding removed without scaling native disparity");
  auto ess=wla::uvc::ModelSettings("light_ess");cv::Mat holes(ess.network,CV_32F,cv::Scalar(10.F)),conf(ess.network,CV_32F,cv::Scalar(1.F));
  conf(cv::Rect(200,100,20,20)).setTo(.1F);
  const auto hole=wla::uvc::StereoOutput(holes,conf,ess);
  Require(hole.at<float>(137,280)==0.F&&hole.at<float>(40,40)>0,"confidence holes are not filled by resize");
  bool rejected=false;try{auto bad=wla::uvc::StereoInput(cv::Mat(480,640,CV_8UC3));(void)bad;}catch(const std::exception&){rejected=true;}
  Require(rejected,"uncalibrated input dimensions rejected");
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}std::cout<<"stereo input contract passed\n";return 0;}
