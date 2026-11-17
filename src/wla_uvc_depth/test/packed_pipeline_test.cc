#include "wla_uvc_depth/core.h"
#include "wla_uvc_depth/stereo_engine.h"
#include "wla_uvc_depth/direct_capture.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iostream>
using Json=nlohmann::json;
int main(int argc,char** argv){
 if(argc!=5){std::cerr<<"engine rig.yaml camera_device output_dir\n";return 2;}
 try{
  cv::setNumThreads(1);const std::filesystem::path out=argv[4];std::filesystem::create_directories(out);
  std::shared_ptr<wla::uvc::JpegFrame> frame;
  if(std::filesystem::is_regular_file(argv[3])){
   frame=std::make_shared<wla::uvc::JpegFrame>();std::ifstream saved(argv[3],std::ios::binary);
   frame->bytes.assign(std::istreambuf_iterator<char>(saved),{});
  }else{wla::uvc::DirectCapture camera(argv[3]);const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
   while(std::chrono::steady_clock::now()<deadline&&!frame)frame=camera.latest();}
  if(!frame)throw std::runtime_error("no_camera_frame");
  std::ofstream jpeg(out/"packed.jpg",std::ios::binary);jpeg.write(reinterpret_cast<const char*>(frame->bytes.data()),frame->bytes.size());jpeg.close();
  auto raw=cv::imdecode(frame->bytes,cv::IMREAD_COLOR),gray=cv::imdecode(frame->bytes,cv::IMREAD_GRAYSCALE);
  cv::Mat grayBgr;cv::cvtColor(gray,grayBgr,cv::COLOR_GRAY2BGR);
  Json report;report["jpeg_color_vs_gray_max"]=cv::norm(raw,grayBgr,cv::NORM_INF);
  wla::uvc::Options options;options.algorithm="none";options.rgbMode="full";
  wla::uvc::Core core(wla::uvc::LoadCalibration(argv[2]),options);
  wla::uvc::StereoEngine engine(argv[1],"las2_s");engine.configurePacked(core.rectifyMaps(),core.reprojectionMatrix(),core.leftR());
  cv::Mat pattern(720,2560,CV_8UC3);cv::RNG rng(8192);rng.fill(pattern,cv::RNG::UNIFORM,0,256);
  for(const auto& [name,packed]:std::vector<std::pair<std::string,cv::Mat>>{{"real_color",raw},{"real_gray",gray},{"random_color",pattern}}){
   cv::Mat left,right;cv::flip(packed(cv::Rect(1280,0,1280,720)),left,-1);cv::flip(packed(cv::Rect(0,0,1280,720)),right,-1);
   auto rect=core.rectify(left,right);auto ref=engine.infer(rect.first,rect.second);
   auto got=engine.inferPacked(packed);auto inputs=engine.packedInputs();
   double inputMax=0,inputSum=0;size_t inputCount=0;
   for(int eye=0;eye<2;++eye){auto expected=wla::uvc::StereoInput(eye?rect.second:rect.first,engine.settings());for(size_t i=0;i<expected.size();++i){double d=std::abs(expected[i]-inputs[eye][i]);inputMax=std::max(inputMax,d);inputSum+=d;++inputCount;}}
   const auto referencePoints=core.pointsFromDisparity(got.disparity);double xyzMax=0;size_t mismatch=0,valid=0;int depthMax=0;
   for(int y=0;y<360;++y)for(int x=0;x<640;++x){auto a=referencePoints.at<cv::Vec3f>(y,x),b=got.points.at<cv::Vec3f>(y,x);if(std::isfinite(a[0])!=std::isfinite(b[0]))++mismatch;
    if(std::isfinite(a[0])&&std::isfinite(b[0])&&a[2]>=.15&&a[2]<=5){for(int c=0;c<3;++c)xyzMax=std::max(xyzMax,double(std::abs(a[c]-b[c])));++valid;}
    uint16_t expected=0;if(std::isfinite(a[0])&&std::isfinite(a[1])&&std::isfinite(a[2])&&a[2]>=.15&&a[2]<=5){cv::Vec3d z=cv::Matx33d(core.leftR())*cv::Vec3d(a);if(z[2]>=.15&&z[2]<=5)expected=uint16_t(std::nearbyint(z[2]*1000));}
    depthMax=std::max(depthMax,std::abs(int(expected)-int(got.depth.at<uint16_t>(y,x))));
   }
   const auto dispMax=cv::norm(ref.disparity,got.disparity,cv::NORM_INF);
   Json row={{"input_max",inputMax},{"input_mean",inputSum/inputCount},{"disparity_max_px",dispMax},{"xyz_max_m",xyzMax},{"valid_points_compared",valid},{"finite_mask_mismatch",mismatch},{"depth_max_mm",depthMax}};
   report[name]=row;std::cout<<name<<": "<<row.dump()<<'\n';
   if(inputMax!=0||dispMax>.001||xyzMax>.0001||mismatch||depthMax>1)throw std::runtime_error("packed_contract_mismatch:"+name);
  }
  const auto full=engine.inferPacked(gray).depth.clone();
  const auto minimal=engine.inferPacked(gray,false,false,false);
  if(!minimal.points.empty()||!minimal.leftRect.empty()||!minimal.disparity.empty()||cv::norm(full,minimal.depth,cv::NORM_INF)!=0)
    throw std::runtime_error("demand_driven_depth_contract_mismatch");
  report["depth_only_download_matches_full"]=true;
  std::ofstream(out/"contract.json")<<report.dump(2)<<'\n';
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;
}
