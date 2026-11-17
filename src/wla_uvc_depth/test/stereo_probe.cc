#include "wla_uvc_depth/stereo_engine.h"
#include "wla_uvc_depth/core.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <cmath>
int main(int argc,char** argv) {
  if(argc!=6){std::cerr<<"engine rig.yaml left.png right.png output_directory\n";return 2;}
  try {
    cv::setNumThreads(1);const std::filesystem::path out=argv[5];std::filesystem::create_directories(out);
    wla::uvc::Options opt;opt.algorithm="none";opt.rgbMode="full";
    wla::uvc::Core core(wla::uvc::LoadCalibration(argv[2]),opt);
    auto [left,right]=core.rectify(cv::imread(argv[3]),cv::imread(argv[4]));
    cv::imwrite((out/"left_rect.png").string(),left);cv::imwrite((out/"right_rect.png").string(),right);
    wla::uvc::StereoEngine engine(argv[1]);auto result=engine.infer(left,right);
    std::vector<double> wall,gpu;
    for(int i=0;i<10;++i){const auto start=std::chrono::steady_clock::now();result=engine.infer(left,right);wall.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());gpu.push_back(result.gpuMs);}
    const auto points=core.pointsFromDisparity(result.disparity);cv::Mat depth(points.size(),CV_16U,cv::Scalar(0));
    size_t finite=0,valid=0;double minimum=1e20,maximum=-1e20;
    for(int y=0;y<points.rows;++y)for(int x=0;x<points.cols;++x){
      const float d=result.disparity.at<float>(y,x);if(std::isfinite(d)){++finite;minimum=std::min(minimum,double(d));maximum=std::max(maximum,double(d));}
      const auto p=points.at<cv::Vec3f>(y,x);if(std::isfinite(p[0])&&std::isfinite(p[2])&&p[2]>=.15&&p[2]<=5){++valid;depth.at<uint16_t>(y,x)=static_cast<uint16_t>(std::nearbyint(p[2]*1000));}
    }
    cv::Mat gray,preview;depth.convertTo(gray,CV_8U,-255./5000,255);cv::applyColorMap(gray,preview,cv::COLORMAP_TURBO);preview.setTo(0,depth==0);
    cv::imwrite((out/"depth.png").string(),depth);cv::imwrite((out/"depth_preview.jpg").string(),preview);
    cv::Mat comparison;cv::hconcat(std::vector<cv::Mat>{left,right,preview},comparison);cv::imwrite((out/"comparison.jpg").string(),comparison);
    std::ofstream raw(out/"disparity.bin",std::ios::binary);raw.write(reinterpret_cast<const char*>(result.disparity.ptr<float>()),result.disparity.total()*sizeof(float));
    nlohmann::json j={{"wall_ms",wall},{"gpu_ms",gpu},{"finite_disparity_fraction",double(finite)/points.total()},{"valid_depth_fraction",double(valid)/points.total()},{"disparity_min",minimum},{"disparity_max",maximum},{"rgb_input_used",false}};
    std::ofstream(out/"result.json")<<j.dump(2)<<'\n';std::cout<<j.dump()<<'\n';
    if(finite!=points.total()||!valid)throw std::runtime_error("invalid_stereo_network_prediction");
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;
}
