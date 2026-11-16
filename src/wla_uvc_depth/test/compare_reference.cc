#include "wla_uvc_depth/core.h"
#include <opencv2/imgcodecs.hpp>
#include <iostream>
int main(int argc,char** argv) {
  if(argc!=7){std::cerr<<"rig left right rgb output-prefix sgbm|bm\n";return 2;}
  try {
    cv::setNumThreads(1);wla::uvc::Options options;options.algorithm=argv[6];wla::uvc::Core core(wla::uvc::LoadCalibration(argv[1]),options);
    const auto result=core.depth(cv::imread(argv[2]),cv::imread(argv[3]));const auto color=core.color(cv::imread(argv[4]));
    const std::string prefix=argv[5];cv::imwrite(prefix+"_depth.png",result.depth);cv::imwrite(prefix+"_color.png",color);
    cv::imwrite(prefix+"_left.png",result.left);cv::imwrite(prefix+"_right.png",result.right);
    std::cout<<"native OpenCV "<<CV_VERSION<<'\n';
    cv::FileStorage file(prefix+"_disparity.yaml",cv::FileStorage::WRITE);file<<"disparity"<<result.disparity;
    file<<"R1"<<core.leftR()<<"Q"<<core.reprojectionMatrix();
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;
}
