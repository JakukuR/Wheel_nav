#include <vpi/Context.h>
#include <vpi/Stream.h>
#include <vpi/Image.h>
#include <vpi/Status.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/StereoDisparity.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <filesystem>
void Check(VPIStatus s){if(s!=VPI_SUCCESS){char text[VPI_MAX_STATUS_MESSAGE_LENGTH];vpiGetLastStatusMessage(text,sizeof(text));throw std::runtime_error(std::string(vpiStatusGetName(s))+":"+text);}}
struct Resources{
 VPIStream stream=nullptr;VPIPayload payload=nullptr;VPIImage l=nullptr,r=nullptr,bl=nullptr,br=nullptr,disp=nullptr,pitch=nullptr,conf=nullptr;
 ~Resources(){if(stream)vpiStreamSync(stream);if(conf)vpiImageDestroy(conf);if(pitch&&pitch!=disp)vpiImageDestroy(pitch);if(disp)vpiImageDestroy(disp);if(bl)vpiImageDestroy(bl);if(br)vpiImageDestroy(br);if(l)vpiImageDestroy(l);if(r)vpiImageDestroy(r);if(payload)vpiPayloadDestroy(payload);if(stream)vpiStreamDestroy(stream);}
};
int main(int argc,char** argv){if(argc!=5&&argc!=6){std::cerr<<"cuda|ofa|ofa-pva-vic left_rect right_rect out_dir\n";return 2;}
 try{
 cv::setNumThreads(1);const std::string mode=argv[1];auto left=cv::imread(argv[2],cv::IMREAD_COLOR),right=cv::imread(argv[3],cv::IMREAD_COLOR);
 if(left.empty()||right.empty()||left.size()!=right.size())throw std::runtime_error("invalid_rectified_pair");
 const bool cuda=mode=="cuda",hybrid=mode=="ofa-pva-vic";
 if(argc==6&&(!cuda||std::string(argv[5])!="strict"))throw std::runtime_error("strict_option_requires_cuda_backend");
 if(!cuda&&mode!="ofa"&&!hybrid)throw std::runtime_error("invalid_backend");
 const uint64_t backend=cuda?VPI_BACKEND_CUDA:hybrid?(VPI_BACKEND_OFA|VPI_BACKEND_PVA|VPI_BACKEND_VIC):VPI_BACKEND_OFA;
 Resources v;Check(vpiStreamCreate(0,&v.stream));Check(vpiImageCreateWrapperOpenCVMat(left,0,&v.l));Check(vpiImageCreateWrapperOpenCVMat(right,0,&v.r));
 VPIStereoDisparityEstimatorCreationParams create;Check(vpiInitStereoDisparityEstimatorCreationParams(&create));create.maxDisparity=128;create.downscaleFactor=1;
 const auto format=cuda?VPI_IMAGE_FORMAT_Y8_ER:VPI_IMAGE_FORMAT_Y8_ER_BL;
 Check(vpiCreateStereoDisparityEstimator(backend,left.cols,left.rows,format,&create,&v.payload));
 Check(vpiImageCreate(left.cols,left.rows,cuda||hybrid?VPI_IMAGE_FORMAT_S16:VPI_IMAGE_FORMAT_S16_BL,0,&v.disp));
 if(cuda||hybrid){Check(vpiImageCreate(left.cols,left.rows,VPI_IMAGE_FORMAT_U16,0,&v.conf));v.pitch=v.disp;}else v.pitch=v.disp;
 {Check(vpiImageCreate(left.cols,left.rows,format,0,&v.bl));Check(vpiImageCreate(left.cols,left.rows,format,0,&v.br));}
 VPIStereoDisparityEstimatorParams p;Check(vpiInitStereoDisparityEstimatorParams(&p));
 if(hybrid)p.confidenceType=VPI_STEREO_CONFIDENCE_INFERENCE;
 if(cuda&&argc==6){p.confidenceThreshold=65023;p.uniqueness=.95F;}
 VPIConvertImageFormatParams convert;Check(vpiInitConvertImageFormatParams(&convert));
 std::vector<double> times;cv::Mat raw,confidence;
 for(int i=0;i<23;++i){const auto start=std::chrono::steady_clock::now();
  Check(vpiImageSetWrappedOpenCVMat(v.l,left));Check(vpiImageSetWrappedOpenCVMat(v.r,right));
  {Check(vpiSubmitConvertImageFormat(v.stream,VPI_BACKEND_CUDA,v.l,v.bl,&convert));Check(vpiSubmitConvertImageFormat(v.stream,VPI_BACKEND_CUDA,v.r,v.br,&convert));}
  Check(vpiSubmitStereoDisparityEstimator(v.stream,backend,v.payload,v.bl,v.br,v.disp,v.conf,&p));
  if(v.pitch!=v.disp)Check(vpiSubmitConvertImageFormat(v.stream,VPI_BACKEND_CUDA,v.disp,v.pitch,&convert));
  Check(vpiStreamSync(v.stream));
  VPIImageData data;Check(vpiImageLockData(v.pitch,VPI_LOCK_READ,VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR,&data));cv::Mat view;Check(vpiImageDataExportOpenCVMat(data,&view));raw=view.clone();Check(vpiImageUnlock(v.pitch));
  if(v.conf){Check(vpiImageLockData(v.conf,VPI_LOCK_READ,VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR,&data));Check(vpiImageDataExportOpenCVMat(data,&view));confidence=view.clone();Check(vpiImageUnlock(v.conf));}
  if(i>=3)times.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
 }
 cv::Mat disparity;raw.convertTo(disparity,CV_32F,1./32.); // VPI S16 Q10.5, not OpenCV SGBM /16.
 if(!confidence.empty())disparity.setTo(0,confidence<p.confidenceThreshold);
 std::filesystem::path out=argv[4];std::filesystem::create_directories(out);std::ofstream f(out/"disparity.bin",std::ios::binary);f.write(reinterpret_cast<const char*>(disparity.ptr<float>()),disparity.total()*sizeof(float));
 cv::Mat gray,preview;disparity.convertTo(gray,CV_8U,255./128.);cv::applyColorMap(gray,preview,cv::COLORMAP_TURBO);preview.setTo(0,disparity<=0);cv::imwrite((out/"disparity.jpg").string(),preview);
 nlohmann::json j={{"backend",mode},{"times_ms",times},{"input_width",left.cols},{"input_height",left.rows},{"max_disparity",128},{"disparity_scale",32},{"confidence_available",v.conf!=nullptr},{"confidence_threshold",p.confidenceThreshold},{"p1",p.p1},{"p2",p.p2},{"uniqueness",p.uniqueness},{"valid_fraction",double(cv::countNonZero(disparity>0))/disparity.total()}};
 std::ofstream(out/"result.json")<<j.dump(2)<<'\n';std::cout<<j.dump()<<'\n';
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;}
