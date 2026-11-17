#include "wla_uvc_depth/stereo_engine.h"
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
namespace wla::uvc {
namespace {

void Cuda(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
class Logger:public nvinfer1::ILogger {
 public:void log(Severity s,const char* text) noexcept override{if(s<=Severity::kWARNING)std::cerr<<"TensorRT: "<<text<<'\n';}
};
bool Shape(nvinfer1::Dims d,int channels,cv::Size size){return d.nbDims==4&&d.d[0]==1&&d.d[1]==channels&&d.d[2]==size.height&&d.d[3]==size.width;}
}
std::vector<float> StereoInput(const cv::Mat& image,cv::Size network_size) {
  const int WIDTH=network_size.width,HEIGHT=network_size.height;
  if(network_size!=cv::Size(640,384)&&network_size!=cv::Size(320,192))throw std::runtime_error("unsupported_stereo_network_resolution");
  const int content_height=WIDTH*360/640,PAD=(HEIGHT-content_height)/2;
  const size_t PLANE=size_t(WIDTH)*HEIGHT;
  if(image.size()!=cv::Size(640,360)||(image.type()!=CV_8UC1&&image.type()!=CV_8UC3))throw std::runtime_error("stereo_input_requires_rectified_640x360");
  cv::Mat rgb,padded;
  cv::cvtColor(image,rgb,image.channels()==1?cv::COLOR_GRAY2RGB:cv::COLOR_BGR2RGB);
  if(WIDTH!=640)cv::resize(rgb,rgb,{WIDTH,content_height},0,0,cv::INTER_AREA);
  cv::copyMakeBorder(rgb,padded,PAD,PAD,0,0,cv::BORDER_REPLICATE);
  constexpr float MEAN[3]={.485F,.456F,.406F},STD[3]={.229F,.224F,.225F};
  std::vector<float> result(PLANE*3);
  for(int y=0;y<HEIGHT;++y)for(int x=0;x<WIDTH;++x)for(int c=0;c<3;++c)
    result[c*PLANE+y*WIDTH+x]=(padded.at<cv::Vec3b>(y,x)[c]/255.F-MEAN[c])/STD[c];
  return result;
}
struct StereoEngine::Impl {
  cv::Size size;
  Logger logger;std::unique_ptr<nvinfer1::IRuntime> runtime;
  std::unique_ptr<nvinfer1::ICudaEngine> engine;std::unique_ptr<nvinfer1::IExecutionContext> context;
  cudaStream_t stream=nullptr;cudaEvent_t begin=nullptr,end=nullptr;
  void* left=nullptr;void* right=nullptr;void* output=nullptr;
  ~Impl(){
    if(stream)cudaStreamSynchronize(stream);
    context.reset();engine.reset();runtime.reset();
    if(left)cudaFree(left);
    if(right)cudaFree(right);
    if(output)cudaFree(output);
    if(begin)cudaEventDestroy(begin);
    if(end)cudaEventDestroy(end);
    if(stream)cudaStreamDestroy(stream);
  }
};
StereoEngine::StereoEngine(const std::string& path):m_impl(std::make_unique<Impl>()) {
  auto& p=*m_impl;std::ifstream file(path,std::ios::binary|std::ios::ate);
  if(!file)throw std::runtime_error("missing_stereo_engine");
  const auto bytes=file.tellg();if(bytes<=0||bytes>1024LL*1024*1024)throw std::runtime_error("invalid_stereo_engine_size");
  std::vector<char> data(static_cast<size_t>(bytes));file.seekg(0);file.read(data.data(),bytes);if(!file)throw std::runtime_error("truncated_stereo_engine");
  p.runtime.reset(nvinfer1::createInferRuntime(p.logger));if(!p.runtime)throw std::runtime_error("TensorRT_runtime_failed");
  p.engine.reset(p.runtime->deserializeCudaEngine(data.data(),data.size()));if(!p.engine||p.engine->getNbIOTensors()!=3)throw std::runtime_error("requires_two_inputs_one_disparity");
  for(int i=0;i<3;++i){
    const auto name=p.engine->getIOTensorName(i);const std::string n=name;
    if(p.engine->getTensorDataType(name)!=nvinfer1::DataType::kFLOAT||
       (n!="left_image"&&n!="right_image"&&n!="disparity")||
       (p.engine->getTensorIOMode(name)==nvinfer1::TensorIOMode::kINPUT)!=(n!="disparity"))throw std::runtime_error("unexpected_stereo_engine_IO");
  }
  p.context.reset(p.engine->createExecutionContext());if(!p.context)throw std::runtime_error("stereo_context_failed");
  const auto profile=p.engine->getProfileShape("left_image",0,nvinfer1::OptProfileSelector::kOPT);
  if(profile.nbDims!=4||profile.d[0]!=1||profile.d[1]!=3)throw std::runtime_error("invalid_stereo_profile");
  p.size={int(profile.d[3]),int(profile.d[2])};
  if(p.size!=cv::Size(640,384)&&p.size!=cv::Size(320,192))throw std::runtime_error("unsupported_stereo_profile_resolution");
  const auto PLANE=size_t(p.size.width)*p.size.height;
  const nvinfer1::Dims4 shape{1,3,p.size.height,p.size.width};
  for(const char* name:{"left_image","right_image"})if(!p.context->setInputShape(name,shape))throw std::runtime_error("stereo_engine_requires_matching_fixed_profile");
  if(!Shape(p.context->getTensorShape("disparity"),1,p.size))throw std::runtime_error("stereo_output_shape_mismatch");
  Cuda(cudaStreamCreate(&p.stream));Cuda(cudaEventCreate(&p.begin));Cuda(cudaEventCreate(&p.end));
  Cuda(cudaMalloc(&p.left,PLANE*3*sizeof(float)));Cuda(cudaMalloc(&p.right,PLANE*3*sizeof(float)));Cuda(cudaMalloc(&p.output,PLANE*sizeof(float)));
  if(!p.context->setTensorAddress("left_image",p.left)||!p.context->setTensorAddress("right_image",p.right)||!p.context->setTensorAddress("disparity",p.output))throw std::runtime_error("stereo_tensor_binding_failed");
}
StereoEngine::~StereoEngine()=default;
cv::Size StereoEngine::networkSize() const noexcept{return m_impl->size;}
StereoInference StereoEngine::infer(const cv::Mat& left,const cv::Mat& right) {
  auto& p=*m_impl;const int WIDTH=p.size.width,HEIGHT=p.size.height;const size_t PLANE=size_t(WIDTH)*HEIGHT;
  auto a=StereoInput(left,p.size),b=StereoInput(right,p.size);cv::Mat padded(HEIGHT,WIDTH,CV_32F);
  Cuda(cudaMemcpyAsync(p.left,a.data(),a.size()*sizeof(float),cudaMemcpyHostToDevice,p.stream));
  Cuda(cudaMemcpyAsync(p.right,b.data(),b.size()*sizeof(float),cudaMemcpyHostToDevice,p.stream));
  Cuda(cudaEventRecord(p.begin,p.stream));if(!p.context->enqueueV3(p.stream))throw std::runtime_error("stereo_enqueue_failed");Cuda(cudaEventRecord(p.end,p.stream));
  Cuda(cudaMemcpyAsync(padded.ptr<float>(),p.output,PLANE*sizeof(float),cudaMemcpyDeviceToHost,p.stream));Cuda(cudaStreamSynchronize(p.stream));
  float ms=0;Cuda(cudaEventElapsedTime(&ms,p.begin,p.end));
  const int content_height=WIDTH*360/640,PAD=(HEIGHT-content_height)/2;
  cv::Mat disparity=padded(cv::Rect(0,PAD,WIDTH,content_height)).clone();
  if(WIDTH!=640){cv::resize(disparity,disparity,{640,360},0,0,cv::INTER_LINEAR);disparity*=640.F/WIDTH;}
  return {disparity,ms};
}
}
