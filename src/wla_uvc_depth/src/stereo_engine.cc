#include "wla_uvc_depth/stereo_engine.h"
#include "stereo_cuda.h"
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <opencv2/imgproc.hpp>
#include <dlfcn.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <cmath>
namespace wla::uvc {
namespace {
void Cuda(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
class Logger:public nvinfer1::ILogger {
 public:void log(Severity s,const char* text) noexcept override{if(s<=Severity::kWARNING)std::cerr<<"TensorRT: "<<text<<'\n';}
};
size_t Volume(nvinfer1::Dims shape){size_t n=1;for(int i=0;i<shape.nbDims;++i){if(shape.d[i]<=0)throw std::runtime_error("unresolved_model_shape");n*=shape.d[i];}return n;}
}
StereoModel ModelSettings(const std::string& key,cv::Size network){
  StereoModel s;s.key=key;
  if(key=="light_ess"||key=="ess"){
    s.name=key=="light_ess"?"Light ESS FP16":"Full ESS FP16";
    s.network=key=="light_ess"?cv::Size(480,288):cv::Size(960,576);s.content=s.network;
    s.left="input_left";s.right="input_right";s.disparity="output_left";s.confidence="output_conf";
    s.normalization="minus_one_one";s.confidenceThreshold=.4F;
  }else if(key=="cfast"||key=="las2_s"||key=="las2_m"||key=="lightstereo_s"){
    s.name=key=="cfast"?"C-Fast-FoundationStereo":key=="las2_s"?"LAS2-S FP16":key=="las2_m"?"LAS2-M FP16":"LightStereo-S General";
    s.network=network.area()?network:cv::Size(640,384);
    if(s.network!=cv::Size(640,384)&&(key!="cfast"||s.network!=cv::Size(320,192)))throw std::runtime_error("unsupported_stereo_network_resolution");
    s.content={s.network.width,s.network.width*360/640};s.padTop=(s.network.height-s.content.height)/2;s.padBottom=s.padTop;
    s.left=key=="cfast"?"left_image":"left";s.right=key=="cfast"?"right_image":"right";s.disparity="disparity";
    s.normalization=(key=="las2_s"||key=="las2_m")?"raw_rgb":"imagenet";
  }else throw std::runtime_error("unknown_stereo_model:"+key);
  if(network.area()&&network!=s.network)throw std::runtime_error("model_network_size_mismatch");
  return s;
}
std::vector<float> StereoInput(const cv::Mat& image,const StereoModel& s){
  if(image.size()!=cv::Size(640,360)||(image.type()!=CV_8UC1&&image.type()!=CV_8UC3))throw std::runtime_error("stereo_input_requires_rectified_640x360");
  cv::Mat rgb,padded;cv::cvtColor(image,rgb,image.channels()==1?cv::COLOR_GRAY2RGB:cv::COLOR_BGR2RGB);
  if(rgb.size()!=s.content)cv::resize(rgb,rgb,s.content,0,0,s.key=="cfast"?cv::INTER_AREA:cv::INTER_LINEAR);
  cv::copyMakeBorder(rgb,padded,s.padTop,s.padBottom,0,0,cv::BORDER_REPLICATE);
  constexpr float MEAN[3]={.485F,.456F,.406F},STD[3]={.229F,.224F,.225F};
  const size_t plane=padded.total();std::vector<float> result(plane*3);
  for(int y=0;y<padded.rows;++y)for(int x=0;x<padded.cols;++x)for(int c=0;c<3;++c){
    float v=padded.at<cv::Vec3b>(y,x)[c];
    if(s.normalization=="minus_one_one")v=(v-127.5F)/127.5F;
    else if(s.normalization=="imagenet")v=(v/255.F-MEAN[c])/STD[c];
    else if(s.normalization!="raw_rgb")throw std::runtime_error("unknown_stereo_normalization");
    result[c*plane+y*padded.cols+x]=v;
  }
  return result;
}
std::vector<float> StereoInput(const cv::Mat& image,cv::Size size){return StereoInput(image,ModelSettings("cfast",size));}
cv::Mat StereoOutput(const cv::Mat& output,const cv::Mat& confidence,const StereoModel& s){
  if(output.type()!=CV_32FC1||output.size()!=s.network||(!confidence.empty()&&(confidence.type()!=CV_32FC1||confidence.size()!=s.network))||(!s.confidence.empty()&&confidence.empty()))throw std::runtime_error("invalid_stereo_output_shape");
  const cv::Rect roi(0,s.padTop,s.content.width,s.content.height);cv::Mat d=output(roi).clone(),valid(s.content,CV_32F);
  for(int y=0;y<d.rows;++y)for(int x=0;x<d.cols;++x){
    float& value=d.at<float>(y,x);const float c=confidence.empty()?1.F:confidence.at<float>(y+s.padTop,x);
    const bool ok=std::isfinite(value)&&value>0&&std::isfinite(c)&&c>=s.confidenceThreshold;
    valid.at<float>(y,x)=ok?1.F:0.F;if(!ok)value=0;
  }
  if(d.size()!=cv::Size(640,360)){
    cv::resize(d,d,{640,360},0,0,cv::INTER_LINEAR);d*=640.F/s.content.width;
    cv::resize(valid,valid,{640,360},0,0,cv::INTER_LINEAR);
    // All contributing source neighbours must be valid; interpolation must not fill holes.
    d.setTo(0,valid<.999F);
  }
  return d;
}
struct StereoEngine::Impl {
  StereoModel model;void* plugin=nullptr;
  Logger logger;std::unique_ptr<nvinfer1::IRuntime> runtime;
  std::unique_ptr<nvinfer1::ICudaEngine> engine;std::unique_ptr<nvinfer1::IExecutionContext> context;
  cudaStream_t stream=nullptr;cudaEvent_t begin=nullptr,end=nullptr;
  void* left=nullptr;void* right=nullptr;void* output=nullptr;void* confidence=nullptr;
  uint8_t* packed=nullptr;float* maps=nullptr;float* points=nullptr;uint16_t* depth=nullptr;uint8_t* preview=nullptr;
  void* pinned=nullptr;GpuGeometry geometry{};bool configured=false;
  ~Impl(){
    if(stream)cudaStreamSynchronize(stream);
    context.reset();engine.reset();runtime.reset();
    if(packed)cudaFree(packed);
    if(maps)cudaFree(maps);
    if(points)cudaFree(points);
    if(depth)cudaFree(depth);
    if(preview)cudaFree(preview);
    if(pinned)cudaFreeHost(pinned);
    if(left)cudaFree(left);
    if(right)cudaFree(right);
    if(output)cudaFree(output);
    if(confidence)cudaFree(confidence);
    if(begin)cudaEventDestroy(begin);
    if(end)cudaEventDestroy(end);
    if(stream)cudaStreamDestroy(stream);
    if(plugin)dlclose(plugin);
  }
};
StereoEngine::StereoEngine(const std::string& path,const std::string& key,const std::string& plugin):m_impl(std::make_unique<Impl>()){
  Cuda(cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync));
  auto& p=*m_impl;p.model=ModelSettings(key);
  if(!plugin.empty()){p.plugin=dlopen(plugin.c_str(),RTLD_NOW|RTLD_GLOBAL);if(!p.plugin)throw std::runtime_error(dlerror());}
  else if(!p.model.confidence.empty())throw std::runtime_error("ESS_requires_matching_plugin_library");
  std::ifstream file(path,std::ios::binary|std::ios::ate);if(!file)throw std::runtime_error("missing_stereo_engine");
  const auto bytes=file.tellg();if(bytes<=0||bytes>1024LL*1024*1024)throw std::runtime_error("invalid_stereo_engine_size");
  std::vector<char> data(static_cast<size_t>(bytes));file.seekg(0);file.read(data.data(),bytes);if(!file)throw std::runtime_error("truncated_stereo_engine");
  p.runtime.reset(nvinfer1::createInferRuntime(p.logger));if(!p.runtime)throw std::runtime_error("TensorRT_runtime_failed");
  p.engine.reset(p.runtime->deserializeCudaEngine(data.data(),data.size()));
  if(!p.engine||p.engine->getNbIOTensors()!=(p.model.confidence.empty()?3:4))throw std::runtime_error("unexpected_stereo_engine_IO_count");
  for(int i=0;i<p.engine->getNbIOTensors();++i){const auto name=p.engine->getIOTensorName(i);const std::string n=name;
    const bool input=n==p.model.left||n==p.model.right;
    if(p.engine->getTensorDataType(name)!=nvinfer1::DataType::kFLOAT||(!input&&n!=p.model.disparity&&n!=p.model.confidence)||(p.engine->getTensorIOMode(name)==nvinfer1::TensorIOMode::kINPUT)!=input)throw std::runtime_error("unexpected_stereo_engine_IO");
  }
  p.context.reset(p.engine->createExecutionContext());if(!p.context)throw std::runtime_error("stereo_context_failed");
  if(key=="cfast"){
    auto d=p.engine->getTensorShape(p.model.left.c_str());if(d.nbDims!=4)throw std::runtime_error("invalid_stereo_profile");
    if(d.d[2]<0||d.d[3]<0)d=p.engine->getProfileShape(p.model.left.c_str(),0,nvinfer1::OptProfileSelector::kOPT);
    p.model=ModelSettings(key,{int(d.d[3]),int(d.d[2])});
    const nvinfer1::Dims4 shape{1,3,p.model.network.height,p.model.network.width};
    for(const auto& n:{p.model.left,p.model.right})if(!p.context->setInputShape(n.c_str(),shape))throw std::runtime_error("stereo_profile_requires_matching_resolution");
  }
  const size_t plane=p.model.network.area();
  for(const auto& n:{p.model.left,p.model.right,p.model.disparity,p.model.confidence}){
    if(n.empty())continue;
    const bool input=n==p.model.left||n==p.model.right;const auto shape=p.context->getTensorShape(n.c_str());
    if(shape.nbDims<2||Volume(shape)!=plane*(input?3:1)||shape.d[shape.nbDims-1]!=p.model.network.width||shape.d[shape.nbDims-2]!=p.model.network.height||(input&&(shape.nbDims!=4||shape.d[0]!=1||shape.d[1]!=3)))throw std::runtime_error("stereo_model_shape_mismatch");
  }
  Cuda(cudaStreamCreate(&p.stream));Cuda(cudaEventCreate(&p.begin));Cuda(cudaEventCreate(&p.end));
  Cuda(cudaMalloc(&p.left,plane*3*sizeof(float)));Cuda(cudaMalloc(&p.right,plane*3*sizeof(float)));Cuda(cudaMalloc(&p.output,plane*sizeof(float)));
  if(!p.model.confidence.empty())Cuda(cudaMalloc(&p.confidence,plane*sizeof(float)));
  if(!p.context->setTensorAddress(p.model.left.c_str(),p.left)||!p.context->setTensorAddress(p.model.right.c_str(),p.right)||!p.context->setTensorAddress(p.model.disparity.c_str(),p.output)||(!p.model.confidence.empty()&&!p.context->setTensorAddress(p.model.confidence.c_str(),p.confidence)))throw std::runtime_error("stereo_tensor_binding_failed");
}
StereoEngine::~StereoEngine()=default;
cv::Size StereoEngine::networkSize()const noexcept{return m_impl->model.network;}
const StereoModel& StereoEngine::settings()const noexcept{return m_impl->model;}
StereoInference StereoEngine::infer(const cv::Mat& left,const cv::Mat& right){
  auto& p=*m_impl;const auto size=p.model.network;const size_t plane=size.area();
  auto a=StereoInput(left,p.model),b=StereoInput(right,p.model);cv::Mat padded(size,CV_32F),confidence;
  Cuda(cudaMemcpyAsync(p.left,a.data(),a.size()*sizeof(float),cudaMemcpyHostToDevice,p.stream));
  Cuda(cudaMemcpyAsync(p.right,b.data(),b.size()*sizeof(float),cudaMemcpyHostToDevice,p.stream));
  Cuda(cudaEventRecord(p.begin,p.stream));if(!p.context->enqueueV3(p.stream))throw std::runtime_error("stereo_enqueue_failed");Cuda(cudaEventRecord(p.end,p.stream));
  Cuda(cudaMemcpyAsync(padded.data,p.output,plane*sizeof(float),cudaMemcpyDeviceToHost,p.stream));
  if(p.confidence){confidence.create(size,CV_32F);Cuda(cudaMemcpyAsync(confidence.data,p.confidence,plane*sizeof(float),cudaMemcpyDeviceToHost,p.stream));}
  Cuda(cudaStreamSynchronize(p.stream));float ms=0;Cuda(cudaEventElapsedTime(&ms,p.begin,p.end));
  StereoInference result;result.disparity=StereoOutput(padded,confidence,p.model);result.gpuMs=ms;return result;
}
void StereoEngine::configurePacked(const std::array<cv::Mat,4>& maps,const cv::Mat& q,const cv::Mat& leftR){
 auto& p=*m_impl;if(p.configured)throw std::runtime_error("packed_geometry_already_configured");
 if((p.model.key!="las2_s"&&p.model.key!="las2_m")||p.model.network!=cv::Size(640,384))throw std::runtime_error("packed_path_requires_LAS2_640x384");
 for(const auto& m:maps)if(m.size()!=cv::Size(640,360)||m.type()!=CV_32F||!m.isContinuous())throw std::runtime_error("invalid_rectify_maps");
 if(q.size()!=cv::Size(4,4)||q.type()!=CV_64F||leftR.size()!=cv::Size(3,3)||leftR.type()!=CV_64F)throw std::runtime_error("invalid_gpu_geometry");
 Cuda(cudaMalloc(reinterpret_cast<void**>(&p.maps),4*640*360*sizeof(float)));for(int i=0;i<4;++i)Cuda(cudaMemcpy(p.maps+i*640*360,maps[i].data,640*360*sizeof(float),cudaMemcpyHostToDevice));
 for(int y=0;y<4;++y)for(int x=0;x<4;++x)p.geometry.q[y*4+x]=q.at<double>(y,x);
 for(int y=0;y<3;++y)for(int x=0;x<3;++x)p.geometry.inverseR[y*3+x]=leftR.at<double>(x,y);
 Cuda(cudaMalloc(reinterpret_cast<void**>(&p.packed),2560*720*3));Cuda(cudaMalloc(reinterpret_cast<void**>(&p.points),640*360*3*sizeof(float)));Cuda(cudaMalloc(reinterpret_cast<void**>(&p.depth),640*360*sizeof(uint16_t)));Cuda(cudaMalloc(reinterpret_cast<void**>(&p.preview),640*360*3));
 Cuda(cudaMallocHost(&p.pinned,640*384*sizeof(float)+640*360*(3*sizeof(float)+sizeof(uint16_t)+3)));
 p.configured=true;
}
StereoInference StereoEngine::inferPacked(const cv::Mat& raw){
 auto& p=*m_impl;if(!p.configured||raw.size()!=cv::Size(2560,720)||(raw.type()!=CV_8UC1&&raw.type()!=CV_8UC3)||!raw.isContinuous())throw std::runtime_error("invalid_packed_input");
 const size_t plane=640*360,network=640*384;auto* host=static_cast<uint8_t*>(p.pinned);
 float* disparity=reinterpret_cast<float*>(host);float* points=reinterpret_cast<float*>(host+network*4);auto* depth=reinterpret_cast<uint16_t*>(host+network*4+plane*12);auto* preview=host+network*4+plane*14;
 Cuda(cudaMemcpyAsync(p.packed,raw.data,raw.total()*raw.elemSize(),cudaMemcpyHostToDevice,p.stream));
 PrepareStereo(p.packed,raw.channels(),p.maps,static_cast<float*>(p.left),static_cast<float*>(p.right),p.preview,p.stream);Cuda(cudaGetLastError());
 Cuda(cudaEventRecord(p.begin,p.stream));if(!p.context->enqueueV3(p.stream))throw std::runtime_error("stereo_enqueue_failed");Cuda(cudaEventRecord(p.end,p.stream));
 ReprojectStereo(static_cast<float*>(p.output),p.geometry,p.points,p.depth,p.stream);Cuda(cudaGetLastError());
 Cuda(cudaMemcpyAsync(disparity,p.output,network*4,cudaMemcpyDeviceToHost,p.stream));Cuda(cudaMemcpyAsync(points,p.points,plane*12,cudaMemcpyDeviceToHost,p.stream));Cuda(cudaMemcpyAsync(depth,p.depth,plane*2,cudaMemcpyDeviceToHost,p.stream));Cuda(cudaMemcpyAsync(preview,p.preview,plane*3,cudaMemcpyDeviceToHost,p.stream));
 Cuda(cudaStreamSynchronize(p.stream));float ms=0;Cuda(cudaEventElapsedTime(&ms,p.begin,p.end));
 // Views belong to this engine and remain valid until the next inference.
 StereoInference result;result.disparity=cv::Mat(360,640,CV_32F,disparity+12*640);result.gpuMs=ms;
 result.points=cv::Mat(360,640,CV_32FC3,points);result.depth=cv::Mat(360,640,CV_16U,depth);result.leftRect=cv::Mat(360,640,CV_8UC3,preview);return result;
}
std::array<std::vector<float>,2> StereoEngine::packedInputs()const{
 auto& p=*m_impl;if(!p.configured)throw std::runtime_error("packed_geometry_not_configured");std::array<std::vector<float>,2> out;
 for(int i=0;i<2;++i){out[i].resize(640*384*3);Cuda(cudaMemcpy(out[i].data(),i?p.right:p.left,out[i].size()*sizeof(float),cudaMemcpyDeviceToHost));}return out;
}

}
