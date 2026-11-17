#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <nlohmann/json.hpp>
#include <dlfcn.h>
#include <sys/resource.h>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <chrono>
#include <vector>
#include <memory>
#include <map>
#include <cmath>
using Json=nlohmann::json;
void Cuda(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
class Logger:public nvinfer1::ILogger {public:void log(Severity s,const char* m)noexcept override{if(s<=Severity::kWARNING)std::cerr<<m<<'\n';}};
std::vector<float> Input(const cv::Mat& bgr,const Json& s){
 cv::Mat rgb,scaled,padded;cv::cvtColor(bgr,rgb,cv::COLOR_BGR2RGB);
 const int w=s.at("content_width"),h=s.at("content_height"),pad=s.value("pad_top",0),bottom=s.value("pad_bottom",0);
 cv::resize(rgb,scaled,{w,h},0,0,cv::INTER_LINEAR);cv::copyMakeBorder(scaled,padded,pad,bottom,0,0,cv::BORDER_REPLICATE);
 const size_t plane=padded.total();std::vector<float> out(plane*3);const std::string norm=s.at("normalization");
 const float mean[3]={.485F,.456F,.406F},sd[3]={.229F,.224F,.225F};
 for(int y=0;y<padded.rows;++y)for(int x=0;x<padded.cols;++x)for(int c=0;c<3;++c){float v=padded.at<cv::Vec3b>(y,x)[c];
  if(norm=="minus_one_one")v=(v-127.5F)/127.5F;else if(norm=="imagenet")v=(v/255.F-mean[c])/sd[c];else if(norm!="raw_rgb")throw std::runtime_error("unknown_normalization");
  out[c*plane+y*padded.cols+x]=v;}
 return out;
}
int main(int argc,char** argv){if(argc!=5){std::cerr<<"spec.json left_rect right_rect out_dir\n";return 2;}
 try{cv::setNumThreads(1);Json s;std::ifstream(argv[1])>>s;const auto left=cv::imread(argv[2]),right=cv::imread(argv[3]);
 if(left.size()!=cv::Size(640,360)||right.size()!=left.size())throw std::runtime_error("requires_rectified_640x360_bgr_pair");
 const std::filesystem::path out=argv[4];std::filesystem::create_directories(out);
 void* plugin=nullptr;const auto plugin_path=s.value("plugin",std::string());if(!plugin_path.empty()){plugin=dlopen(plugin_path.c_str(),RTLD_NOW|RTLD_GLOBAL);if(!plugin)throw std::runtime_error(dlerror());}
 Logger logger;std::ifstream f(s.at("engine").get<std::string>(),std::ios::binary|std::ios::ate);if(!f)throw std::runtime_error("missing_engine");
 const size_t bytes=f.tellg();std::vector<char> plan(bytes);f.seekg(0);f.read(plan.data(),bytes);if(!f)throw std::runtime_error("truncated_engine");
 std::unique_ptr<nvinfer1::IRuntime> runtime(nvinfer1::createInferRuntime(logger));std::unique_ptr<nvinfer1::ICudaEngine> engine(runtime->deserializeCudaEngine(plan.data(),plan.size()));if(!engine)throw std::runtime_error("deserialize_failed");
 std::unique_ptr<nvinfer1::IExecutionContext> context(engine->createExecutionContext());if(!context)throw std::runtime_error("context_failed");
 const int w=s.at("width"),h=s.at("height");const size_t plane=size_t(w)*h;
 const auto ln=s.at("input_left").get<std::string>(),rn=s.at("input_right").get<std::string>(),dn=s.at("output_disparity").get<std::string>(),cn=s.value("output_confidence",std::string());
 std::map<std::string,void*> buffers;std::vector<std::string> names={ln,rn,dn};if(!cn.empty())names.push_back(cn);
 if(engine->getNbIOTensors()!=int(names.size()))throw std::runtime_error("unexpected_model_IO_count");
 for(const auto& n:names){if(engine->getTensorDataType(n.c_str())!=nvinfer1::DataType::kFLOAT)throw std::runtime_error("requires_float32_model_IO");auto shape=engine->getTensorShape(n.c_str());size_t count=1;for(int i=0;i<shape.nbDims;++i){if(shape.d[i]<=0)throw std::runtime_error("requires_static_model");count*=shape.d[i];}
  if(count!=plane*((n==ln||n==rn)?3:1))throw std::runtime_error("model_IO_shape_mismatch");
  Cuda(cudaMalloc(&buffers[n],count*sizeof(float)));if(!context->setTensorAddress(n.c_str(),buffers[n]))throw std::runtime_error("binding_failed");}
 cudaStream_t stream;cudaEvent_t begin,end;Cuda(cudaStreamCreate(&stream));Cuda(cudaEventCreate(&begin));Cuda(cudaEventCreate(&end));
 cv::Mat disparity(h,w,CV_32F),confidence(h,w,CV_32F,cv::Scalar(1));std::vector<double> wall,gpu;
 for(int i=0;i<23;++i){const auto t=std::chrono::steady_clock::now();auto l=Input(left,s),r=Input(right,s);
  Cuda(cudaMemcpyAsync(buffers[ln],l.data(),l.size()*sizeof(float),cudaMemcpyHostToDevice,stream));Cuda(cudaMemcpyAsync(buffers[rn],r.data(),r.size()*sizeof(float),cudaMemcpyHostToDevice,stream));
  Cuda(cudaEventRecord(begin,stream));if(!context->enqueueV3(stream))throw std::runtime_error("enqueue_failed");Cuda(cudaEventRecord(end,stream));
  Cuda(cudaMemcpyAsync(disparity.data,buffers[dn],plane*sizeof(float),cudaMemcpyDeviceToHost,stream));if(!cn.empty())Cuda(cudaMemcpyAsync(confidence.data,buffers[cn],plane*sizeof(float),cudaMemcpyDeviceToHost,stream));Cuda(cudaStreamSynchronize(stream));float ms;Cuda(cudaEventElapsedTime(&ms,begin,end));
  if(i>=3){gpu.push_back(ms);wall.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count());}
 }
 const int top=s.value("pad_top",0),cw=s.at("content_width"),ch=s.at("content_height");disparity=disparity(cv::Rect(0,top,cw,ch)).clone();confidence=confidence(cv::Rect(0,top,cw,ch)).clone();
 cv::Mat valid(disparity.size(),CV_32F);const double threshold=s.value("confidence_threshold",0.);
 for(int y=0;y<ch;++y)for(int x=0;x<cw;++x){auto& d=disparity.at<float>(y,x);const auto c=confidence.at<float>(y,x);const bool ok=std::isfinite(d)&&d>0&&std::isfinite(c)&&c>=threshold;valid.at<float>(y,x)=ok?1.F:0.F;if(!ok)d=0;}
 cv::resize(disparity,disparity,{640,360},0,0,cv::INTER_LINEAR);disparity*=640.F/cw;cv::resize(valid,valid,{640,360},0,0,cv::INTER_LINEAR);
 for(int y=0;y<360;++y)for(int x=0;x<640;++x)if(valid.at<float>(y,x)<.999F)disparity.at<float>(y,x)=0;
 std::ofstream raw(out/"disparity.bin",std::ios::binary);raw.write(reinterpret_cast<const char*>(disparity.ptr<float>()),disparity.total()*sizeof(float));
 cv::Mat depth(360,640,CV_16U,cv::Scalar(0));const double bf=s.at("bf_640");size_t count=0;
 for(int y=0;y<360;++y)for(int x=0;x<640;++x){const float d=disparity.at<float>(y,x);if(!std::isfinite(d)||d<=0)continue;const double z=bf/d;if(z>=.15&&z<=5){depth.at<uint16_t>(y,x)=uint16_t(std::nearbyint(z*1000));++count;}}
 cv::Mat gray,preview;depth.convertTo(gray,CV_8U,-255./5000,255);cv::applyColorMap(gray,preview,cv::COLORMAP_TURBO);preview.setTo(0,depth==0);cv::imwrite((out/"depth.png").string(),depth);cv::imwrite((out/"depth.jpg").string(),preview);cv::Mat compare;cv::hconcat(std::vector<cv::Mat>{left,right,preview},compare);cv::imwrite((out/"comparison.jpg").string(),compare);
 rusage use{};getrusage(RUSAGE_SELF,&use);Json j={{"model",s.at("model")},{"gpu_ms",gpu},{"wall_ms",wall},{"engine_bytes",bytes},{"execution_context_bytes",engine->getDeviceMemorySizeV2()},{"peak_rss_kib",use.ru_maxrss},{"confidence_threshold",threshold},{"valid_depth_fraction",double(count)/depth.total()},{"spec",s}};std::ofstream(out/"result.json")<<j.dump(2)<<'\n';std::cout<<j.dump()<<'\n';
 Cuda(cudaStreamSynchronize(stream));context.reset();engine.reset();runtime.reset();for(auto& [n,p]:buffers){(void)n;Cuda(cudaFree(p));}Cuda(cudaEventDestroy(begin));Cuda(cudaEventDestroy(end));Cuda(cudaStreamDestroy(stream));if(plugin)dlclose(plugin);
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;}
