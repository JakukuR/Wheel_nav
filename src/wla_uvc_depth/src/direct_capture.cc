#include "wla_uvc_depth/direct_capture.h"
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <chrono>
#include <stdexcept>
#include <cerrno>
#include <cstring>
namespace wla::uvc {
namespace {
int Ioctl(int fd,unsigned long op,void* p){int r;do{r=ioctl(fd,op,p);}while(r<0&&errno==EINTR);return r;}
void Check(bool ok,const char* what){if(!ok)throw std::runtime_error(std::string(what)+": "+std::strerror(errno));}
int64_t Now(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
}
struct DirectCapture::Impl {
 int fd=-1;bool streaming=false;std::vector<std::pair<void*,size_t>> buffers;
 ~Impl(){if(fd>=0){if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;Ioctl(fd,VIDIOC_STREAMOFF,&t);}for(auto [p,n]:buffers)munmap(p,n);close(fd);}}
};
DirectCapture::DirectCapture(const std::string& device):m_impl(std::make_unique<Impl>()){
 auto& p=*m_impl;p.fd=open(device.c_str(),O_RDWR|O_NONBLOCK|O_CLOEXEC);Check(p.fd>=0,"open_camera");
 v4l2_capability cap{};Check(Ioctl(p.fd,VIDIOC_QUERYCAP,&cap)==0,"query_camera");
 const auto caps=(cap.capabilities&V4L2_CAP_DEVICE_CAPS)?cap.device_caps:cap.capabilities;
 if(!(caps&V4L2_CAP_VIDEO_CAPTURE)||!(caps&V4L2_CAP_STREAMING))throw std::runtime_error("camera_requires_v4l2_capture_streaming");
 v4l2_format f{};f.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;f.fmt.pix.width=2560;f.fmt.pix.height=720;f.fmt.pix.pixelformat=V4L2_PIX_FMT_MJPEG;f.fmt.pix.field=V4L2_FIELD_ANY;
 Check(Ioctl(p.fd,VIDIOC_S_FMT,&f)==0,"set_mjpeg");
 if(f.fmt.pix.width!=2560||f.fmt.pix.height!=720||f.fmt.pix.pixelformat!=V4L2_PIX_FMT_MJPEG)throw std::runtime_error("direct_capture_requires_2560x720_mjpeg");
 v4l2_streamparm rate{};rate.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;rate.parm.capture.timeperframe.numerator=1;rate.parm.capture.timeperframe.denominator=60;Check(Ioctl(p.fd,VIDIOC_S_PARM,&rate)==0,"set_capture_rate");
 v4l2_requestbuffers req{};req.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;req.memory=V4L2_MEMORY_MMAP;req.count=4;Check(Ioctl(p.fd,VIDIOC_REQBUFS,&req)==0&&req.count>=2,"allocate_capture_buffers");
 for(unsigned i=0;i<req.count;++i){v4l2_buffer b{};b.type=req.type;b.memory=req.memory;b.index=i;Check(Ioctl(p.fd,VIDIOC_QUERYBUF,&b)==0,"query_buffer");void* address=mmap(nullptr,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,p.fd,b.m.offset);Check(address!=MAP_FAILED,"map_capture_buffer");p.buffers.emplace_back(address,b.length);Check(Ioctl(p.fd,VIDIOC_QBUF,&b)==0,"queue_buffer");}
 v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;Check(Ioctl(p.fd,VIDIOC_STREAMON,&type)==0,"start_capture");p.streaming=true;
}
DirectCapture::~DirectCapture()=default;
std::shared_ptr<JpegFrame> DirectCapture::latest(int timeout){
 auto& p=*m_impl;pollfd poller{p.fd,POLLIN,0};const int ready=poll(&poller,1,timeout);if(ready<0&&errno==EINTR)return {};Check(ready>=0,"poll_camera");if(!ready)return {};
 if(poller.revents&(POLLERR|POLLHUP|POLLNVAL))throw std::runtime_error("camera_poll_failure");
 std::shared_ptr<JpegFrame> result;
 for(unsigned drained=0;drained<p.buffers.size();++drained){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;
  if(Ioctl(p.fd,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN)break;Check(false,"dequeue_camera");}
  if(b.index>=p.buffers.size()||b.bytesused>p.buffers[b.index].second)throw std::runtime_error("invalid_capture_buffer");
  if(!(b.flags&V4L2_BUF_FLAG_ERROR)&&b.bytesused){auto frame=std::make_shared<JpegFrame>();auto* start=static_cast<uint8_t*>(p.buffers[b.index].first);frame->bytes.assign(start,start+b.bytesused);
   const int64_t now=Now(),stamp=int64_t(b.timestamp.tv_sec)*1000000000LL+int64_t(b.timestamp.tv_usec)*1000;
   frame->kernelTimestamp=(b.flags&V4L2_BUF_FLAG_TIMESTAMP_MASK)==V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC&&stamp<=now&&now-stamp<1000000000LL;
   frame->monotonicNs=frame->kernelTimestamp?stamp:now;result=std::move(frame);}
  Check(Ioctl(p.fd,VIDIOC_QBUF,&b)==0,"requeue_camera");
 }
 return result;
}
}
