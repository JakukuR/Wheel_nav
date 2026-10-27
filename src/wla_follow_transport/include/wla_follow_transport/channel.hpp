#pragma once
#include <nlohmann/json.hpp>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <spawn.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
extern char **environ;

namespace wla_follow {
constexpr size_t SLOT_BYTES=16*1024*1024;
inline double monotonic() {
  timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec+t.tv_nsec*1e-9;
}
struct Packet {
  nlohmann::json metadata;
  std::vector<uint8_t> payload;
  void add(const std::string &name,const uint8_t *data,size_t size) {
    if(payload.size()+size>SLOT_BYTES-1024*1024) throw std::runtime_error("IPC payload too large");
    metadata["blobs"][name]={{"offset",payload.size()},{"size",size}};
    payload.insert(payload.end(),data,data+size);
  }
  size_t write(uint8_t *destination) const {
    const auto text=metadata.dump(); const auto length=static_cast<uint32_t>(text.size());
    const size_t total=4+text.size()+payload.size();
    if(length>1024*1024 || total>SLOT_BYTES) throw std::runtime_error("IPC packet too large");
    // Local protocol: little-endian uint32 metadata length, UTF-8 JSON, binary blobs.
    for(int i=0;i<4;++i) destination[i]=(length>>(i*8))&255;
    memcpy(destination+4,text.data(),text.size());
    memcpy(destination+4+text.size(),payload.data(),payload.size()); return total;
  }
  static Packet read(const uint8_t *source,size_t size) {
    if(size<4 || size>SLOT_BYTES) throw std::runtime_error("invalid IPC packet size");
    uint32_t len=0; for(int i=0;i<4;++i) len|=uint32_t(source[i])<<(i*8);
    if(len>1024*1024 || 4+len>size) throw std::runtime_error("invalid IPC metadata size");
    Packet p; p.metadata=nlohmann::json::parse(source+4,source+4+len);
    p.payload.assign(source+4+len,source+size); return p;
  }
  std::vector<uint8_t> blob(const std::string &name) const {
    const auto &r=metadata.at("blobs").at(name);
    const size_t offset=r.at("offset"), size=r.at("size");
    if(offset>payload.size() || size>payload.size()-offset) throw std::runtime_error("invalid IPC blob range");
    return {payload.begin()+offset,payload.begin()+offset+size};
  }
};

class Channel {
 public:
  std::string directory,socket_path,memory_path;
  int server=-1,peer=-1,file=-1; pid_t child=-1;
  uint8_t *memory=nullptr;
  bool input_busy=false; uint64_t input_sequence=0;
  Channel() {
    char path[]="/tmp/wla-follow-XXXXXX";
    if(!mkdtemp(path)) throw std::runtime_error("create IPC directory failed");
    directory=path; socket_path=directory+"/socket"; memory_path=directory+"/frames";
    file=open(memory_path.c_str(),O_RDWR|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);
    if(file<0 || ftruncate(file,SLOT_BYTES*2)) throw std::runtime_error("create shared frame file failed");
    memory=static_cast<uint8_t*>(mmap(nullptr,SLOT_BYTES*2,PROT_READ|PROT_WRITE,MAP_SHARED,file,0));
    if(memory==MAP_FAILED) { memory=nullptr; throw std::runtime_error("mmap failed"); }
    server=socket(AF_UNIX,SOCK_SEQPACKET|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    sockaddr_un addr{}; addr.sun_family=AF_UNIX; strncpy(addr.sun_path,socket_path.c_str(),sizeof(addr.sun_path)-1);
    if(server<0 || bind(server,reinterpret_cast<sockaddr*>(&addr),sizeof(addr)) || listen(server,1))
      throw std::runtime_error("bind IPC server failed");
    chmod(socket_path.c_str(),0600);
  }
  void spawn(const std::string &python) {
    std::vector<std::string> args{python,"-m","wla_person_follow.hybrid_worker","--socket",socket_path,"--memory",memory_path};
    std::vector<char*> argv; for(auto &s:args) argv.push_back(s.data()); argv.push_back(nullptr);
    const int error=posix_spawnp(&child,python.c_str(),nullptr,nullptr,argv.data(),environ);
    if(error) { child=-1; throw std::runtime_error("start Python worker: "+std::string(strerror(error))); }
  }
  bool accept_peer() {
    if(peer>=0) return false;
    int fd=accept4(server,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
    if(fd<0) { if(errno==EAGAIN || errno==EWOULDBLOCK) return false; throw std::runtime_error("IPC accept failed"); }
    ucred cred{}; socklen_t length=sizeof(cred);
    if(getsockopt(fd,SOL_SOCKET,SO_PEERCRED,&cred,&length) || cred.uid!=getuid() || cred.pid!=child) {
      close(fd); throw std::runtime_error("unexpected IPC peer");
    }
    peer=fd; return true;
  }
  bool send(const nlohmann::json &value) {
    if(peer<0) return false;
    const auto s=value.dump();
    if(s.size()>128*1024) throw std::runtime_error("oversized IPC notification");
    const ssize_t n=::send(peer,s.data(),s.size(),MSG_DONTWAIT|MSG_NOSIGNAL);
    return n==static_cast<ssize_t>(s.size());
  }
  bool receive(nlohmann::json &value) {
    char data[128*1024]; const ssize_t n=recv(peer,data,sizeof(data),MSG_DONTWAIT|MSG_TRUNC);
    if(n<0 && (errno==EAGAIN || errno==EWOULDBLOCK)) return false;
    if(n<=0 || n>static_cast<ssize_t>(sizeof(data))) throw std::runtime_error("IPC closed or oversized message");
    value=nlohmann::json::parse(data,data+n); return true;
  }
  bool alive() {
    if(child<0) return false;
    int status;
    if(waitpid(child,&status,WNOHANG)==0) return true;
    child=-1; return false;
  }
  ~Channel() {
    if(peer>=0) close(peer);
    if(child>0) {
      kill(child,SIGTERM);
      for(int i=0;i<20 && child>0;++i) {
        if(waitpid(child,nullptr,WNOHANG)==child) child=-1;
        else std::this_thread::sleep_for(std::chrono::milliseconds(25));
      }
      if(child>0) { kill(child,SIGKILL); waitpid(child,nullptr,0); }
    }
    if(server>=0) close(server);
    if(memory) munmap(memory,SLOT_BYTES*2);
    if(file>=0) close(file);
    unlink(socket_path.c_str()); unlink(memory_path.c_str()); rmdir(directory.c_str());
  }
};
}  // namespace wla_follow
