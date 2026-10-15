#!/usr/bin/env python3
"""Compile upstream callback/algorithms in a hardware-free harness.
The regression invokes the actual motion callback extracted from the pinned
driver. A negative control restores the old call and MUST fail for mode 2.
"""
from pathlib import Path
import subprocess
import sys
import tempfile


def function(text, marker):
    start = text.index(marker)
    brace = text.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


def main(root):
    node = (root / 'realsense2_camera/src/rs_node_setup.cpp').read_text()
    base = (root / 'realsense2_camera/src/base_realsense_node.cpp').read_text()
    header = (root / 'realsense2_camera/include/base_realsense_node.h').read_text()
    callback = function(node, 'std::function<void(rs2::frame)> imu_callback_function')
    declaration = next(line.strip() for line in header.splitlines()
                       if 'void imu_callback_sync(' in line)
    algorithms = '\n'.join(function(base, marker) for marker in (
        'template <typename T> T lerp',
        'void BaseRealSenseNode::FillImuData_LinearInterpolation',
        'void BaseRealSenseNode::FillImuData_Copy'))
    prefix = r"""
#include <Eigen/Core>
#include <cmath>
#include <deque>
#include <functional>
#include <iostream>
#include <utility>
using stream_index_pair = std::pair<int,int>;
const stream_index_pair ACCEL{1,0}, GYRO{2,0};
enum class imu_sync_method {NONE=0, COPY=1, LINEAR_INTERPOLATION=2};
struct CimuData {
    stream_index_pair m_type;
    Eigen::Vector3d m_data{0,0,0};
    double m_time_ns=-1;
    CimuData() = default;
    CimuData(stream_index_pair t, Eigen::Vector3d d, double n)
      :m_type(t),m_data(d),m_time_ns(n){}
    bool is_set() const {return m_time_ns>0;}
};
namespace rs2 {using frame=CimuData;}
namespace sensor_msgs {namespace msg {
struct Imu {double stamp; Eigen::Vector3d accel,gyro;};
}}
class BaseRealSenseNode {
public:
    imu_sync_method _imu_sync_method;
    std::deque<sensor_msgs::msg::Imu> published;
    int raw_count=0;
    void imu_callback(rs2::frame) {++raw_count;}
    sensor_msgs::msg::Imu CreateUnitedMessage(CimuData a,CimuData g) {
        return {g.m_time_ns,a.m_data,g.m_data};
    }
    void FillImuData_Copy(CimuData,std::deque<sensor_msgs::msg::Imu>&);
    void FillImuData_LinearInterpolation(CimuData,std::deque<sensor_msgs::msg::Imu>&);
"""
    dispatch = r"""
void BaseRealSenseNode::imu_callback_sync(rs2::frame frame, imu_sync_method mode) {
    std::deque<sensor_msgs::msg::Imu> messages;
    if(mode==imu_sync_method::COPY) FillImuData_Copy(frame,messages);
    else if(mode==imu_sync_method::LINEAR_INTERPOLATION)
        FillImuData_LinearInterpolation(frame,messages);
    else throw 1;
    for(auto m:messages) published.push_back(m);
}
int main(int argc,char**argv) {
    int mode=std::stoi(argv[1]);
    BaseRealSenseNode node;
    node._imu_sync_method=static_cast<imu_sync_method>(mode);
    auto callback=node.callback();
    callback(CimuData(ACCEL,{0,0,0},100000000));
    callback(CimuData(GYRO,{1,2,3},105000000));
    callback(CimuData(ACCEL,{10,20,30},110000000));
    if(node.raw_count!=3) return 2;
    if(mode==0) return node.published.empty()?0:3;
    if(node.published.size()!=1) return 4;
    const auto m=node.published.front();
    if(m.stamp!=105000000 || (m.gyro-Eigen::Vector3d(1,2,3)).norm()>1e-12) return 5;
    Eigen::Vector3d expected=mode==1?Eigen::Vector3d(0,0,0):Eigen::Vector3d(5,10,15);
    if((m.accel-expected).norm()>1e-12) {
        std::cerr<<"mode "<<mode<<" accel="<<m.accel.transpose()
                 <<" expected="<<expected.transpose()<<"\n";
        return 6;
    }
    return 0;
}
"""
    def code(cb):
        return (prefix + declaration + '\n std::function<void(rs2::frame)> callback(){'
                + cb + ';return imu_callback_function;}\n};\n' + algorithms + dispatch)
    with tempfile.TemporaryDirectory(prefix='realsense-imu-regression-') as tmp:
        tmp=Path(tmp)
        for variant, cb in (
            ('patched',callback),
            ('negative_control',callback.replace(
                'imu_callback_sync(frame, _imu_sync_method)', 'imu_callback_sync(frame)'))):
            src=tmp/(variant+'.cpp'); binary=tmp/variant
            src.write_text(code(cb))
            subprocess.run(['c++','-std=c++14','-I/usr/include/eigen3',
                            str(src),'-o',str(binary)],check=True)
            for mode in (0,1,2):
                result=subprocess.run([str(binary),str(mode)],capture_output=True,text=True)
                expected=6 if variant=='negative_control' and mode==2 else 0
                if result.returncode!=expected:
                    raise AssertionError((variant,mode,result.returncode,result.stderr))
                print(f'{variant} mode={mode}: expected exit={expected}, got={result.returncode}')
    print('PASS: NONE stays disabled; COPY holds accel; LINEAR interpolates at gyro stamp; old call fails.')


if __name__=='__main__':
    main(Path(sys.argv[1]))
