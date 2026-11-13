"""Isolated domain 212: measured joint FK, IMU, floor filtering and stale rejection."""
import argparse,json,os,struct,subprocess,time
import rclpy
from sensor_msgs.msg import Imu,JointState,PointCloud2,PointField
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

def main():
    p=argparse.ArgumentParser();p.add_argument('--executable',required=True);p.add_argument('--geometry',required=True);a=p.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='212'
    model=json.load(open(a.geometry));rclpy.init();n=rclpy.create_node('test_tof_obstacles')
    ip=n.create_publisher(Imu,'/imu/data',qos_profile_sensor_data);jp=n.create_publisher(JointState,'/joint_states',10)
    cp=n.create_publisher(PointCloud2,'/r680/tof/points_raw',qos_profile_sensor_data)
    gs=[];ss=[];clouds=[]
    subs=[n.create_subscription(String,'/robot_state/geometry',lambda m:gs.append(json.loads(m.data)),qos_profile_sensor_data),
          n.create_subscription(String,'/r680/tof/ground_status',lambda m:ss.append(json.loads(m.data)),10),
          n.create_subscription(PointCloud2,'/r680/tof/points',clouds.append,qos_profile_sensor_data)]
    proc=subprocess.Popen([a.executable,'--ros-args','-p','geometry_file:='+a.geometry,'-p','support_mode:=both'],stdout=subprocess.DEVNULL)
    def feed(joints=True):
        stamp=n.get_clock().now().to_msg();imu=Imu();imu.header.stamp=stamp;imu.header.frame_id='base_Link';imu.orientation.w=1.;ip.publish(imu)
        if joints:
            j=JointState();j.header=imu.header;j.name=[x['name'] for x in model['joints']];j.position=[0.]*8;jp.publish(j)
        rclpy.spin_once(n,timeout_sec=.025)
    def wait(cond,update=True):
        end=time.monotonic()+6
        while time.monotonic()<end and not cond():
            assert proc.poll() is None
            feed(update)
        assert cond(),'timeout'
    try:
        wait(lambda:bool(gs) and not gs[-1]['available'],False)
        wait(lambda:bool(gs) and gs[-1]['available'])
        g=gs[-1];assert len(g['frames'])==18 and g['ground']['available'];h=g['ground']['offset_m']
        cloud=PointCloud2();cloud.header.frame_id='base_Link';cloud.height=1;cloud.width=3;cloud.point_step=24;cloud.row_step=72
        cloud.fields=[PointField(name=k,offset=o,datatype=d,count=1) for k,o,d in [('x',0,7),('y',4,7),('z',8,7),('distance_m',12,7),('sigma_mm',16,4),('zone',18,4),('sensor_id',20,2)]]
        cloud.data=b''.join(struct.pack('<ffffHHB3x',.5,0,z,1.,3,i,i) for i,z in enumerate([.01-h,.1-h,.6-h]))
        for _ in range(12):
            feed();cloud.header.stamp=n.get_clock().now().to_msg();cp.publish(cloud)
        wait(lambda:bool(clouds) and bool(ss) and ss[-1]['filtered_available'])
        assert clouds[-1].width==1 and clouds[-1].data[20]==1
        assert ss[-1]['ground_removed']==1 and ss[-1]['above_max_removed']==1
        old=len(clouds);end=time.monotonic()+.7
        while time.monotonic()<end:feed(False)
        wait(lambda:gs and not gs[-1]['available'],False)
        for _ in range(8):
            feed(False);cloud.header.stamp=n.get_clock().now().to_msg();cp.publish(cloud)
        end=time.monotonic()+.3
        while time.monotonic()<end:feed(False)
        # No filtered publication after joint telemetry has expired.
        assert len(clouds)==old and not ss[-1]['filtered_available']
        wait(lambda:gs[-1]['available'])
        print(json.dumps({'passed':True,'checks':['8 joint + 7 sensor geometry','joint-derived wheel floor','ground and high-point removal','stale joints with continuing IMU reject cloud','fresh geometry recovery']}))
    finally:
        proc.terminate();proc.wait(timeout=5);n.destroy_node();rclpy.shutdown();del subs
if __name__=='__main__':main()
