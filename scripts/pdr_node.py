#!/home/zotac/anaconda3/envs/carla/bin/python


import os, sys
print("using python interpreter ", sys.executable)
from ins_module.PDR import INS, State
import rospkg, rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf
from functools import partial
import numpy as np
from carla_msgs.msg import CarlaEgoVehicleStatus

def broadcastTF(event,state:State):
    br = tf.TransformBroadcaster()
    rotation = np.identity(4)
    rotation[0:3,0:3] = state.C_bn
    br.sendTransform(state.pos,
                    [state.Q_bn.x, state.Q_bn.y, state.Q_bn.z, state.Q_bn.w],
                    #  tf.transformations.quaternion_from_matrix(rotation),
                     rospy.Time.now(),
                     "imu_link",
                     "map")

if __name__ == "__main__":
    imu_config = os.path.join(rospkg.RosPack().get_path("wheel_ins"),"config","carla_imu.yaml")
    ins = INS(imu_config)

    rospy.init_node("ins_node")
    rospy.Subscriber("/carla/ego_vehicle/imu",Imu, ins.imuCallback, queue_size=100)
    rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, ins.setInitPose, queue_size=1)
    rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, ins.carlaSpeedUpdate, queue_size=100)
    rospy.Timer(rospy.Duration(0.1), partial(broadcastTF,state=ins.state))
    rospy.spin()