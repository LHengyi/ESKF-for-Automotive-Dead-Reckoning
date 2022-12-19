import cv2
import rospy
import rospkg
import os
import numpy as np
from scipy.spatial.transform import Rotation
import quaternion
import ins_module.utils as utils
from sensor_msgs.msg import Imu
from carla_msgs.msg import CarlaEgoVehicleStatus
from ins_module.KalmanFilter import ESKF

class IMUParam(object):
    def __init__(self, config) -> None:
        super().__init__()
        settings = cv2.FileStorage(config, cv2.FileStorage_READ)
        self.frequency = settings.getNode("imu").getNode("frequency").real()
        self.misalignment = settings.getNode("imu").getNode("misalignment").mat()
        self.correlation_time = settings.getNode("imu").getNode("correlation_time").real()
        self.std_bg = settings.getNode("imu").getNode("std_bg").real()
        self.std_ba = settings.getNode("imu").getNode("std_ba").real()
        self.std_sg = settings.getNode("imu").getNode("std_sg").real()
        self.std_sa = settings.getNode("imu").getNode("std_sa").real()
        self.VRW = settings.getNode("imu").getNode("VRW").real()
        self.ARW = settings.getNode("imu").getNode("ARW").real()
        self.init_yaw = settings.getNode("imu").getNode("init_yaw").real()
        self.gravity = settings.getNode("gravity").mat()
        self.wheel_radius = settings.getNode("wheel").getNode("radius").real()
        self.use_wheel_odom = settings.getNode("use_wheel_odom").string() == 'true'
        

class State(object):
    def __init__(self, init_pose = np.zeros((3,1)),state_size=21) -> None:
        super().__init__()
        self.state_size = state_size
        self.init_orientation = False
        self.init_position = False
        self.MemoInitData = np.zeros((1,7))
        # error state model, error of position, velocity, attitude, gyro bias, acc bias, gyro scale factor, acc scale factor
        self.states = np.zeros((self.state_size,1))
        self.pos = init_pose
        self.vel = np.zeros((3,1))
        self.gravity = np.array([0,0,9.8]).reshape((3,1))
        self.attitude = np.zeros((3,1))
        self.C_bn = np.identity(3) # body frame to navigation frame
        self.Q_bn = quaternion.from_rotation_matrix(self.C_bn)
        self.C_vn = np.identity(3) # vehicle frame to navigation frame
        self.C_bv = np.identity(3)
        self.Bg = np.zeros((3,1))
        self.Ba = np.zeros((3,1))
        self.Sg = np.zeros((3,1))
        self.Sa = np.zeros((3,1))
        self.current_time = -1
        self.dt = 0
        self.filter_update = 0
        self.pre_measure_dt = np.zeros((6,1))
        self.sys_wheel_odom = 0.0
        self.Gx_comped = 0
        self.vehicle_pitch = 0
        self.T_Odom = -1

    def initAttitude(self, imu_buffer, imu_param:IMUParam):
        self.MemoInitData = np.average(imu_buffer,axis=0)
        # caculate initial attitude, assume IMU is binded to the wheel, x axis is the rotating axis
        self.attitude[0] = np.arctan2(-self.MemoInitData[2],-self.MemoInitData[3])
        self.attitude[1] = np.arctan2(self.MemoInitData[1], np.sqrt(self.MemoInitData[2]**2 + self.MemoInitData[3]**2))
        self.attitude[2] = imu_param.init_yaw

        # rotation matrix and quaternion
        self.C_bn = Rotation.from_euler('ZYX',[self.attitude[2],self.attitude[1],self.attitude[0]]).as_matrix() # intrinsic rotation
        self.Q_bn = quaternion.from_rotation_matrix(self.C_bn)

        # set initial gyro bias
        self.Bg[0] = self.MemoInitData[4]
        self.Bg[1] = self.MemoInitData[5]
        self.Bg[2] = self.MemoInitData[6]
        self.gravity = imu_param.gravity
    
    def VINSImuInit(self, imu_buffer, imu_param:IMUParam):
        self.MemoInitData = np.mean(imu_buffer,axis=0)
        mean = np.mean(imu_buffer[:,1:4], axis=0)
        var = np.var(imu_buffer[:,1:4], axis=0)
        if np.any(var[1:] >= 3):
            return False
        z_axis = mean.reshape((3,1))/np.linalg.norm(mean)
        unit_X = np.array([1,0,0]).reshape((3,1))
        x_axis = unit_X - z_axis @ z_axis.T @ unit_X
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis.T,x_axis.T).T
        y_axis /= np.linalg.norm(y_axis)
        R_ig = np.block([x_axis, y_axis, z_axis]) 
        rotation = Rotation.from_matrix(R_ig.T)
        self.C_bn = rotation.as_matrix()
        self.Q_bn = quaternion.from_rotation_matrix(self.C_bn)
        self.attitude = rotation.as_euler("ZYX")

        # set initial gyro bias
        self.Bg[0] = self.MemoInitData[4]
        self.Bg[1] = self.MemoInitData[5]
        self.Bg[2] = self.MemoInitData[6]
        self.gravity = imu_param.gravity
        self.Ba = -self.C_bn.T @ self.gravity.flatten() + self.MemoInitData[1:4]
        self.Ba = self.Ba.reshape((3,1))
    
    def carlaImuInit(self, imu_buffer, imu_msg, imu_param):
        self.MemoInitData = np.mean(imu_buffer,axis=0)
        self.Q_bn = np.quaternion(imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z)
        self.C_bn = quaternion.as_rotation_matrix(self.Q_bn)
        self.attitude = Rotation.from_matrix(self.C_bn).as_euler("ZYX")
        # set initial gyro bias
        self.Bg[0] = self.MemoInitData[4]
        self.Bg[1] = self.MemoInitData[5]
        self.Bg[2] = self.MemoInitData[6]
        self.gravity = imu_param.gravity
        self.Ba = -self.C_bn.T @ self.gravity.flatten() + self.MemoInitData[1:4]
        self.Ba = self.Ba.reshape((3,1))

        

    def initOrientation(self, imu_buffer, msg, imu_param:IMUParam):
        if imu_param.use_wheel_odom:
            self.initAttitude(imu_buffer, imu_param)
        else:
            self.carlaImuInit(imu_buffer, msg, imu_param)

        self.current_time = imu_buffer[-1,0]
        self.T_Odom = imu_buffer[-1,0]
        self.init_orientation = True
        # self.gravity = imu_param.gravity
        # self.gravity = -self.MemoInitData[1:4].reshape((3,1))


class INS(object):
    def __init__(self, config) -> None:
        super().__init__()
        self.imu_param = IMUParam(config)
        settings = cv2.FileStorage(config, cv2.FileStorage_READ)
        init_pose = settings.getNode("init_pos").mat()
        self.state = State(init_pose)
        self.use_wheel_odom = settings.getNode("use_wheel_odom").string() == 'true'
        self.odom_dt = settings.getNode("odom_dt").real()
        self.odom_std = settings.getNode("odom_std").mat()
        self.wheel_lever_arm = settings.getNode("wheel_lever_arm").mat()
        self.yaw_bv = settings.getNode("yaw_bv").real()
        self.bufferSize = int(self.imu_param.frequency)
        # buffer: size * [timestamp, acc, gyro]
        self.buffer = np.zeros((self.bufferSize, 7))
        self.imu_count = 0
        self.current_measure = np.zeros((7,)) # time, acc, gyro
        
        # set params for kalman filter
        self.kalman_filter = ESKF()
        Rv = np.zeros((4,4))
        Rv[1:,1:] = np.diag(self.odom_std.flatten())
        self.kalman_filter.setR(Rv)
        Q = np.zeros((self.state.state_size,self.state.state_size))
        Q[3:6,3:6] = np.identity(3) * (self.imu_param.VRW ** 2)
        Q[6:9,6:9] = np.identity(3) * (self.imu_param.ARW ** 2)
        Q[9:12,9:12] = np.identity(3) * (2.0 * self.imu_param.std_bg / self.imu_param.correlation_time)
        Q[12:15,12:15] = np.identity(3) * (2.0 * self.imu_param.std_ba / self.imu_param.correlation_time)
        Q[15:18,15:18] = np.identity(3) * (2.0 * self.imu_param.std_sg / self.imu_param.correlation_time)
        Q[18:21,18:21] = np.identity(3) * (2.0 * self.imu_param.std_sa / self.imu_param.correlation_time)
        P = np.zeros((self.state.state_size,self.state.state_size))
        P[0:3,0:3] = np.identity(3) * (0.05 ** 2)
        P[3:6,3:6] = np.identity(3) * (0.05 ** 2)
        P[6:9,6:9] = np.identity(3) * (0.02 ** 2)
        P[9:12,9:12] = np.identity(3) * (self.imu_param.std_bg)
        P[12:15,12:15] = np.identity(3) * (self.imu_param.std_ba)
        P[15:18,15:18] = np.identity(3) * (self.imu_param.std_sg)
        P[18:21,18:21] = np.identity(3) * (self.imu_param.std_sa)
        self.kalman_filter.setQ(Q[0:self.state.state_size,0:self.state.state_size])
        self.kalman_filter.setP(P[0:self.state.state_size,0:self.state.state_size])


    def biasCompenstate(self):
        self.current_measure[1] = (1-self.state.Sa[0]) * (self.current_measure[1] - self.state.Ba[0])
        self.current_measure[2] = (1-self.state.Sa[1]) * (self.current_measure[2] - self.state.Ba[1])
        self.current_measure[3] = (1-self.state.Sa[2]) * (self.current_measure[3] - self.state.Ba[2])
        self.current_measure[4] = (1-self.state.Sg[0]) * (self.current_measure[4] - self.state.Bg[0])
        self.current_measure[5] = (1-self.state.Sg[1]) * (self.current_measure[5] - self.state.Bg[1])
        self.current_measure[6] = (1-self.state.Sg[2]) * (self.current_measure[6] - self.state.Bg[2])
    
    """
    Implementation of Inertial Navigation System(INS), reference Shin, E.-H. "Estimation techniques for low-cost inertial navigation"
    states expressed in navigation frame
    """
    def InertialImplNavFrame(self):
        gyro_dt = 0.5*(self.buffer[-1,-3:]+self.buffer[-2,-3:]) * self.state.dt
        # equation 2.60, estimation body frame rotation vector
        Gzeta = np.cross(self.state.pre_measure_dt[-3:].flatten(), gyro_dt)
        Gzeta = gyro_dt + Gzeta/12.0
        v = np.linalg.norm(Gzeta,ord=2) # check whether matrix norm or vector norm
        if v > 1e-12:
            quat_dt = quaternion.from_rotation_vector(Gzeta.flatten())
            quat_temp = quat_dt * self.state.Q_bn # check quaternion multiplication
            self.state.Q_bn = quat_temp
            self.state.C_bn = quaternion.as_rotation_matrix(self.state.Q_bn)
            self.state.attitude = Rotation.from_matrix(self.state.C_bn).as_euler(seq='ZYX')

        # velocity update, equation 2.48
        acc_dt = 0.5*(self.buffer[-1,1:4]+self.buffer[-2,1:4]) * self.state.dt # Delta v{f,k}^{b}
        zeta = acc_dt + 0.5 * np.cross(gyro_dt.flatten(), acc_dt.flatten()) + \
               (np.cross(self.state.pre_measure_dt[-3:].flatten(),acc_dt.flatten()) + \
               np.cross(self.state.pre_measure_dt[0:3].flatten(),gyro_dt.flatten()))/12.0

        # equation 2.49
        I_skew = np.identity(3) - 0.5*utils.skewMatrix(gyro_dt)
        half_Cbn = self.state.C_bn @ I_skew
        an_hat = half_Cbn @ zeta
        an_hat = an_hat.reshape((3,1))
        an_hat -= self.state.gravity * self.state.dt

        # update position
        self.state.pos += self.state.vel * self.state.dt + 0.5 * an_hat * self.state.dt
        self.state.vel += an_hat

        self.state.pre_measure_dt = np.block([acc_dt,gyro_dt])
        rospy.loginfo_throttle(1,"INS estimation: speed {}, attitude {}\n".format(self.state.vel.T,  self.state.attitude.T))


    def EKF_Predict(self):
        gyro_dt = 0.5*(self.buffer[-1,-3:]+self.buffer[-2,-3:]) * self.state.dt
        acc_dt = 0.5*(self.buffer[-1,1:4]+self.buffer[-2,1:4]) * self.state.dt # Delta v{f,k}^{b}
        acc_global = self.state.C_bn @ self.buffer[-1,1:4]

        self.kalman_filter.predict(self.state,self.buffer[-1,1:4], self.buffer[-1,-3:])

    def getVelocity(self):
        average_Gx = np.average(self.buffer[-int(self.imu_param.frequency)//4:,4])
        self.state.Gx_comped = (1 - self.state.Sg[0]) * (average_Gx - self.state.Bg[0])
        self.state.sys_wheel_odom = -self.imu_param.wheel_radius * self.state.Gx_comped
        C_vn_euler = np.array([0,0,self.state.attitude[2] - self.yaw_bv])

        self.state.C_vn = Rotation.from_euler("ZYX",C_vn_euler).as_matrix()
        self.state.C_bv = self.state.C_vn.T @ self.state.C_bn

    def EKF_Update(self, inno, H):
        K = self.kalman_filter.update(inno, H)
        dx = K @ inno
        self.state.states += dx
        rospy.loginfo("delta x: {}\n".format(np.array2string(dx.flatten(),max_line_width=np.inf, threshold=np.inf)))

        self.state.filter_update += 3

    def NHCUpdate(self):
        if self.use_wheel_odom:
            if self.state.current_time - self.state.T_Odom > self.odom_dt - 0.5/self.imu_param.frequency:
                V_vel = self.state.C_vn.T @ self.state.vel
                V_velskew = self.state.C_vn.T @ utils.skewMatrix(self.state.vel)

                gyro = self.state.pre_measure_dt[3:]/self.state.dt
                wib_skew = utils.skewMatrix(gyro)
                Cbvwskew = self.state.C_bv @ wib_skew
                vLeverArm = Cbvwskew @ self.wheel_lever_arm

                Cbnwskew = self.state.C_bn @ wib_skew
                vnLeverArm = Cbnwskew @ self.wheel_lever_arm

                vnLeverArm_skew = utils.skewMatrix(vnLeverArm)
                vVLeverArm_skew = self.state.C_vn.T @ vnLeverArm_skew

                leverArm_skew = utils.skewMatrix(self.wheel_lever_arm)
                cLeverArm_skew = self.state.C_bv @ leverArm_skew

                Cbv_LAskew_wibb = cLeverArm_skew @ np.diag(gyro)

                Hv = np.zeros((3,self.state.state_size))
                Hv[0:3,3:6] = self.state.C_vn.T
                Hv[0:3,6:9] = vVLeverArm_skew
                Hv[0:3,9:12] = -cLeverArm_skew
                Hv[0:3,15:18] = -Cbv_LAskew_wibb
                Hv[0:3,8] = -V_velskew[:,-1]

                Hv[0:3,9:12] += np.identity(3) * (-self.imu_param.wheel_radius)
                Hv[0:3,15:18] += np.diag(-self.imu_param.wheel_radius * gyro)

                Zv = V_vel + vLeverArm
                Zv[0] -= self.state.sys_wheel_odom

                Rv = np.diag(self.odom_std)
                inno = Zv - Hv @ self.state.states
                self.EKF_Update(inno, Hv, Rv)
                self.state.T_Odom = self.state.current_time

    def carlaSpeedUpdate(self, vehicle_status: CarlaEgoVehicleStatus):
        if self.state.current_time - self.state.T_Odom > self.odom_dt - 0.5/self.imu_param.frequency:
            # Velocity in body frame, assuming no slipping, jumping.
            # Zv: Z position + speed in vehicle/imu frme
            Zv = np.zeros((4,1))
            # Zv = np.array([0 - self.state.pos.flatten()[2], vehicle_status.velocity - np.linalg.norm(self.state.vel.flatten()), 0, 0])
            v_body = self.state.C_bn.T @ self.state.vel
            Zv[0] = 0 - self.state.pos.flatten()[2]
            Zv[1] = vehicle_status.velocity - v_body.flatten()[0]
            Zv[2] = 0 - v_body.flatten()[1]
            Zv[3] = 0 - v_body.flatten()[2]
            Hv = np.zeros((4,self.state.state_size))
            Hv[0,2] = 1
            Hv[1:4,3:6] = self.state.C_bn.T
            inno = Zv.reshape((-1,1)) - Hv @ self.state.states # states should be zero after reset
            self.EKF_Update(inno, Hv)
            self.state.T_Odom = self.state.current_time


    def stateFeedback(self):
        self.state.pos += self.state.states[0:3]
        self.state.vel += self.state.states[3:6]

        # Delta_rotation = Rotation.from_rotvec(self.state.states[6:9].flatten())
        # Delta_quat = quaternion.from_rotation_matrix(Delta_rotation.as_matrix())
        # self.state.C_bn = Rotation.from_quat(self.state.Q_bn).as_matrix()
        delta_quat = quaternion.from_rotation_vector(self.state.states[6:9].flatten())
        # self.state.Q_bn = Delta_quat * self.state.Q_bn
        # self.state.Q_bn = self.state.Q_bn * delta_quat
        # self.state.attitude = Rotation.from_matrix(quaternion.as_rotation_matrix(self.state.Q_bn)).as_euler("ZYX")
        # self.state.C_bn = quaternion.as_rotation_matrix(self.state.Q_bn)
        self.state.Bg += self.state.states[9:12]
        self.state.Ba += self.state.states[12:15]
        self.state.Sg += self.state.states[15:18]
        self.state.Sa += self.state.states[18:21]
        self.state.states = np.zeros((self.state.state_size,1))
        self.state.filter_update = 0

    def imuCallback(self, msg):
        timestamp = msg.header.stamp.to_sec()
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z
        self.current_measure = np.array([timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z])
        if self.imu_count < self.imu_param.frequency:
            self.buffer[self.imu_count,:] = self.current_measure # shallow copy
        else:
            self.buffer[0:-1,:] = self.buffer[1:, :]
            self.buffer[-1,:] = self.current_measure

            if not self.state.init_orientation:
                self.state.initOrientation(self.buffer, msg, self.imu_param)
            elif not self.state.init_position:
                rospy.loginfo("Wait for initial position.\n")
            else:
                self.state.current_time = timestamp
                self.state.dt = self.buffer[-1,0] - self.buffer[-2,0]
                # compenstate bias
                self.biasCompenstate()
                # INS implementation
                self.InertialImplNavFrame()
                # EKF predict
                self.EKF_Predict()
                # self.getVelocity()
                # # odom update
                # self.NHCUpdate()
                if self.state.filter_update >= 1:
                    self.stateFeedback()
        self.imu_count += 1
    
    def setInitPose(self, msg):
        if not self.state.init_position:
            self.state.pos[0] = msg.pose.pose.position.x
            self.state.pos[1] = msg.pose.pose.position.y
            self.state.pos[2] = msg.pose.pose.position.z
            self.state.init_position = True

if __name__ == "__main__":
    imu_config = os.path.join(rospkg.RosPack().get_path("wheel_ins"),"config","gazebo_imu.yaml")
    ins = INS(imu_config)

    rospy.init_node("ins_node", log_level=rospy.DEBUG)
    rospy.Subscriber("/left_rear_imu",Imu, ins.imuCallback, queue_size=100)
    
    rospy.spin()