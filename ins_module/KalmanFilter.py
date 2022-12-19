from ins_module.utils import skewMatrix
import numpy as np
import rospy
import quaternion
"""
Base class for kalman filter
"""
class KalmanFilter(object):
    def __init__(self, state_size=15, inno_size=3) -> None:
        self.inno_size = inno_size
        self.state_size = state_size
        self.P = np.identity(self.state_size)
        self.Q = np.identity(self.state_size)
        self.R = np.identity(self.inno_size)

    def setP(self, P):
        self.P = P
    
    def setR(self, R):
        self.R = R
    
    def setQ(self, Q):
        self.Q = Q
    
    def transition_matrix(self, states, acc, gyro):
        raise NotImplementedError

    def predict(self, states, acc: np.ndarray, gyro: np.ndarray):
        Fx = self.transition_matrix(states, acc, gyro)
        self.P = Fx @ self.P @ Fx.T
        self.P += self.Q * states.dt

    def update(self, inno, H):
        rospy.logdebug("Innovation: {}".format(np.array2string(inno.flatten(), max_line_width=np.inf)))
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        I_KH = np.identity(self.state_size) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

        return K

    def reset(self):
        return

"""
Error state kalman filter
"""
class ESKF(KalmanFilter):
    def __init__(self, state_size=21, inno_size=3) -> None:
        super().__init__(state_size, inno_size)
        
    def transition_matrix(self, states, acc: np.ndarray, gyro: np.ndarray):
        Fx = np.identity(self.state_size)
        Fx[0:3,3:6] = np.identity(3) * states.dt
        Fx[3:6,6:9] = -states.C_bn @ skewMatrix(acc.flatten()) * states.dt
        Fx[3:6,9:12] = -states.C_bn * states.dt
        delta_gyro = gyro * states.dt
        delta_quat = quaternion.from_rotation_vector(delta_gyro)
        Fx[6:9,6:9] = quaternion.as_rotation_matrix(delta_quat).T
        Fx[6:9,12:15] = -np.identity(3) * states.dt
        return Fx

