#!/usr/bin/env python
import rospy
import serial
from time import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import numpy as np
from math import cos, sin, pi
from numpy import transpose, matmul, array, eye
from numpy.linalg import inv
# global angle

class EkfNode(object):

    def __init__(self):
        # global angle
        self.camera_flag = False
        self.axy_sel = array([[1,0,0],[0,1,0],[0,0,0]])
        self.wz_sel = array([[0,0,0],[0,0,0],[0,0,1]])
        self.tprev = time()

        self.covpos_sel = array([[1,0,0],[0,1,0],[0,0,0],[0,0,0],[0,0,0],[0,0,1]])
        self.covtwist_sel = np.array([[1,0],[0,1],[0,0],[0,0],[0,0],[0,0]])

        self.mu = array([0,0,0,0,0]).reshape(5,1)
        self.Sigma = eye(5)

        self.Q = eye(3)
        self.z = array([0,0,0]).reshape(3,1)

        self.rate = rospy.Rate(200)

        self.pub = rospy.Publisher('satellite_odom', Odometry, queue_size=10)
        self.posepub = rospy.Publisher('satellite_pose', PoseWithCovarianceStamped, queue_size=10)
        self.imu_sub = rospy.Subscriber('razor_imu', Imu, callback=self.imu_callback)
        self.aruco_sub = rospy.Subscriber('aruco_stamped', PoseWithCovarianceStamped, callback=self.aruco_callback)

        # angle = 0

    def start(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            rospy.spin()

    def imu_callback(self,imu_msg):
        # global angle
        # print(imu_msg.linear_acceleration)
        lin_acc = array([[imu_msg.linear_acceleration.x],[imu_msg.linear_acceleration.y],[imu_msg.linear_acceleration.z]])
        ang_vel = array([[imu_msg.angular_velocity.x],[imu_msg.angular_velocity.y],[imu_msg.angular_velocity.z]])

        u = matmul(self.axy_sel,lin_acc) + matmul(self.wz_sel,ang_vel)

        # u[2] = -u[2]
        offset_angle = pi/2
        ax = u[0]*cos(offset_angle)+u[1]*sin(offset_angle)
        ay =-u[0]*sin(offset_angle)+u[1]*cos(offset_angle)
        u[0] = ax
        u[1] = ay
        

        cov_linear = array(imu_msg.linear_acceleration_covariance).reshape((3,3))
        cov_angular = array(imu_msg.angular_velocity_covariance).reshape((3,3))

        M = btab(cov_linear,self.axy_sel) + btab(cov_angular,self.wz_sel)

        # u = array([ax,ay,wz])
        # M = cov_linear
        # M[2,0:2] = 0
        # M[0:2,2] = 0
        # M[2,2] = cov_angular[2,2]

        #print('u = ')
        #print(u)
        #print()
        #print('M = ')
        #print(M)
        #print()

        #print("mu = ")
        #print(self.mu)
        #print()


        dt = time() - self.tprev
        self.tprev = time()

        mubar = process_update_mu(self.mu,self.Sigma,u,M,dt)
        # angle = angle + dt*u[2]

        Sigmabar = process_update_Sigma(self.mu, self.Sigma, u, M, dt)


        if self.camera_flag:
            K = measurement_kalman_gain_K(Sigmabar,self.Q)
            self.mu = measurement_update_mu(mubar, K, self.z)
            self.Sigma = measurement_update_Sigma(Sigmabar, K)
            self.camera_flag = False
        else:
            self.mu = mubar
            self.Sigma = Sigmabar

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'map'

        odom.pose.pose.position.x = self.mu[0]
        odom.pose.pose.position.y = self.mu[1]
        # print(self.mu[2])
        # phi = self.mu[2]
        quat = quaternion_from_euler(0,0,float(self.mu[2]))
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        odom.pose.covariance = babt(self.Sigma[0:3,0:3],self.covpos_sel).reshape(36,1)

        odom.twist.twist.linear.x = self.mu[3]
        odom.twist.twist.linear.y = self.mu[4]
        odom.twist.twist.angular.z = u[2]

        odom.twist.covariance = babt(self.Sigma[3:6,3:6],self.covtwist_sel).reshape(36,1)
        odom.twist.covariance[35] = M[2,2]

        #print("Estimated eulers: " + str(euler_from_quaternion(quat)))

        self.pub.publish(odom)

        satpose = PoseWithCovarianceStamped()
        satpose.pose = odom.pose
        satpose.header.frame_id = 'map'
        satpose.header.stamp = rospy.Time.now()
        self.posepub.publish(satpose)



    def aruco_callback(self,aruco_pose):
        # global camera_flag, axy_sel, wz_sel, covpos_sel, z, Q
        self.camera_flag = True
        xyzposition = array([aruco_pose.pose.pose.position.x,aruco_pose.pose.pose.position.y,aruco_pose.pose.pose.position.z]).reshape(3,1)
        quat = array([aruco_pose.pose.pose.orientation.x,aruco_pose.pose.pose.orientation.y,aruco_pose.pose.pose.orientation.z,aruco_pose.pose.pose.orientation.w])#.reshape(4,1)
        xyzangles = array(euler_from_quaternion(quat)).reshape(3,1)
        aruco_cov = array(aruco_pose.pose.covariance).reshape(6,6)

        self.z = matmul(self.axy_sel,xyzposition) + matmul(self.wz_sel,xyzangles)
        # Janky transform
        self.z[0] = self.z[0]#*.30/.16
        self.z[1] = -self.z[1]#*.30/.16
        self.z[2] = -self.z[2]
        # self.z[2] = xyzangles[2]
        # print(float(self.z[2]))
        self.Q = btab(aruco_cov,self.covpos_sel)
        #print("measured eulers: " + str(xyzangles.reshape(1,3)))
        #print('z = ' + str(self.z.reshape(1,3)))
        #print(self.z)
        #print()
        #print('Q = ')
        #print(self.Q)
        #print()

def process_update_mu(mu,Sigma,u,M,dt):
    #print(mu)
    x = mu[0]
    y = mu[1]
    phi = mu[2]
    vx = mu[3]
    vy = mu[4]

    ax = u[0]
    ay = u[1]
    wz = u[2]
    # print(wz)
    # print(dt)

    xbar = x + vx*dt + 0.5*dt**2*(ax*cos(phi)-ay*sin(phi))
    ybar = y + vy*dt + 0.5*dt**2*(ax*sin(phi)+ay*cos(phi))
    phibar = phi + dt*wz
    # print(dt*wz)
    phibar = ((phibar + pi) % (2*pi)) - pi
    # print(phibar)

    vxbar = vx + dt*(ax*cos(phi)-ay*sin(phi))
    vybar = vy + dt*(ax*sin(phi)+ay*cos(phi))

    mubar = array([xbar, ybar, phibar, vxbar, vybar]).reshape(5,1)

    return mubar

def process_update_Sigma(mu,Sigma,u,M,dt):
#     x = mu[0]
#     y = mu[1]
    phi = mu[2]
    # vx = mu[3]
    # vy = mu[4]

    ax = u[0]
    ay = u[1]
    # wz = u[2]

    # Jacobian of mubar wrt mu
    G = array([ [1, 0, -0.5*dt**2*(ax*sin(phi) + ay*cos(phi)), dt,  0],
                [0, 1,  0.5*dt**2*(ax*cos(phi) - ay*sin(phi)),  0, dt],
                [0, 0,                                      1,  0,  0],
                [0, 0,        -dt*(ax*sin(phi) + ay*cos(phi)),  1,  0],
                [0, 0,         dt*(ax*cos(phi) - ay*sin(phi)),  0,  1]])

    # Jacobian of mubar wrt u
    V = array([ [0.5*dt**2*cos(phi), -0.5*dt**2*sin(phi),  0],
                [0.5*dt**2*sin(phi),  0.5*dt**2*cos(phi),  0],
                [                 0,                   0, dt],
                [       dt*cos(phi),        -dt*sin(phi),  0],
                [       dt*sin(phi),         dt*cos(phi),  0]])

    Sigmabar = babt(Sigma,G) + babt(M,V)

    return Sigmabar


def measurement_kalman_gain_K(Sigmabar,Q):
    C = array([[1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [0, 0, 1, 0, 0]])

    Ct = transpose(C)

    K = matmul(Sigmabar,matmul(Ct,inv(babt(Sigmabar,C) + Q)))
    # print(K)

    return K

def measurement_update_mu(mubar,K,z):
    C = array([ [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [0, 0, 1, 0, 0]])

    if abs(z[2] - mubar[2]) > abs(z[2] + 2*pi - mubar[2]):
        z[2] = z[2] + 2*pi
    elif abs(z[2] - mubar[2]) > abs(z[2] - 2*pi - mubar[2]):
        z[2] = z[2] - 2*pi

    mu = mubar + matmul(K,z-matmul(C,mubar))
    # mu[2]=mubar[2]#ignore phi measurements
    # print(mu)

    return mu

def measurement_update_Sigma(Sigmabar,K):
    C = array([[1, 0, 0, 0, 0],
               [0, 1, 0, 0, 0],
               [0, 0, 1, 0, 0]])

    Sigma = matmul(eye(5) - matmul(K,C),Sigmabar)

    return Sigma

# utility, returns B'AB
def btab(A,B):
    return matmul(transpose(B),matmul(A,B))

# utility, returns BAB'
def babt(A,B):
    return matmul(B,matmul(A,transpose(B)))


if __name__ == '__main__':
    try:
        rospy.init_node('satellite_state_ekf_publisher', anonymous=True)
        ekfnode = EkfNode()
        ekfnode.start()
    except rospy.ROSInterruptException:
        pass