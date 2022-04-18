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
from math import cos, sin
from numpy import transpose, matmul, array, eye
from numpy.linalg import inv

camera_flag = False
axy_sel = array([[1,0,0],[0,1,0],[0,0,0]])
wz_sel = array([[0,0,0],[0,0,0],[0,0,1]])
tprev = time()

covpos_sel = array([[1,0,0],[0,1,0],[0,0,0],[0,0,0],[0,0,0],[0,0,1]])
covtwist_sel = np.array([[1,0],[0,1],[0,0],[0,0],[0,0],[0,0]])

mu = array([0,0,0,0,0]).reshape(5,1)
Sigma = eye(5)

Q = eye(3)
z = array([0,0,0]).reshape(3,1)

def ekf_main():
    global pub
    pub = rospy.Publisher('satellite_state', Odometry, queue_size=10)
    rospy.init_node('satellite_state_ekf_publisher', anonymous=True)
    imu_sub = rospy.Subscriber('razor_imu', Imu, callback=imu_callback)
    aruco_sub = rospy.Subscriber('aruco_stamped', PoseWithCovarianceStamped, callback=aruco_callback)
    
    while not rospy.is_shutdown():
        rospy.spin()

def imu_callback(imu_msg):
    global axy_sel, wz_sel, u, M, tprev, camera_flag, mu, Sigma, Q, v, pub

    # print(imu_msg.linear_acceleration)
    lin_acc = array([[imu_msg.linear_acceleration.x],[imu_msg.linear_acceleration.y],[imu_msg.linear_acceleration.z]])
    ang_vel = array([[imu_msg.angular_velocity.x],[imu_msg.angular_velocity.y],[imu_msg.angular_velocity.z]])

    u = matmul(axy_sel,lin_acc) + matmul(wz_sel,ang_vel)

    cov_linear = array(imu_msg.linear_acceleration_covariance).reshape((3,3))
    cov_angular = array(imu_msg.angular_velocity_covariance).reshape((3,3))

    M = btab(cov_linear,axy_sel) + btab(cov_angular,wz_sel)

    # u = array([ax,ay,wz])
    # M = cov_linear
    # M[2,0:2] = 0
    # M[0:2,2] = 0
    # M[2,2] = cov_angular[2,2]

    print('u = ')
    print(u)
    print()
    print('M = ')
    print(M)
    print()

    print("mu = ")
    print(mu)


    dt = time()-tprev
    tprev = time()

    mubar = process_update_mu(mu,Sigma,u,M,dt)
    Sigmabar = process_update_Sigma(mu, Sigma, u, M, dt)
    

    if camera_flag:
        K = measurement_kalman_gain_K(Sigmabar,Q)
        mu = measurement_update_mu(mubar, K, z)
        Sigma = measurement_update_Sigma(Sigmabar, K)
        camera_flag = False
    else:
        mu = mubar
        Sigma = Sigmabar
    
    odom = Odometry()
    odom.pose.pose.position.x = mu[0]
    odom.pose.pose.position.y = mu[1]
    quat = quaternion_from_euler(0,0,float(mu[2]))
    odom.pose.pose.orientation.x = quat[0]
    odom.pose.pose.orientation.y = quat[1]
    odom.pose.pose.orientation.z = quat[2]
    odom.pose.pose.orientation.w = quat[3]
    
    odom.pose.covariance = babt(Sigmabar[0:3,0:3],covpos_sel).reshape(36,1)

    odom.twist.twist.linear.x = mu[3]
    odom.twist.twist.linear.y = mu[4]
    odom.twist.twist.angular.z = u[2]

    odom.twist.covariance = babt(Sigmabar[3:6,3:6],covtwist_sel).reshape(36,1)
    odom.twist.covariance[35] = M[2,2]
    odom.header.frame_id = "map"
    odom.header.stamp = rospy.Time.now()
    pub.publish(odom)



def aruco_callback(aruco_pose):
    global camera_flag, axy_sel, wz_sel, covpos_sel, z, Q
    camera_flag = True
    xyzposition = array([aruco_pose.pose.position.x,aruco_pose.pose.position.y,aruco_pose.pose.position.z]).reshape(3,1)
    quat = array([aruco_pose.pose.orientation.x,aruco_pose.pose.orientation.y,aruco_pose.pose.orientation.z,aruco_pose.pose.orientation.w])#.reshape(4,1)
    xyzangles = array(euler_from_quaternion(quat)).reshape(3,1)
    aruco_cov = array(aruco_pose.covariance).reshape(6,6)

    z = matmul(axy_sel,xyzposition) + matmul(axy_sel,xyzangles)
    Q = btab(aruco_cov,covpos_sel)

    print('z = ')
    print(z)
    print()
    print('Q = ')
    print(Q)
    print()

def process_update_mu(mu,Sigma,u,M,dt):
    print(mu)
    x = mu[0]
    y = mu[1]
    phi = mu[2]
    vx = mu[3]
    vy = mu[4]

    ax = u[0]
    ay = u[1]
    wz = u[2]

    xbar = x + vx*dt + 0.5*dt**2*(ax*cos(phi)-ay*sin(phi))
    ybar = y + vy*dt + 0.5*dt**2*(ax*sin(phi)+ay*cos(phi))
    phibar = phi + dt*wz
    vxbar = vx + dt*(ax*cos(phi)-ay*sin(phi))
    vybar = vy + dt*(ax*sin(phi)+ay*cos(phi))

    mubar = array([xbar, ybar, phibar, vxbar, vybar]).reshape(5,1)

    return mubar

def process_update_Sigma(mu,Sigma,u,M,dt):
    x = mu[0]
    y = mu[1]
    phi = mu[2]
    vx = mu[3]
    vy = mu[4]

    ax = u[0]
    ay = u[1]
    wz = u[2]

    # Jacobian of mubar wrt mu
    G = array([[1, 0, -0.5*dt**2*(ax*sin(phi) + ay*cos(phi)), dt,  0],
               [0, 1,  0.5*dt**2*(ax*cos(phi) - ay*sin(phi)),  0, dt],
               [0, 0,                                      1,  0,  0],
               [0, 0,        -dt*(ax*sin(phi) + ay*cos(phi)),  1,  0],
               [0, 0,         dt*(ax*cos(phi) - ay*sin(phi)),  0,  1]])

    # Jacobian of mubar wrt u
    V = array([[0.5*dt**2*cos(phi), -0.5*dt**2*sin(phi),  0],
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

    return K

def measurement_update_mu(mubar,K,z):
    C = array([[1, 0, 0, 0, 0],
               [0, 1, 0, 0, 0],
               [0, 0, 1, 0, 0]])
    mu = mubar + matmul(K,z-matmul(C,mubar))

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
        ekf_main()
    except rospy.ROSInterruptException:
        pass