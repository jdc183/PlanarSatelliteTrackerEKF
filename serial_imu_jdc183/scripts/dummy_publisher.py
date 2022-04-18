#!/usr/bin/env python
import rospy
import serial
import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
import numpy as np
from numpy import array
from math import cos, sin, pi

g = 9.81
lincov = 0.25**2
angcov = 0.002**2
poscov = 0.005**2
phicov = 0.01**2
r = 1
w = 0.5

def read_imu():
    imupub = rospy.Publisher('razor_imu', Imu, queue_size=10)
    arucopub = rospy.Publisher('aruco_stamped', PoseWithCovarianceStamped, queue_size = 10)
    rospy.init_node('dummy_publisher', anonymous=True)
    rate = rospy.Rate(100)
    imumsg = Imu()
    arucomsg = PoseWithCovarianceStamped()
    i = 0
    while not rospy.is_shutdown():
        u = lemniscate_sample_trajectory_u(time.time())
        
        # print("accxyz:\t" + str(accx) + "\t" + str(accy) + "\t" + str(accz) + "\tgyrxyz:\t" + str(gyrx) + "\t" + str(gyry) + "\t" + str(gyrz))]
        # rospy.loginfo("accxyz:\t" + str(accx) + "\t" + str(accy) + "\t" + str(accz) + "\tgyrxyz:\t" + str(gyrx) + "\t" + str(gyry) + "\t" + str(gyrz))
        imumsg.linear_acceleration.x = u[0]
        imumsg.linear_acceleration.y = u[1]
        imumsg.angular_velocity.z = u[2]

        imumsg.linear_acceleration_covariance[0] = lincov
        imumsg.linear_acceleration_covariance[4] = lincov
        imumsg.linear_acceleration_covariance[8] = lincov

        imumsg.angular_velocity_covariance[0] = angcov
        imumsg.angular_velocity_covariance[4] = angcov
        imumsg.angular_velocity_covariance[8] = angcov

        imumsg.header.stamp = rospy.Time.now()
        imupub.publish(imumsg)

        if i % int(np.random.uniform(10,120)) == 0:
            i = 1
            z = lemniscate_sample_trajectory_z(time.time())
            arucomsg.pose.pose.position.x = z[0]
            arucomsg.pose.pose.position.y = z[1]
            arucomsg.pose.pose.orientation.x = quaternion_from_euler(0,0,float(z[2]))[0]
            arucomsg.pose.pose.orientation.y = quaternion_from_euler(0,0,float(z[2]))[1]
            arucomsg.pose.pose.orientation.z = quaternion_from_euler(0,0,float(z[2]))[2]
            arucomsg.pose.pose.orientation.w = quaternion_from_euler(0,0,float(z[2]))[3]

            # arucomsg.pose.pose.orientation.x = 0.651878653439
            # arucomsg.pose.pose.orientation.y = 0.686701781443
            # arucomsg.pose.pose.orientation.z = -0.18640260571
            # arucomsg.pose.pose.orientation.w = -0.262200215746

            arucomsg.pose.covariance[0] = poscov
            arucomsg.pose.covariance[7] = poscov
            arucomsg.pose.covariance[14] = poscov
            arucomsg.pose.covariance[21] = phicov
            arucomsg.pose.covariance[28] = phicov
            arucomsg.pose.covariance[35] = phicov

            arucomsg.header.stamp = rospy.Time.now()
            arucomsg.header.frame_id = "map"
            
            arucopub.publish(arucomsg)
        else:
            i = i+1

        rate.sleep()

def lemniscate_sample_trajectory_mu(t):
    x = r*cos(w*t)/(1+sin(w*t)**2)
    y = r*sin(w*t)*cos(w*t)/(1+sin(w*t)**2)
    phi = w*t
    # phi = ((phi + pi) % (2*pi)) - pi

    vx = -(r*w*sin(w*t)*(sin(w*t)**2+2*cos(w*t)**2+1))/(sin(w*t)**2+1)**2
    vy = -(r*w*(sin(w*t)**4+(cos(w*t)**2+1)*sin(w*t)**2-cos(w*t)**2))/(sin(w*t)**2+1)**2

    mu = array([[x],[y],[phi],[vx],[vy]])
    return mu

def lemniscate_sample_trajectory_u(t):
   #ax1 = (r.*w.^2.*cos(w.*t).*(5.*sin(w.*t).^4+(6.*cos(w.*t).^2+4).*sin(w.*t).^2-2.*cos(w.*t).^2-1))./(sin(w.*t).^2+1).^3;
    ax1 = (r*w**2*cos(w*t)*(5*sin(w*t)**4+(6*cos(w*t)**2+4)*sin(w*t)**2-2*cos(w*t)**2-1))/(sin(w*t)**2+1)**3
   #ay1 = (2.*r.*w.^2.*cos(w.*t).*sin(w.*t).*(sin(w.*t).^4+(cos(w.*t).^2-1).*sin(w.*t).^2-3.*cos(w.*t).^2-2))./(sin(w.*t).^2+1).^3;
    ay1 = (2*r*w**2*cos(w*t)*sin(w*t)*(sin(w*t)**4+(cos(w*t)**2-1)*sin(w*t)**2-3*cos(w*t)**2-2))/(sin(w*t)**2+1)**3
  
   #ax = ax1.*cos(theta)+ay1.*sin(theta);
    ax = ax1*cos(w*t)+ay1*sin(w*t) + np.random.normal(0,lincov,1)
   #ay =-ax1.*sin(theta)+ay1.*cos(theta);
    ay =-ax1*sin(w*t)+ay1*cos(w*t) + np.random.normal(0,lincov,1)
    # az = -9.81

    wz = w + np.random.normal(0,angcov,1)

    u = array([[ax], [ay], [wz]])
    return u

def lemniscate_sample_trajectory_z(t):
    x = r*cos(w*t)/(1+sin(w*t)**2) + np.random.normal(0,poscov,1)
    y = r*sin(w*t)*cos(w*t)/(1+sin(w*t)**2) + np.random.normal(0,poscov,1)
    phi = w*t + np.random.normal(0,phicov,1)
    # phi = ((phi + pi) % (2*pi)) - pi
    # phi = phi - int(phi/2/pi)*2*pi

    z = array([[x],[y],[phi]])
    return z

def rest_sample_trajectory_mu(t):
    x = 0
    y = 0
    phi = pi/3

    vx = 0
    vy = 0

    mu = array([[x],[y],[phi],[vx],[vy]])
    return mu

def rest_sample_trajectory_u(t):
    ax1 = -0.2
    ay1 = 0
    w = 0.05
  
    ax = round((ax1*cos(w**t)+ay1*sin(w**t) + np.random.normal(0,lincov,1) )*10)/10
    ay = round((-ax1*sin(w**t)+ay1*cos(w**t) + np.random.normal(0,lincov,1) )*10)/10
    # az = -9.81

    wz = w + np.random.normal(0,angcov,1)

    u = array([[ax], [ay], [wz]])
    return u

def rest_sample_trajectory_z(t):
    x = -0.0259 + np.random.normal(0,poscov,1)
    y = 0.118 + np.random.normal(0,poscov,1)
    phi = pi/3# + np.random.normal(0,phicov,1)

    z = array([[x],[y],[phi]])
    return z

if __name__ == '__main__':
    try:
        read_imu()
    except rospy.ROSInterruptException:
        pass