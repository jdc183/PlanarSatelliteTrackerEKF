#!/usr/bin/env python
import rospy
import serial
import time
from sensor_msgs.msg import Imu
import numpy as np

razor = serial.Serial(port='/dev/ttyACM0',baudrate=9600, timeout=0.1)

g = 9.81
pi = np.pi
lincov = 0.02**2 #m-s^-2 not g
angcov = 0.002**2

def read_imu():
    pub = rospy.Publisher('razor_imu', Imu, queue_size=10)
    rospy.init_node('razor_publisher', anonymous=True)
    # rate = rospy.Rate(1000)
    imumsg = Imu()

    accxbar = 0
    accybar = 0
    acczbar = 0
    gyrxbar = 0
    gyrybar = 0
    gyrzbar = 0

    for i in range(500):
        line = str(razor.readline()).split(',')
        if len(line)==10:
            accx = float(line[1])*g
            accy = float(line[2])*g
            accz = float(line[3])*g
            gyrx = float(line[4])*pi/180
            gyry = float(line[5])*pi/180
            gyrz = float(line[6])*pi/180

            accxbar = (i*accxbar+accx)/(i+1)
            accybar = (i*accybar+accy)/(i+1)
            acczbar = (i*acczbar+accz)/(i+1)
            gyrxbar = (i*gyrxbar+gyrx)/(i+1)
            gyrybar = (i*gyrybar+gyry)/(i+1)
            gyrzbar = (i*gyrzbar+gyrz)/(i+1)

    while not rospy.is_shutdown():
        line = str(razor.readline()).split(',')
        # print(len(line))
        if len(line)==10:
            accx = float(line[1])*g
            accy = float(line[2])*g
            accz = float(line[3])*g
            gyrx = float(line[4])*pi/180
            gyry = float(line[5])*pi/180
            gyrz = float(line[6])*pi/180
            # print("accxyz:\t" + str(accx) + "\t" + str(accy) + "\t" + str(accz) + "\tgyrxyz:\t" + str(gyrx) + "\t" + str(gyry) + "\t" + str(gyrz))]
            # rospy.loginfo("accxyz:\t" + str(accx) + "\t" + str(accy) + "\t" + str(accz) + "\tgyrxyz:\t" + str(gyrx) + "\t" + str(gyry) + "\t" + str(gyrz))
            imumsg.linear_acceleration.x = accx-accxbar
            imumsg.linear_acceleration.y = accy-accybar
            imumsg.linear_acceleration.z = accz-acczbar
            imumsg.angular_velocity.x = gyrx-gyrxbar
            imumsg.angular_velocity.y = gyry-gyrybar
            imumsg.angular_velocity.z = gyrz-gyrzbar

            imumsg.linear_acceleration_covariance[0] = lincov
            imumsg.linear_acceleration_covariance[4] = lincov
            imumsg.linear_acceleration_covariance[8] = lincov

            imumsg.angular_velocity_covariance[0] = angcov
            imumsg.angular_velocity_covariance[4] = angcov
            imumsg.angular_velocity_covariance[8] = angcov

            imumsg.header.stamp = rospy.Time.now()
            pub.publish(imumsg)
            # rate.sleep()

        

if __name__ == '__main__':
    try:
        read_imu()
    except rospy.ROSInterruptException:
        pass