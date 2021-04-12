# source /opt/ros/melodic/setup.bash
# cd /home/pi/ros_catkin_ws/src/dog_snout/src/ && python combine_dog_snout_T265.py
# https://stackoverflow.com/questions/37373211/update-the-global-variable-in-rospy

# tegwyn@tegwyn-Xavier:~$ rostopic find sensor_msgs/Imu
# /camera/accel/sample
# /camera/gyro/sample
# /camera/accel/imu_info


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from sensor_msgs.msg import Imu as msg_Imu
from ublox_dog_msgs.msg import NavPOSECEF
import os
import time
from datetime import datetime

class Nodo(object):
    def __init__(self):
        # Params
        self.x1 = 0.0
        self.y1 = 0.0

        self.x2 = 0.0
        self.y2 = 0.0
        #self.a = None

        self.newX = 0.0
        self.newY = 0.0
        
        self.Imu_lin_accel = 0.0
        self.Imu_ang_vel = 0.0
        
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        #self.pub = rospy.Publisher("~chatter1", std_msgs.msg.Float64, queue_size=10)

        # Subscribers
        #rospy.Subscriber("/ublox_dog_gps/fix", NavSatFix, callback_dog)
        rospy.Subscriber("/ublox_dog_gps/navposecef", NavPOSECEF, self.callback_dog_navposecef)    
        #rospy.Subscriber("/ublox_snout_gps/fix", NavSatFix, callback_snout)
        rospy.Subscriber("/ublox_snout_gps/navposecef", NavPOSECEF, self.callback_snout_navposecef)

        # rospy.Subscriber("/camera/odom/sample", msg_Imu, self.callback_Imu_accel, queue_size=5)
        rospy.Subscriber("/camera/accel/sample", msg_Imu, self.callback_Imu_accel)
        rospy.Subscriber("/camera/gyro/sample", msg_Imu, self.callback_Imu_gyro)

    def callback_Imu_accel(self, msg):
        #self.a = msg.data
        self.Imu_lin_accel = msg.linear_acceleration
        # rospy.loginfo("IMU x: {}".format(msg.linear_acceleration))
        
    def callback_Imu_gyro(self, msg):
        #self.a = msg.data
        self.Imu_ang_vel = msg.angular_velocity
        # rospy.loginfo("IMU x: {}".format(msg.angular_velocity))
        
    def callback_dog_navposecef(self, msg):
        #self.a = msg.data
        self.x1 = msg.ecefX
        self.y1 = msg.ecefY
        # rospy.loginfo("DOG ecefX: {}".format(msg.ecefX))
        # rospy.loginfo("DOG ecefY: {}".format(msg.ecefY))

    def callback_snout_navposecef(self, msg):
        #self.a = msg.data
        self.x2 = msg.ecefX 
        self.y2 = msg.ecefY 
        # rospy.loginfo("SNOUT exefX: {}".format(msg.ecefX))
        # rospy.loginfo("SNOUT exefY: {}".format(msg.ecefY))

    def start(self):
        rospy.loginfo("COMBINE DOG SNOUT")
        # file = open(/home/pi/ros_catkin_ws/src/dog_snout/src/GPS_and_camera_log.csv)

        while not rospy.is_shutdown():
                # self.pub.publish(self.x1)
                # now = datetime.now()
                # file.write(str(now) + "," + str(self.x1) + "," + str(self.y1) + "," + str(self.Imu_lin_accel) + "," + str(self.Imu_ang_vel) + "\n")
                # file.flush()
                print("MAIN DOG X1 " + (str)(self.x1))
                print("MAIN DOG Y1 " + (str)(self.y1))
                print ("---------")
                print("MAIN SNOUT X2 " + (str)(self.x2))
                print("MAIN SNOUT Y2 " + (str)(self.y2))

                self.newX = (self.x1 - self.x2)
                self.newY = (self.y1 - self.y2)
                print("NEW X Y " +(str)(self.newX) + " : " + (str)(self.newY))
                print ("---------")
                print("IMU linear accel: ")
                print(str(self.Imu_lin_accel))
                print ("---------")
                print("IMU angular velocity: ")
                print(str(self.Imu_ang_vel))
                print("")
                               
                self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("dog_snout", anonymous=True)
    my_node = Nodo()
    my_node.start()
