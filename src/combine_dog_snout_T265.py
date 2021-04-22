# source /opt/ros/melodic/setup.bash
# cd /home/tegwyn/catkin_ws/src/dog_snout/src/ && python3 combine_dog_snout_T265_test.py
# cd /home/tegwyn/catkin_ws/src/dog_snout/src/ && python3 combine_dog_snout_T265.py
# https://stackoverflow.com/questions/37373211/update-the-global-variable-in-rospy
# export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.6/pyrealsense2

import sys
sys.path.append('/usr/local/lib/python3.6/pyrealsense2')
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages${PYTHONPATH:+:${PYTHONPATH}}')
sys.path.append('/home/tegwyn/catkin_ws/devel/lib')

sys.path.append('/home/tegwyn/catkin_ws/devel/lib/python3/dist-packages')
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/home/tegwyn/catkin_ws/devel/lib/')

# First import the library
import pyrealsense2 as rs

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from ublox_dog_msgs.msg import NavPOSECEF
import os
import time
from datetime import datetime

class Nodo(object):
    def __init__(self):
		
	# Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()

        # Build config object and request pose data
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)

        # Start streaming with requested config
        self.pipe.start(cfg)
		
        # Params
        self.x1 = 0.0
        self.y1 = 0.0

        self.x2 = 0.0
        self.y2 = 0.0
        #self.a = None

        self.newX = 0.0
        self.newY = 0.0

        
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(5)

        # Publishers
        #self.pub = rospy.Publisher("~chatter1", std_msgs.msg.Float64, queue_size=10)

        # Subscribers
        #rospy.Subscriber("/ublox_dog_gps/fix", NavSatFix, callback_dog)
        rospy.Subscriber("/ublox_dog_gps/navposecef", NavPOSECEF, self.callback_dog_navposecef)    
        #rospy.Subscriber("/ublox_snout_gps/fix", NavSatFix, callback_snout)
        rospy.Subscriber("/ublox_snout_gps/navposecef", NavPOSECEF, self.callback_snout_navposecef)

        # rospy.Subscriber("/camera/odom/sample", msg_Imu, self.callback_Imu_accel, queue_size=5)
        # rospy.Subscriber("/camera/accel/sample", msg_Imu, self.callback_Imu_accel)
        # rospy.Subscriber("/camera/gyro/sample", msg_Imu, self.callback_Imu_gyro)
        
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
        file = open("/home/tegwyn/catkin_ws/src/dog_snout/src/GPS_and_camera_log.csv","a")
        file.write("TimeStamp,dogX1,dogY1,headingX,headingY,positionX,positionY,positionZ,velocityX,velocityY,velocityZ,accelerationX,accelerationY,accelerationZ"+"\n")
        # file.write("TimeStamp,dogX1,dogY1,positionX,positionY,positionZ,velocityX,velocityY,velocityZ,accelerationX,accelerationY,accelerationZ"+"\n")
        # file.write("positionX,positionY,positionZ,velocityX,velocityY,velocityZ,accelerationX,accelerationY,accelerationZ,dogX1,dogY1,headingX,headingY"+"\n")

        while not rospy.is_shutdown():
                # self.pub.publish(self.x1)
                self.newX = (self.x1 - self.x2)
                self.newY = (self.y1 - self.y2)
    
                print("MAIN DOG X1 " + (str)(self.x1))
                print("MAIN DOG Y1 " + (str)(self.y1))
                print ("---------")
                print("MAIN SNOUT X2 " + (str)(self.x2))
                print("MAIN SNOUT Y2 " + (str)(self.y2))

                print("NEW X Y " +(str)(self.newX) + " : " + (str)(self.newY))
                print ("---------")

                # Wait for the next set of frames from the camera
                frames = self.pipe.wait_for_frames()

                # Fetch pose frame
                pose = frames.get_pose_frame()
                if pose:
                    # Print some of the pose data to the terminal
                    data = pose.get_pose_data()
                    print("Frame #{}".format(pose.frame_number))
                    print("Position: {}".format(data.translation))
                    print("Velocity: {}".format(data.velocity))
                    print("Acceleration: {}\n".format(data.acceleration))

                now = str(datetime.now())+','

                dogX1 = (str)(self.x1)+','
                dogY1 = (str)(self.y1)+','

                snoutX2 = (str)(self.x2)+','
                snoutY2 = (str)(self.y2)+','

                headingX = (str)(self.newX)+','
                headingY = (str)(self.newY)+','

                positionXYZ = (str)(data.translation)
                velocityXYZ = (str)(data.velocity)
                accelXYZ = (str)(data.acceleration)

                positionXYZ = positionXYZ.replace('x:','')
                positionXYZ = positionXYZ.replace('y:','')
                positionXYZ = positionXYZ.replace('z:','')

                velocityXYZ = velocityXYZ.replace('x:','')
                velocityXYZ = velocityXYZ.replace('y:','')
                velocityXYZ = velocityXYZ.replace('z:','')

                # print("Acceleration_0:  ",accelXYZ)
                accelXYZ = accelXYZ.replace('x:','')
                # print("Acceleration_1:  ",accelXYZ)
                accelXYZ = accelXYZ.replace('y:','')
                # print("Acceleration_2:  ",accelXYZ)
                accelXYZ = accelXYZ.replace('z:','')
                # print("Acceleration_3:  ",accelXYZ)



                # data2write = positionXYZ + "," + velocityXYZ + "," + accelXYZ + "," + dogX1 + dogY1 + headingX + headingY
                # print("all data to write: ", data2write)

                # file.write(data2write + "\n")
                # file.write(now + dogX1 + dogY1 + positionXYZ + velocityXYZ + accelXYZ + "\n")
                file.write(now + dogX1 + dogY1 + headingX + headingY + positionXYZ + "," + velocityXYZ + "," + accelXYZ + "\n")
                file.flush()
                self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("dog_snout", anonymous=True)
    my_node = Nodo()
    my_node.start()
