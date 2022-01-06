#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose2D
# TD5 : We add the odometry message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from random import random
from std_msgs.msg import Int16MultiArray

import math

from evry_project_plugins.srv import DistanceToFlag

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

class Flag:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.ID = 0
        self.alreadyFound = 0


class Robot:
    def __init__(self, group, robot_name, nb_flags):
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0 #Sonar distance
        self.flagSpeed = 0.0 #Flag speed
        self.prevFlagDis = 0.0 #Previous flag distance - for computing the velocity relative to the flag        
        self.flagDir = 0 #Flag direction = let know 	
        self.prevFlagSpeed = 0
        self.flagIsTracked = 0
        self.isTurning = 0
        self.justFoundFlag = 1 # flag for avoiding wrong calculation of the flag velocity after a stop 
        self.flagList = Int16MultiArray()
        self.flagList.data = []
        self.flagList.data = [0,0,0,0,0,0,0,0]
        self.flagS = Int16MultiArray()
        self.flagS.data = []
        self.flagS.data = [0,0,0,0,0,0,0,0]
        
        
        
        self.vel = 1 # max forward velocity
        self.ang = 1 # max angular velocity
	
	

        # TD5 : We add the pose of the robot as global parameters
        self.x = 0.0
        self.y = 0.0
        # Quaternion
        self.yaw = 0.0 # Yaw angle rotation
        #ns : Name of the robot, like robot_A1, robot_A2 etc.
        #To be used for your subscriber and publisher with the robot itself
        self.group = group
        self.robot_name = robot_name
        self.ns = self.group + "/" + self.robot_name

        self.nb_flags = nb_flags    #Number of flags to discover in the environment

        '''Listener and publisher'''

        rospy.Subscriber(self.ns + "/sensor/sonar_front", Range, self.callbacksonar)
        self.cmd_vel_pub = rospy.Publisher(self.ns + "/cmd_vel", Twist, queue_size = 1)
        self.cmd_pos_pub = rospy.Publisher(self.ns + "/odom", Odometry, queue_size = 1)
        
        self.pub = rospy.Publisher("flags", Int16MultiArray, queue_size=1)
        rospy.Subscriber("flags", Int16MultiArray, self.callbackFlags)
        
        # TD5 : We listen the pose of the robot
        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackpose)
        self.pub_velocity() #Run the publisher once

    def callbackFlags(self,data):
        print(" ")
## flag publisher function - not valid yet   
#        print("HI THERE111111111111111 ")
#        self.flagList = data
#        self.flagS = data
#        print(self.flagList.data)
#        print("HI THERE44444444444444 ")

    def callbackpose(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w

        _, _, self.yaw = euler_from_quaternion(qx, qy, qz, qw)
        #print(self.yaw)

    def callbacksonar(self,data):
        self.sonar = data.range

    def get_sonar(self):
        return self.sonar

    def get_flagSpeed(self):
        return self.flagSpeed

    def get_flagAcc(self):
        return self.flagAcc

    def set_speed_angle(self,speed,angle):
        self.speed = speed
        self.angle = angle
        self.pub_velocity()

    def pub_velocity(self):
        self.speed = min(2, self.speed) # Maximum speed at 2 m/s

        cmd_vel = Twist()
        cmd_vel.linear.x = self.speed
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0

        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = self.angle

        self.cmd_vel_pub.publish(cmd_vel)
        
    def pub_position(self):
        pos = Odometry()
        pos.pose.pose.position.x = 20
        pos.pose.pose.position.y = 20
        
        self.cmd_pos_pub.publish(pos)
            

    def getDistanceToFlag(self):
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            # TD5 : We update the pose into the service
            pose.x = self.x
            pose.y = self.y
            pose.theta = 0.0
            result = service(pose)
            

            return result.distance
        except rospy.ServiceException as e :
            print("Service call failed: %s"%e)
            


            return result.distance, result.id_flag
        except rospy.ServiceException as e :
            print("Service call failed: %s"%e)
            
            
    def getFlagName(self):
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            # TD5 : We update the pose into the service
            pose.x = self.x
            pose.y = self.y
            pose.theta = 0.0
            result = service(pose)
            

            return result.id_flag
        except rospy.ServiceException as e :
            print("Service call failed: %s"%e)


    def computeVelocities(self,tp):
        # this function computes the relative velocity of the robot to the nearrest flag
        if self.justFoundFlag > 2:
            self.prevFlagSpeed = self.flagSpeed
            self.flagSpeed = (self.prevFlagDis - self.getDistanceToFlag())/tp
            self.prevFlagDis = self.getDistanceToFlag()
        self.justFoundFlag += 1
        
    def velSignChange(self):
        # sensing transition point meaning that a flag was found
        if self.prevFlagSpeed > 0 and self.flagSpeed < 0 and self.justFoundFlag > 4:
            vel = 0
            ang = 0 
            # if the transition was detected the coordinates will be computed and saved
            self.saveFlagCoords()
        else:
            vel = self.vel
            ang = 0 
        return vel, ang   
    
    
    def setOrientation(self, goal):
        # this function sets robot's orientation for an angle given as an argument. Not very elegant since while is used, but it does its job well
        while goal > 3.1415:
            goal = goal - 2*3.1415
        while goal < -3.1415:
            goal = goal + 2*3.1415
        if ((self.yaw - goal) >= 0 and (self.yaw - goal) < 3.1415) or ((self.yaw - goal) >= -2*3.1415 and (self.yaw - goal) < -3.1415):
            while abs(self.yaw - goal) > 0.1:
                self.set_speed_angle(0, -self.ang)
                self.isTurning = 1
                rospy.sleep(0.01)
        else:                          
            while abs(self.yaw - goal) > 0.1:
                self.set_speed_angle(0, self.ang)
                self.isTurning = 1
                rospy.sleep(0.01)       
        self.set_speed_angle(0, 0)            
       
        
    def checkFlagSide(self):
        # after a flag was found then it needs to be checked on which side of the robot it is. To do this first it takes a 90 degrees turn, then it moves forward and checks if the distance grew or became smaller. If it became smaller, it means that the flag was on the side of the turn.
        initDist = self.getDistanceToFlag()
        initYaw = self.yaw
        
        self.setOrientation(initYaw + 1.5708)
        self.set_speed_angle(self.vel, 0)
        rospy.sleep(0.3)
        
        finalDist = self.getDistanceToFlag()
        
        self.set_speed_angle(-self.vel, 0)
        rospy.sleep(0.3)
        
        self.setOrientation(initYaw)
        
        if initDist > finalDist:
            return 1 # return 1 in case that the flag was on the left side 
        else:
            return 0 # return 0 in case that the flag was on the right side
            
    def saveFlagCoords(self):
        # this function is responsible for computing, printint and publishing the coordinates of the flag
    
        robotX = self.x
        robotY = self.y
        dist = self.getDistanceToFlag()
        yaw = self.yaw
        offset = 0.75;
        
        # checking whether the flag was on the right or on the left
        if self.checkFlagSide() == 1:
            x = robotX - dist * math.sin(yaw) - offset * math.cos(yaw)
            y = robotY + dist * math.cos(yaw) - offset * math.sin(yaw)
            x = int(round(x))
            y = int(round(y))
#            print("Flag on the left")
        else:
            x = robotX + dist * math.sin(yaw) + offset * math.cos(yaw)
            y = robotY - dist * math.cos(yaw) + offset * math.sin(yaw)
            x = int(round(x))
            y = int(round(y))
#            print("Flag on the right")
            
        alreadyfound = 1
        self.justFoundFlag = 0
        
        s = str(self.getFlagName())
        
        
        print("The flag: "+ s +" was found by: "+rospy.get_param("~robot_name") + ". Coords: " )
        print("X: " + str(x))
        print("Y: " + str(y))


## publishing the coordinates - not valid yet
#        if self.flagList.data[self.getFlagName()-1] == 0:
#            self.flagList.data[self.getFlagName()-1] = 1
#            self.flagS = self.flagList
#            rospy.loginfo(self.flagS)
#            self.pub.publish(self.flagS)
            
        
        
        
    def moveTo(self,GX,GY):
        # this function sets the orientation of the robot such that it heads toward a point given as an argument to the function 
        pose = Pose2D()
        pose.x = self.x
        pose.y = self.y
        angle = (math.atan((pose.y-GY)/(pose.x-GX))+math.pi)
#        dist = ((pose.x-GX)**2+(pose.y-GY)**2)**0.5
        self.setOrientation(angle)
        self.set_speed_angle(1,0)
    
    
    
def run_demo():
    '''Main loop'''
    group = rospy.get_param("~group")
    robot_name = rospy.get_param("~robot_name")
    nb_flags = rospy.get_param("nb_flags")
    robot = Robot(group, robot_name, nb_flags)
    print("Robot : " + str(robot_name) +" from Group : " + str(group) + " is starting..")
    k=1
    robot.pub_position()
    
    # starting velocities
    velocity = 1 
    angle = 0

    # at the beginnnig, the robots are told to go different directions on the map to make the searching efficient from the beginning    
    if(robot_name == "robot_1"):
        robot.moveTo(-47,-22)
    if(robot_name == "robot_3"):
        robot.moveTo(-47,22)
        
    robot.set_speed_angle(1,0)
    while not rospy.is_shutdown():
    
        #Write here your strategy..
    
        samplingTime = 0.65
        
        # computing the relative velocity of the robot to the nearrest flag
        robot.computeVelocities(samplingTime)
        # checking if the transition from positive to negative velocity was detected
        velocity, angle = robot.velSignChange()

        sonar = float(robot.get_sonar())
        # obstacle avoidance by making a random turn 
        if sonar < 4:
            robot.setOrientation(random()*2*3.1415-3.1415)
        
        #Finishing by publishing the desired speed. DO NOT TOUCH.
        robot.set_speed_angle(velocity,angle)
        rospy.sleep(samplingTime)



if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous = True)


    run_demo()