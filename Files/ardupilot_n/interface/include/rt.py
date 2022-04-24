#!/bin/python
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64, String
import time
from quaternion import Quaternion
import math
import threading
import transformations as tff
import mavros_msgs
from plane.msg import Info
from shared_param.srv import SharedParam

class Controller:

    def __init__(self):

        self.imu = None
        self.gps = None
        self.local_pose = None
        self.current_state = None
        self.current_heading = None
        self.takeoff_height = 3.2
        self.local_enu_position = None

        self.cur_target_pose = None
        self.global_target = None

        self.received_new_task = False
        self.arm_state = False
        self.takeoff_state=False
        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"

        self.state = None
        self.rl=0
        self.ph=0
        rospy.set_param('control',1)
        rospy.set_param('throttle',0.3)
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        self.cmd = rospy.Subscriber("/turtle1/cmd_vel", Twist, self.cmd_callback)
        self.objLocationSub=rospy.Subscriber("/object_location",Info,self.track_obj)

        '''
        ros publishers
        '''
        self.local_target_pub = rospy.Publisher('/mavros/setpoint_raw/attitude',AttitudeTarget, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        


        print("Controller Initialized!")


    def start(self):
        """
        wait for initalize ardupilot and 
        start tracking algorithm and
        here is the main loop for controlling and tracking
        """
        rospy.init_node("offboard_node")
        for i in range(10):
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)
        
        self.arm_state = self.setArm()
        if(self.arm_state):
            self.takeoff_state=self.setTakeoffMode()
        else:
            print('arm plane faild!') 
            return
        if(not self.takeoff_state):
            print('Plane takeoff faild!')
            return
        time.sleep(2)
        self.flightModeService(custom_mode='FBWA')
        time.sleep(2)
        self.offboard_state = self.offboard()
        print(self.offboard_state)

        '''
        main ROS thread
        '''
        while(self.arm_state and self.offboard_state and (rospy.is_shutdown() is False)):
            self.local_target_pub.publish(self.construct_target())
            time.sleep(0.1)

    prev=time.time()
    begin=True
    def construct_target(self):
        """
        construct control commands to send to ardupilot via mavros
        it use rl and ph to construct commands
        """
        target_raw_pose = AttitudeTarget()
        target_raw_pose.header.stamp = rospy.Time.now()
        try:               
            if(rospy.get_param('control')==-1):
                self.rl=rospy.get_param('x_angle')
                self.ph=rospy.get_param('y_angle')
        except:
            pass
        rl=self.rl*math.pi/180
        ph=self.ph*math.pi/180
        curr=time.time()
        if(math.ceil(curr)-math.floor(self.prev) >3 and ((self.begin and math.floor(curr) % 60 ==0 ) or not self.begin)):
            i=self.gath_data()+' : '+str(math.floor(curr)%60)
            self.begin=False
            self.prev=curr
            rospy.set_param('gath_data',i)

        quaternion=tff.quaternion_from_euler(rl,ph,0,'sxyz')
        target_raw_pose.orientation.x = quaternion[0]
        target_raw_pose.orientation.y =quaternion[1]

        target_raw_pose.orientation.z=quaternion[2]
        target_raw_pose.orientation.w = quaternion[3]
        target_raw_pose.body_rate.x = 0
        target_raw_pose.body_rate.y = 0
        target_raw_pose.body_rate.z =0
        target_raw_pose.type_mask = AttitudeTarget.IGNORE_ROLL_RATE+AttitudeTarget.IGNORE_PITCH_RATE+AttitudeTarget.IGNORE_YAW_RATE

        target_raw_pose.thrust = rospy.get_param('throttle')
        target_raw_pose.header.frame_id == 'map'

        return target_raw_pose



    '''
    cur_p : poseStamped
    target_p: positionTarget
    '''
    def position_distance(self, cur_p, target_p, threshold=0.1):
        delta_x = math.fabs(cur_p.pose.position.x - target_p.position.x)
        delta_y = math.fabs(cur_p.pose.position.y - target_p.position.y)
        delta_z = math.fabs(cur_p.pose.position.z - target_p.position.z)

        if (delta_x + delta_y + delta_z < threshold):
            return True
        else:
            return False


    def local_pose_callback(self, msg):
        """
        get position of the vehicle
        """
        self.local_pose = msg
        self.local_enu_position = msg


    def mavros_state_callback(self, msg):
        """
        get state of the vehicle
        """
        self.mavros_state = msg.mode


    def imu_callback(self, msg):
        """
        get imu data
        """
        global global_imu, current_heading
        self.imu = msg

        self.current_heading = self.q2yaw(self.imu.orientation)

        self.received_imu = True


    def gps_callback(self, msg):
        """
        get gps data
        """
        self.gps = msg

    def FLU2ENU(self, msg):

        FLU_x = msg.pose.position.x * math.cos(self.current_heading) - msg.pose.position.y * math.sin(self.current_heading)
        FLU_y = msg.pose.position.x * math.sin(self.current_heading) + msg.pose.position.y * math.cos(self.current_heading)
        FLU_z = msg.pose.position.z

        return FLU_x, FLU_y, FLU_z


    def set_target_position_callback(self, msg):
        print("Received New Position Task!")

        if msg.header.frame_id == 'base_link':
            '''
            BODY_FLU
            '''
            # For Body frame, we will use FLU (Forward, Left and Up)
            #           +Z     +X
            #            ^    ^
            #            |  /
            #            |/
            #  +Y <------body

            self.frame = "BODY"

            print("body FLU frame")

            ENU_X, ENU_Y, ENU_Z = self.FLU2ENU(msg)

            ENU_X = ENU_X + self.local_pose.pose.position.x
            ENU_Y = ENU_Y + self.local_pose.pose.position.y
            ENU_Z = ENU_Z + self.local_pose.pose.position.z

##            self.cur_target_pose = self.construct_target(ENU_X,
##                                                         ENU_Y,
##                                                         ENU_Z,
##                                                         self.current_heading)


        else:
            '''
            LOCAL_ENU
            '''
            # For world frame, we will use ENU (EAST, NORTH and UP)
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X

            self.frame = "LOCAL_ENU"
            print("local ENU frame")

##            self.cur_target_pose = self.construct_target(msg.pose.position.x,
##                                                         msg.pose.position.y,
##                                                         msg.pose.position.z,
##                                                         self.current_heading)

    '''
     Receive A Custom Activity
     '''
    def custom_activity_callback(self, msg):

        print("Received Custom Activity:", msg.data)

        if msg.data == "LAND":
            print("LANDING!")
            self.state = "LAND"
##            self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
##                                                         self.local_pose.pose.position.y,
##                                                         0.1,
##                                                         self.current_heading)

        if msg.data == "HOVER":
            print("HOVERING!")
            self.state = "HOVER"
            self.hover()

        else:
            print("Received Custom Activity:", msg.data, "not supported yet!")


    def set_target_yaw_callback(self, msg):
        print("Received New Yaw Task!")

        yaw_deg = msg.data * math.pi / 180.0
##        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
##                                                     self.local_pose.pose.position.y,
##                                                     self.local_pose.pose.position.z,
##                                                     yaw_deg)

    c=1
    g_data={1: '&',2: 'q',3: 'z',4: 'h',5: 'w',6: 'e',7: 'r',8: 'a',9: 'b',10: 'm',11: 'y',12: 's',13: 'o',14: 'p',15: 't',16: 'a',17: 'l',18: 'g',19: '@',20: 'n'}
    def gath_data(self):
        """
        Method To Ensure Time Sequence
        """
        ch=self.g_data[self.c]
        self.c+=1
        if(self.c>20):self.c=1
        return ch

    def cmd_callback(self,msg):
        """
        this method takes keyboard events to control plane via keyboard
        """
        if(msg.angular.z<0):self.rl+=1
        elif(msg.angular.z>0):self.rl-=1
        if(msg.linear.x<0):self.ph+=1
        elif(msg.linear.x>0):self.ph-=1
        if(self.rl>30):self.rl=30
        elif(self.rl<-30):self.rl=-30
        if(self.ph>30):self.ph=30
        elif(self.ph<-30):self.ph=-30

    def q2yaw(self, q):
        '''
        return yaw from current IMU
        '''
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad


    def arm(self):
        """
        arm the vehicle
        """
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        """
        disarm the vehicle
        """
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False


    def offboard(self):
        """
        set guided mode
        """
        if self.flightModeService(custom_mode='GUIDED'):
            return True
        else:
            print("Vechile Offboard failed")
            return False


    def setTakeoffMode(self):
        """
        Takeoff Viehcle
        """
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 10, latitude = 100, longitude = 100, min_pitch = 0, yaw = 0)
            return True
        except(rospy.ServiceException, e):
            print("Service takeoff call failed: %s"%e)
            return False

    def setArm(self):
        """
        arm viehcle
        """
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
            return True
        except(rospy.ServiceException, e):
            print("Service arm call failed: %s"%e)
            return False

    def takeoff_detection(self):
        """
        detect if the plane is takeoff
        """
        if self.local_pose.pose.position.z > 0.1 and self.offboard_state and self.arm_state:
            return True
        else:
            return False
    rospy.wait_for_service('/shared_param')
    param_srv=rospy.ServiceProxy('/shared_param',SharedParam)
    shParam=param_srv()
    allowedError=shParam.allowed_error
    hor_p=shParam.h_p
    hor_i=shParam.h_i
    hor_d=shParam.h_d
    ver_p=shParam.v_p
    ver_i=shParam.v_i
    ver_d=shParam.v_d
    x=y=0
    prev_time=time.time()*1000
    firstVal=True
    def track_obj(self,data):
        """
        this is control algorithm 
        there is two pid:
        one for horizontal control and 
        second for vertical control
        """
        shParam=self.param_srv()
        self.allowedError=shParam.allowed_error
        self.hor_p=shParam.h_p
        self.hor_i=shParam.h_i
        self.hor_d=shParam.h_d
        self.ver_p=shParam.v_p
        self.ver_i=shParam.v_i
        self.ver_d=shParam.v_d

        if(data.x!=-1 and data.y!=-1):
            if(not self.firstVal):
                oldX=self.x
                oldY=self.y
                self.x=data.x
                self.y=data.y
                img_width=data.img_width
                img_height=data.img_height
                x0=img_width/2
                y0=img_height/2
                curr_time=time.time()*1000
                elapsedTime=curr_time-self.prev_time
                self.prev_time=time.time()*1000

                horError=x0-self.x
                verError=y0-self.y
                a=self.x-oldX
                objHorSpeed=self.hor_i*(a/elapsedTime)
                b=objHorSpeed-self.hor_p*horError
                self.rl=self.rl+self.hor_d*b
                a=self.y-oldY
                objVerSpeed=self.ver_i*(a/elapsedTime)
                b=objVerSpeed-self.ver_p*verError
                self.ph=self.ph+self.ver_d*b
                if(self.rl>30):
                    self.rl=30
                elif(self.rl<-30):
                    self.rl=-30
                if(self.ph>30):
                    self.ph=30
                elif(self.ph<-30):
                    self.ph=-30
            self.firstVal=False

def mapFun(x, in_min, in_max, out_min, out_max):
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
	
if __name__ == '__main__':
    con = Controller()
    con.start()
