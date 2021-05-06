import time
import datetime
import os
import tellopy
import numpy
import av
import cv2
import urllib.request
import pyrebase
from pynput import keyboard
from simple_pid import PID
from FPS import FPS
from pathlib import *
from OP import *
from math import pi, atan2, sqrt, fabs, degrees, asin
from operator import itemgetter
from CameraMorse import CameraMorse, RollingGraph

def url_to_image(url):
    # download the image, convert it to a NumPy array, and then read
    # it into OpenCV format
    with urllib.request.urlopen(url) as url:
        image = np.asarray(bytearray(url.read()), dtype="uint8")
        image = cv2.imdecode(image, cv2.IMREAD_COLOR)
    # return the image
    return image

def distance (A, B):
    """
        Calculate the square of the distance between points A and B
    """
    return int(sqrt((B[0]-A[0])**2 + (B[1]-A[1])**2))

def angle (A, B, C):
    """
        Calculate the angle between segment(A,p2) and segment (p2,p3)
    """
    if A is None or B is None or C is None:
        return None
    return degrees(atan2(C[1]-B[1],C[0]-B[0]) - atan2(A[1]-B[1],A[0]-B[0]))%360

def vertical_angle (A, B):
    """
        Calculate the angle between segment(A,B) and vertical axe
    """
    if A is None or B is None:
        return None
    return degrees(atan2(B[1]-A[1],B[0]-A[0]) - pi/2)

def quat_to_roll_deg(qx,qy,qz,qw):
    degree = pi/180
    sinr_cosp = 2 * (qw*qx+qy*qz)
    cosr_cosp = 1 - 2*(qx*qx+qy*qy)
    roll = int(atan2(sinr_cosp,cosr_cosp)/degree)
    return roll

def quat_to_pitch_deg(qx,qy,qz,qw):
    degree = pi/180
    sinp = 2 * (qw*qy-qz*qx)
    if fabs(sinp) >= 1 :
        pitch = int(copysign(M_PI/2,sinp)/degree)
    else :
        pitch = int(asin(sinp)/degree)
    return pitch

def quat_to_yaw_deg(qx,qy,qz,qw):
    """
        Calculate yaw from quaternion
    """
    degree = pi/180
    siny_cosp = 2 * (qw*qz+qx*qy)
    cosy_cosp = 1 - 2*(qy*qy+qz*qz)
    yaw = int(atan2(siny_cosp,cosy_cosp)/degree) # ( ~180 ~ 179 )
    return yaw

def main():
    """ Create a tello controller and show the video feed."""
    tello = TelloCV()

    first_frame = True  
    frame_skip = 300

    logo = url_to_image("https://ifh.cc/g/yVjEqx.png") #cv2.imread("/home/seny/Downloads/33.png",cv2.IMREAD_UNCHANGED)

    for frame in tello.container.decode(video=0):

        if tello.record:
            tello.record_vid(frame)
        if 0 < frame_skip:
            frame_skip = frame_skip - 1
            continue
        start_time = time.time()
        if frame.time_base < 1.0/60:
            time_base = 1.0/60
        else:
            time_base = frame.time_base

        # Convert frame to cv2 image
        frame = cv2.cvtColor(np.array(frame.to_image(),dtype=np.uint8), cv2.COLOR_RGB2BGR)
        frame = cv2.resize(frame, (640,480))
        frame = tello.process_frame(frame)

        # frame의 오른쪽에 Hud 값을 출력할 화면의 바탕화면
        r_background = np.zeros((480,360,3), np.uint8)
        l_background = np.zeros((480,360,3), np.uint8)
        r_background[:]=(223,175,138)
        l_background[:]=(223,175,138)
        tello.write_Lhud(l_background)
        tello.write_Rhud(r_background)
        full_frame = cv2.hconcat([l_background,frame, r_background])

        tello.delivery_controls()
        tello.fps.update()

        full_frame = cv2.vconcat([logo,full_frame])
        #cv2.imshow('logo', logo)
        cv2.imshow('Dronah', full_frame)

        cv2.waitKey(1)

        frame_skip = int((time.time() - start_time)/time_base)

class TelloCV(object):
    """
    TelloTracker builds keyboard controls on top of TelloPy as well
    as generating images from the video stream and enabling opencv support
    """
    def __init__(self):
        self.prev_flight_data = None
        self.record = False
        self.tracking = False
        self.keydown = False
        self.date_fmt = '%Y-%m-%d_%H%M%S'
        self.speed = 50
        self.prev_tracking="tr_off"
        self.drone = tellopy.Tello()
        self.init_drone()
        self.init_controls()
        self.prev_tracking=0

	# container for processing the packets into frames
        self.container = av.open(self.drone.get_video_stream())
        self.vid_stream = self.container.streams.video[0]
        self.out_stream_writer=None
        self.out_file = None
        self.out_stream = None
        self.out_name = None
        self.start_time = time.time()
        self.use_openpose=False

        self.op = OP(number_people_max=5, min_size=10, debug=None)
        self.fps = FPS()

        #self.graph_pid1 = RollingGraph(window_name="graph", step_width=2, width=2000, height=300, y_max=200, colors=[(255,255,0)],thickness=[2],threshold=100, waitKey=False)

    def init_drone(self):
        """Connect, uneable streaming and subscribe to events"""
        # self.drone.log.set_level(2)
        self.drone.connect()
        self.set_video_encoder_rate(2)
        self.drone.start_video()
        self.drone.subscribe(self.drone.EVENT_FLIGHT_DATA,
                             self.flight_data_handler)
        self.drone.subscribe(self.drone.EVENT_LOG_DATA,
                             self.log_data_handler)
        self.drone.subscribe(self.drone.EVENT_FILE_RECEIVED,
                             self.handle_flight_received)

        
        # joystick version - tello2.py add
        self.axis_command = {
            "yaw": self.drone.clockwise,
            "roll": self.drone.right,
            "pitch": self.drone.forward,
            "throttle": self.drone.up
        }
        self.axis_speed = { "yaw":0, "roll":0, "pitch":0, "throttle":0}
        self.axis_zero = {"yaw":0, "roll":0, "pitch":0, "throttle":0}
        self.cmd_axis_speed = { "yaw":0, "roll":0, "pitch":0, "throttle":0}
        self.prev_axis_speed = self.axis_speed.copy()
        #self.def_speed =  { "yaw":40, "roll":40, "pitch":40, "throttle":40}
        self.def_speed =  { "yaw":40, "roll":40, "pitch":40, "throttle":40}
        self.is_flying=False
        self.is_tracking = False
        self.focus_ing = False #
        self._db_control=None
        self._db_start=None
        self._db_record=None
        self._db_value=None
        self._db_index=None
        self._db_tracking=None
        self.reset()

    def set_video_encoder_rate(self, rate):
        self.drone.set_video_encoder_rate(rate)
        self.video_encoder_rate = rate

    def reset (self):
        """
            Reset global variables before a fly
        """
        self.ref_pos_x = -1
        self.ref_pos_y = -1
        self.ref_pos_z = -1
        self.pos_x = -1
        self.pos_y = -1
        self.pos_z = -1
        self.ref_x=320
        self.ref_y=240
        self.angle_to_zero=0
        self.prev_yaw=0
        self.prev_yaw_time=0
        self.neck_to_midhip=0
        self.shoulders_width=0
        self.yaw=0
        self.yaw_to_consume = 0
        self.yaw_consumed=0
        self.roll=0
        self.roll_to_consume = 0
        self.roll_consumed=0
        self.turn_ing = False
        self.tracking = False
        self.keep_distance = None
        self.palm_landing = False
        self.palm_landing_approach = False
        self.timestamp_keep_distance = time.time()
        self.wait_before_tracking = None
        self.timestamp_take_picture = None
        self.pan_and_tilt_on = False
        #self.throw_ongoing = False
        #self.scheduled_takeoff = None
        # When in trackin mode, but no body is detected in current frame,
        # we make the drone rotate in the hope to find some body
        # The rotation is done in the same direction as the last rotation done
        self.body_in_prev_frame = False
        self.timestamp_no_body = time.time()
        self.last_rotation_is_cw = True

    def drone_flip(self,direction):
        context_change_flip = {'left':'flip_left','right':'flip_right','forward':'flip_forward','backward':'flip_back'}

        if direction in context_change_flip:
            flip_handler = context_change_flip[direction]
            getattr(self.drone,flip_handler)()
            self.db.child("DRONAH").update({"control": None})
        else :
            print("[Error] Impossible request + ")
            print(direction)
            return

    def drone_movement(self,db_txt,lensize):
        start_movement = {'takeoff','land'}
        context_signed= {'up':1,'down':-1,'right':1,'left':-1,'forward':1,'backward':-1,'clockwise':1,'counter_clockwise':-1}
        context_axis = {'stop':'stop','up':'throttle','down':'throttle','left':'roll','right':'roll','forward':'pitch','backward':'pitch','clockwise':'yaw','counter_clockwise':'yaw'}

        if db_txt in context_axis :
            if db_txt =='stop' :
                self.cmd_axis_speed=self.axis_zero.copy()
            else :
                axis_handler = context_axis[db_txt]   #axis_handler = throttle,roll,pitch,yaw 
                signed_handler = context_signed[db_txt] * 1/lensize #signed_handler = +1,-1 of axis
                self.cmd_axis_speed[axis_handler] = signed_handler * self.def_speed[axis_handler]
            for axis, command in self.axis_command.items():
                command(self.cmd_axis_speed[axis])
        elif db_txt in start_movement :
            getattr(self.drone,db_txt)()
        else :
            print("[Error] Unknown Request +")
            print(db_txt)
            return

    def drone_movement_with_tracking(self,db_txt):

        focus_movement = { 'left':'ref_x','right':'ref_x','up':'ref_y','down':'ref_y'}
        camera_movement = { 'stop':'stop','forward':'pitch','backward':'pitch'}
        context_signed= {'up':1,'down':-1,'right':1,'left':-1,'forward':1,'backward':-1}

        if db_txt =='reset' : 
            self.ref_x=320
            self.ref_y=240
        elif db_txt =='stop' : 
            if self.focus_ing == True :
                self.focus_ing = False
                self.tracking = False
                self.cmd_axis_speed=self.axis_zero.copy()
            if self.turn_ing == True :
                self.roll_to_consume = 0
                self.prev_yaw = 0
                self.axis_speed["roll"] = 0
                self.turn_ing = False
                self.angle_to_zero=0
        elif db_txt in camera_movement :
            axis_handler = camera_movement[db_txt]   #axis_handler = throttle,roll,pitch,yaw 
            signed_handler = context_signed[db_txt] #signed_handler = +1,-1 of axis
            self.cmd_axis_speed[axis_handler] = signed_handler * self.def_speed[axis_handler]
            self.keep_distance = self.shoulders_width
            for axis, command in self.axis_command.items():
                command(self.cmd_axis_speed[axis])
        elif db_txt in focus_movement :
            if not self.focus_ing :
                self.focus_ing = True
            else :
                signed_handler = context_signed[db_txt]
                if focus_movement[db_txt] == 'ref_x' :
                    self.ref_x = self.ref_x + signed_handler*10
                elif focus_movement[db_txt] == 'ref_y' :
                    self.ref_y = self.ref_y + signed_handler*10
                if not ((15 < self.ref_x < 625) and (15 < self.ref_y < 465 )) :
                    self.pan_and_tilt_on=False
                    if not (15 < self.ref_x < 625) : self.ref_x = self.ref_x - signed_handler*10
                    if not (15 < self.ref_y < 465) : self.ref_y = self.ref_y - signed_handler*10
                    self.db.child("DRONAH").update({"control":None})
        elif db_txt is not '' :
            print("[Error] Unknown Request +")
            print(db_txt)
            return

    def drone_turn_with_tracking(self,db_txt):

        self.turn_ing = True
        self.keep_distance = self.neck_to_midhip

        turn_degrees = {'forward':0, 'right':-90, 'backward':-180, 'left':90}
        turn_signed = {'clockwise':1, 'counter_clockwise':-1}

        if db_txt[0] in turn_signed :
            sign_handler = turn_signed[db_txt[0]]
            self.roll_to_consume = sign_handler * 360
        else :
            sign_handler = turn_signed[db_txt[1]]
            degree_handler = turn_degrees[db_txt[0]]
            self.roll_to_consume = sign_handler * degree_handler 

        self.prev_yaw_time = time.time()
        self.prev_yaw = self.yaw

    def on_press(self, keyname):
        """
            Handler for keyboard listener
        """
        if self.keydown:
            return
        try:
            self.keydown = True
            keyname = str(keyname).strip('\'')
            print(keyname)
            #log.info('KEY PRESS ' + keyname)
            if keyname == 'Key.esc':
                self.toggle_tracking(False)
                self.tracking = False
                self.drone.land()
                self.drone.quit()
                cv2.destroyAllWindows() 
                os._exit(0)
            if keyname in self.controls_keypress:
                self.controls_keypress[keyname]()
        except AttributeError:
            print(">>key error{}".format(keyname))
                    
    def on_release(self, keyname):
        """
            Reset on key up from keyboard listener
        """
        self.keydown = False
        keyname = str(keyname).strip('\'')
#        log.info('KEY RELEASE ' + keyname)
        if keyname in self.controls_release:
            key_handler = self.controls_release[keyname]()

    def toggle_distance(self):
        # Locked distance mode
        if self.keep_distance is None :#and turn is False:
            if  time.time() - self.timestamp_keep_distance > 2:
                # The first frame of a serie to activate the distance keeping
                self.keep_distance = self.shoulders_width
                self.timestamp_keep_distance = time.time()
                self.pid_pitch = PID(0.5,0.04,0.3,setpoint=0,output_limits=(-50,50))
                print("KEEP DISTANCE {}".format(self.keep_distance))
        else:
            if time.time() - self.timestamp_keep_distance > 2:
                print("KEEP DISTANCE FINISHED")
                self.keep_distance = None
                self.timestamp_keep_distance = time.time()

    def init_controls(self):

        self.controls_keypress = {
            'Key.tab': lambda : self.toggle_recording(),
            'Key.enter': lambda : self.take_picture(),
            'Key.backspace': lambda : self.reset()
        }
        
        self.controls_release = {
        }
        
        self.key_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.key_listener.start()

        self.config = {
            "apiKey" : "AIzaSyAxpSLxbMdJaYdrfwXe1XO6q_ECk0OdwMI",
            "authDomain" : "dronah-f7e0b",
            "databaseURL" : "https://dronah-f7e0b.firebaseio.com/",
            #"serviceAccount": "/Users/seny/senytime/hanium/main/dronah-f7e0b-firebase-adminsdk-z62f6-5e2c0b1c26.json",
            #"serviceAccount" : "C:/Users/SAMSUNG/main/dronah-f7e0b-firebase-adminsdk-z62f6-5e2c0b1c26.json",
            "serviceAccount" : "/home/seny/Dronah/dronah-f7e0b-firebase-adminsdk-z62f6-5e2c0b1c26.json",
	        "storageBucket": "gs://dronah-f7e0b.appspot.com"
        }

        firebase = pyrebase.initialize_app(self.config)
        self.db = firebase.database()

        def control_handler(message) :
            if message["event"] in ("put", "patch"):
                if message["path"] == "/":
                    self._db_control=message["data"]
                    if self._db_control != None :
                        for control in self._db_control :
                            if control == 'clockwise' or control == 'counter_clockwise':
                                if self.tracking :
                                    self.db.child("DRONAH").update({"control": None})

        def start_handler(message) :
            if message["event"] in ("put", "patch"):
                if message["path"] == "/":
                    self._db_start=message["data"]

        def index_handler(message):
            if message["event"] in ("put", "patch"):
                if message["path"] == "/":
                    self._db_index=message["data"]

        def tracking_handler(message):
            if message["event"] in ("put", "patch"):
                if message["path"] == "/":
                    self._db_tracking=message["data"]

        def value_handler(message):
            if message["event"] in ("put", "patch"):
                if message["path"] == "/":
                    self._db_value=message["data"]

        def record_handler(message):
            if message["event"] in ("put", "patch"):
                if message["path"] == "/":
                    self._db_record=message["data"]
                    self.db.child("DRONAH").update({"record": None})

        self.db.child("DRONAH").update({"value": 0})
        db_control= self.db.child("DRONAH").child("control").stream(control_handler, stream_id="switch_DBstatus1")
        db_start= self.db.child("DRONAH").child("start").stream(start_handler, stream_id="switch_DBstatus2")
        db_index=self.db.child("DRONAH").child("index").stream(index_handler, stream_id="switch_DBstatus3")
        db_tracking=self.db.child("DRONAH").child("tracking").stream(tracking_handler, stream_id="switch_DBstatus4")
        db_value=self.db.child("DRONAH").child("value").stream(value_handler, stream_id="switch_DBstatus5")
        db_record=self.db.child("DRONAH").child("record").stream(record_handler, stream_id="switch_DBstatus6")

    def delivery_controls(self):
        # *단계 take_picture, record 확인
        #print(self._db_record)
        if self._db_record == 'record_on':
            self.record = True
            self.toggle_recording()
        elif self._db_record == 'record_off':
            self.record = False
            self.toggle_recording()
        elif self._db_record == 'take_picture':
            self.take_picture(True)

        #0단계 : index 유무 확인해야함 (날지않아도 가능)
        if self._db_index == 'index_on':
            self.use_openpose=True
        elif self._db_index == 'index_off' :
            self.use_openpose=False
        #elif self._db_index is not None  or  self._db_index != None :
            #print("[Unknown Request1] {}".format(len(self._db_index)))

        # 1단계 : takeoff, land 확인
        start_movement = {'takeoff':True,'land':False}
        if self._db_start in start_movement : 
            if self.is_flying != start_movement[self._db_start]:
                getattr(self.drone,self._db_start)()
                self.is_flying=start_movement[self._db_start]
        #elif self._db_start is not None  or self._db_start != None :
            #print("[Unknown Request2] {}".format(len(self._db_start)))

        # 2단계 : tracking 유무 확인.
        if self.use_openpose and self._db_tracking == 'tr_on' :
             self.toggle_tracking(True)
             if self.keep_distance is None and self.shoulders_width :
                self.toggle_distance()
        elif self._db_tracking == 'tr_off' :
             self.toggle_tracking(False)
             self.keep_distance = None
        #elif self._db_tracking is not None or self._db_tracking !=None :
            #print("[Unknown Request3] {}".format(len(self._db_tracking)))

        #3단계 Control
        scene_movement= {'left':'right','right':'left','up':'up','down':'down'}
        if isinstance(self._db_control, list):
            # 3-1 : tracking mode일 경우
            if self.tracking == True and self._db_control != None :
                if ('pan' in self._db_control) or ('tilt' in self._db_control) :
                    scene_direction=self._db_control[1]
                    self.pan_and_tilt(scene_direction)
                    if self.pan_and_tilt_on == True :
                        self.drone_movement_with_tracking(scene_movement[scene_direction])
                elif ('clockwise' in self._db_control) or ('counter_clockwise' in self._db_control):
                    self.drone_turn_with_tracking(self._db_control)
                else :
                    for control in self._db_control :
                        self.drone_movement_with_tracking(control)
            # 3-2 : 일반 모드일 경우, db_control 안에 flip 있는지 체크 -> 있으면 flip 출력 없으면 합성명령어 출력
            else :
                if 'flip' in self._db_control:
                    self.drone_flip(self._db_control[0])
                else :
                    for control in self._db_control :
                        self.drone_movement(control,len(self._db_control))

    def process_frame(self, raw_frame):
        """convert frame to cv2 image and show"""
        frame = raw_frame.copy()
        h,w,_ = frame.shape
        turn_degree=0
        proximity = int(w/2.6)
        min_distance = int(w/2)
        self.axis_speed = self.cmd_axis_speed.copy() # we need to test 

        if self.turn_ing == True : 

            if  self.yaw * self.prev_yaw < 0 :
                if -2 <= self.yaw <= 2  :
                    if self.yaw > 0 :                                 #COUNTER_CLOCKWISE
                        self.angle_to_zero=self.angle_to_zero+abs(2-self.prev_yaw)
                        self.prev_yaw=2
                    else :                                            #CLOCKWISE
                        self.angle_to_zero=self.angle_to_zero+abs(2+self.prev_yaw)
                        self.prev_yaw=-2

                if (178 <= self.yaw < 180) or (-180 <self.yaw <=-178) :
                    if self.yaw < 0 :                                 #CLOCKWISE
                        self.angle_to_zero=self.angle_to_zero+abs(182+self.prev_yaw)
                        self.prev_yaw=-178
                    else :                                            #COUNTER_CLOCKWISE
                        self.angle_to_zero=self.angle_to_zero+abs(182-self.prev_yaw)
                        self.prev_yaw=178

            if self.angle_to_zero > 0 :
                turn_degree = abs(self.yaw - self.prev_yaw) + self.angle_to_zero
                cv2.putText(frame, "[box] {}".format(self.angle_to_zero), (200, 270), cv2.FONT_HERSHEY_DUPLEX,1.0, (0, 0, 255), lineType=30)
            else :
                turn_degree = abs(self.yaw - self.prev_yaw)

            cv2.putText(frame, "[Turn] {}".format(turn_degree) , (200, 240), cv2.FONT_HERSHEY_DUPLEX,1.0, (0, 0, 255), lineType=30)
            cv2.putText(frame, "[prev_yaw] {}".format(self.prev_yaw) , (200, 300), cv2.FONT_HERSHEY_DUPLEX,1.0, (0, 0, 255), lineType=30)

            if abs(turn_degree) > abs(self.roll_to_consume) and (time.time() - self.prev_yaw_time > 2): #or (turn_degree == 0))
                self.roll_to_consume = 0
                self.prev_yaw = 0
                self.axis_speed["roll"] = 0
                self.turn_ing = False
                self.angle_to_zero=0
                if float(self.neck_to_midhip)/float(self.shoulders_width) < 2: # left side / right side  TO front / back side 
                    self.keep_distance = 1/2* self.keep_distance
                print("Finish -- Turn -- ")
            elif self.roll_to_consume > 0:
                self.axis_speed["roll"] = 2/3 * -self.def_speed["roll"]
            elif self.roll_to_consume < 0:
                self.axis_speed["roll"] = 2/3 * self.def_speed["roll"]

        if self.use_openpose:
            nb_people, pose_kps, face_kps = self.op.eval(frame)
            target = None
            # Our target is the person whose index is 0 in pose_kps
            self.pose = None

            if nb_people > 0 : 
                if nb_people < self._db_value :
                     self._db_value=self._db_value-1
                # We found a body, so we can cancel the exploring 360
                #self.yaw_to_consume = 0
                # Draw the skeleton on the frame
                self.op.draw_body(frame)
                #################

                ############
                for nb_person in range(nb_people) :
                    cv2.putText(frame, "{}".format(nb_person+1), (int(pose_kps[nb_person][0][0]), int(pose_kps[nb_person][0][1])-10), cv2.FONT_HERSHEY_DUPLEX, 1.5, (0, 212, 255), 3)
                # In tracking mode, we track a specific body part (an openpose keypoint):
                # the nose if visible, otherwise the neck, otherwise the midhip
                # The tracker tries to align that body part with the reference point (ref_x, ref_y)
                # 여기서 target을 탐지하는데, 코가 안보이면 목, 목이 안보이면 MidHip으로 탐지하고 따라함


                if self.tracking : #and nb_people == 1 :
                    target = self.op.get_body_kp("Neck",self._db_value-1)
                    if target is None:
                        target = self.op.get_body_kp("Nose",self._db_value-1)
                        if target is None:
                            target = self.op.get_body_kp("MidHip",self._db_value-1)
                #elif self.tracking and nb_people > 1 :
                    #target=(int(pose_kps[self._db_value-1][1][0]),int(pose_kps[self._db_value-1][1][1]))
                    #if target is None:
                        #target = (int(pose_kps[self._db_value-1][0][0]),int(pose_kps[self._db_value-1][0][1]))
                        #if target is None:
                            #target = (int(pose_kps[self._db_value-1][8][0]),int(pose_kps[self._db_value-1][8][1]))

                if target:
                    r_shoulder =self.op.get_body_kp("RShoulder",self._db_value-1)
                    l_shoulder =self.op.get_body_kp("LShoulder",self._db_value-1)
                    nose = self.op.get_body_kp("Nose",self._db_value-1)              
                    neck = self.op.get_body_kp("Neck",self._db_value-1)
                    mid_hip = self.op.get_body_kp("MidHip",self._db_value-1) 

                    if r_shoulder and l_shoulder :
                         self.shoulders_width = distance(r_shoulder,l_shoulder)
                    elif neck and r_shoulder :
                         self.shoulders_width = 2*distance(r_shoulder,neck)
                    else :
                         self.shoulders_width = 2*distance(neck,l_shoulder)

                    #if self.shoulders_width < 5 : #left/right side of person
                         #self.shoulders_width = None

                    if neck and mid_hip :
                         self.neck_to_midhip = distance(neck,mid_hip)
                    else :
                         self.neck_to_midhip = 3 * distance(nose,neck)

                    self.tracking=True #for stop during tracking 
                    self.prev_target = target ##
                    self.body_in_prev_frame = True          # 여기가 사람을 중앙에 인식하도록 하는 것
                    # We draw an arrow from the reference point to the body part we are targeting       
                    xoff = int(target[0]-self.ref_x)
                    yoff = int(self.ref_y-target[1])
                    cv2.circle(frame, (self.ref_x, self.ref_y), 15, (250,150,0), 1,cv2.LINE_AA) # 사람의 중간지점? 무게중심을 원으로 표시하고 그에 따른 움직인 표현   
                    cv2.arrowedLine(frame, (self.ref_x, self.ref_y), target, (250, 150, 0), 6)    # 이전 위치로부터 사람이 움직인 경로 표현
                    cv2.putText(frame, "{}".format(self._db_value), (int(target[0]), int(target[1])-10), cv2.FONT_HERSHEY_DUPLEX, 1.5, (0, 0, 0), 3)

                    # The PID controllers calculate the new speeds for yaw and throttle
                    self.axis_speed["yaw"] = int(-self.pid_yaw(xoff))           # pid controller 사용해서 속도 조절하고 사람 따라다니게 함
                    self.last_rotation_is_cw = self.axis_speed["yaw"] > 0

                    self.axis_speed["throttle"] = int(-self.pid_throttle(yoff)) 

                    ctr_err=abs(xoff*yoff)
                    #self.graph_pid1.new_iter([sqrt(ctr_err)])##

                    # If in turn_ing mode
                    if self.turn_ing :
                        if self.keep_distance and self.neck_to_midhip: #self.shoulders_width:                 # 일정 거리 유지하면서 tracking
                            self.axis_speed["pitch"] = int(self.pid_pitch(self.neck_to_midhip-self.keep_distance))
                    else :
                        if float(self.neck_to_midhip)/float(self.shoulders_width) > 2 and self.keep_distance : # left side / right side of person
                            self.axis_speed["pitch"] = int(self.pid_pitch(self.neck_to_midhip-self.keep_distance))
                        elif self.shoulders_width and self.keep_distance:
                            self.axis_speed["pitch"] = int(self.pid_pitch(self.shoulders_width-self.keep_distance))

                    if self.shoulders_width :
                        if ( self.shoulders_width>75 and self.cmd_axis_speed["pitch"] > 0 ) or self.tracking is False :
                            self.focus_ing = False
                            self.tracking = False
                            self.keep_distance = None
                            self.axis_speed=self.axis_zero.copy()
                            self.cmd_axis_speed=self.axis_zero.copy()
                            self.db.child("DRONAH").update({"control": None})
                            cv2.putText(frame, "SAFE MODE", (200, 240),cv2.FONT_HERSHEY_DUPLEX, 1.5, (0, 0, 255), lineType=30)
                else: # Tracking but no body detected       -> tracking 모드 인데, 아무런 body 인식 못하면, 내부 시간 1초 잡고 회전하면서 몸 찾음
                    if self.body_in_prev_frame:
                        self.timestamp_no_body = time.time()
                        self.body_in_prev_frame = False
                        self.axis_speed["throttle"] = self.prev_axis_speed["throttle"]
                        self.axis_speed["yaw"] = self.prev_axis_speed["yaw"]
                    else:
                        if time.time() - self.timestamp_no_body < 3:
                            #print("NO BODY SINCE < 1", self.axis_speed, self.prev_axis_speed)
                            self.axis_speed["throttle"] = self.prev_axis_speed["throttle"]
                            self.axis_speed["yaw"] = self.prev_axis_speed["yaw"]
                        #else:
                            #self.axis_speed["yaw"] = self.def_speed["yaw"] * (1 if self.last_rotation_is_cw else -1)
                #for person_idx in range(nb_people) :
                #    cv2.putText(r_background, pose_kps[person_idx][:3].tostring(), (360, person_idx*60),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 205, 0), lineType=15)


        # Send axis commands to the drone
        for axis, command in self.axis_command.items():
            if self.axis_speed[axis]is not None and self.axis_speed[axis] != self.prev_axis_speed[axis]:
                command(self.axis_speed[axis])
                self.prev_axis_speed[axis] = self.axis_speed[axis]
            else:
                # This line is necessary to display current values in 'self.write_hud'
                self.axis_speed[axis] = self.prev_axis_speed[axis]

        for i in range(1,3) :
            frame = cv2.line(frame, (0, int(480*i/3)), (640, int(480*i/3)), (0,0,0))
            frame = cv2.line(frame, (int(640*i/3), 0), (int(640*i/3), 480), (0,0,0))

        if self.record:
            diff = int(time.time() - self.start_time)
            mins, secs = divmod(diff, 60)
            cv2.putText(frame, "@REC {:02d}:{:02d}".format(mins, secs), (320, 50),cv2.FONT_HERSHEY_DUPLEX, 1.5, (0, 0, 255), lineType=30)

        return frame

    def write_Rhud(self, r_background):
        """Draw drone info, tracking and record on frame"""
        stats = self.prev_flight_data.split('|')
        stats.pop()
        stats.pop()
        stats.append(f"FPS : {self.fps.get():.2f}")
        stats.append("Use_Openpose : " + str(self.use_openpose))
        stats.append("Tracking Number : "+str(self._db_value))
        stats.append("Shoulders_width : "+str(self.shoulders_width))
        stats.append("neck_to_midhip : "+str(self.neck_to_midhip))
        stats.append("Keep Distance : " + str(self.keep_distance))
        stats.append("Focusing : " + str(self.focus_ing))
        #stats.append("중심좌표 : ({},{})".format(str(self.ref_x),str(self.ref_y)))
        stats.append("Yaw : " + str(self.yaw))
        stats.append("Roll : " + str(self.roll))
        stats.append("Pitch : " + str(self.pitch))
        stats.append("Pitch speed : " + str(self.axis_speed["pitch"]))

        #stats.append("self.prev_yaw: " + str(self.prev_yaw))

        for idx, stat in enumerate(stats):
            text = stat.lstrip()
            cv2.putText(r_background, text, (0, 30 + (idx * 30)),cv2.FONT_HERSHEY_DUPLEX,1.0, (0, 0, 0), lineType=30)

        return r_background

    def write_Lhud(self, l_background):
        cv2.putText(l_background, " State : {}".format(self._db_start), (0, 30), cv2.FONT_HERSHEY_DUPLEX,1.0, (0, 0, 0), lineType=30)
        cv2.putText(l_background, " Index Mode : {}".format(self._db_index), (0, 60), cv2.FONT_HERSHEY_DUPLEX,1.0, (0, 0, 0), lineType=30)
        cv2.putText(l_background, " Track Mode : {}".format(self._db_tracking), (0, 90), cv2.FONT_HERSHEY_DUPLEX,1.0, (0, 0, 0), lineType=30)
        cv2.putText(l_background, " Turn Mode : {}".format(self.turn_ing), (0, 120), cv2.FONT_HERSHEY_DUPLEX,1.0, (0, 0, 0), lineType=30)
        cv2.putText(l_background, " Turn Degree : {}".format(self.roll_to_consume), (0, 150), cv2.FONT_HERSHEY_DUPLEX,1.0, (0, 0, 0), lineType=30)
        cv2.putText(l_background, "     [Movement] ", (0, 210), cv2.FONT_HERSHEY_DUPLEX,1.0, (0, 0, 0), lineType=30)

        if self._db_control : 
            for idx, mvment in enumerate(self._db_control):
                cv2.putText(l_background, " -{}".format(mvment), (0, 250 + (idx * 30)),cv2.FONT_HERSHEY_DUPLEX,1.0, (0, 0, 0), lineType=30)

        #cv2.putText(l_background, "neck_to_midhip {}".format(self.neck_to_midhip), (0, 210), cv2.FONT_HERSHEY_DUPLEX,1.0, (0, 0, 0), lineType=30)

        return l_background

    def toggle_recording(self):
        """Handle recording keypress, creates output stream and file"""
        '''if speed == 0:
            return
        '''
        if self.record:
            datename = [os.getenv('HOME'), datetime.datetime.now().strftime(self.date_fmt)]
            self.out_name = '{}/Pictures/dronah-{}.mp4'.format(*datename)
            print("Outputting video to:", self.out_name)
            self.out_stream_writer = cv2.VideoWriter(self.out_name,cv2.VideoWriter_fourcc(*'mp4v'),25,(self.vid_stream.width,self.vid_stream.height))
            self.start_time = time.time()
            self._db_record = None

        if not self.record:
            print("Video saved to ", self.out_name)
            self.out_stream_writer.release()
            self.out_stream_writer = None
            self._db_record = None
        
    def record_vid(self, frame):
        """
        convert frames to packets and write to file
        """
        if not self.out_stream_writer is None:
            arr = cv2.cvtColor(np.array(frame.to_image(),dtype=np.uint8), cv2.COLOR_RGB2BGR)
            self.out_stream_writer.write(arr)

    def take_picture(self,picture = False):
        """Tell drone to take picture, image sent to file handler"""
        '''if speed == 0:
            return
        '''
        if picture == False:
            return
        self.drone.take_picture()

    def toggle_tracking(self, tracking=False):
        if tracking is True:
            self.tracking = True
        else:
            self.tracking = False
        if self.tracking:
            self.use_openpose = True
            # Init a PID controller for the yaw
            self.pid_yaw = PID(0.25,0,0,setpoint=0,output_limits=(-100,100))
            # ... and one for the throttle
            self.pid_throttle = PID(0.4,0,0,setpoint=0,output_limits=(-80,100))
            # self.init_tracking = True
        
        else:
            self.axis_speed = { "yaw":0, "roll":0, "pitch":0, "throttle":0}
            self.keep_distance = False
        return

    def flight_data_handler(self, event, sender, data):
        """Listener to flight data from the drone."""
        text = str(data)
        if self.prev_flight_data != text:
            self.prev_flight_data = text

    def handle_flight_received(self, event, sender, data):
        """Create a file in ~/Pictures/ to receive image from the drone"""
        path = '%s/Pictures/dronah-%s.jpeg' % (
            os.getenv('HOME'),
            datetime.datetime.now().strftime(self.date_fmt))
        with open(path, 'wb') as out_file:
            out_file.write(data)
        print('Saved photo to %s' % path)

    def log_data_handler(self, event, sender, data):
        """
            Listener to log data from the drone.
        """  
        pos_x = -data.mvo.pos_x
        pos_y = -data.mvo.pos_y
        pos_z = -data.mvo.pos_z
        if abs(pos_x)+abs(pos_y)+abs(pos_z) > 0.07:
            if self.ref_pos_x == -1: # First time we have meaningful values, we store them as reference
                self.ref_pos_x = pos_x
                self.ref_pos_y = pos_y
                self.ref_pos_z = pos_z
            else:
                self.pos_x = pos_x - self.ref_pos_x
                self.pos_y = pos_y - self.ref_pos_y
                self.pos_z = pos_z - self.ref_pos_z
        
        qx = data.imu.q1
        qy = data.imu.q2
        qz = data.imu.q3
        qw = data.imu.q0
        self.yaw = quat_to_yaw_deg(qx,qy,qz,qw)
        self.roll = quat_to_roll_deg(qx,qy,qz,qw)
        self.pitch = quat_to_pitch_deg(qx,qy,qz,qw)

    def palm_land(self, speed):
        """Tell drone to land"""
        if speed == 0:
            return
        self.drone.palm_land()

    # 촬영기법 함수
    def pan_and_tilt(self, db_txt): # pan이랑 tilt 합침 "left"
        context_signed= {'up':-1,'down':1,'right':1,'left':-1}
        pan_speed=10

        signed_handler = context_signed[db_txt]

        if db_txt == 'right' or db_txt == 'left': #pan
            if self.pan_and_tilt_on == False:
                self.ref_x = self.ref_x + signed_handler*pan_speed 
            if not ( 20 < self.ref_x < 620) :
                self.pan_and_tilt_on = True
                #self.drone_movement_with_tracking(db_txt)

        if db_txt == 'up' or db_txt == 'down': #tilt
            if self.pan_and_tilt_on == False:
                self.ref_y = self.ref_y + signed_handler*pan_speed
            if not ( 20 < self.ref_y < 460 ) :
                self.drone_movement_with_tracking(db_txt)
                self.pan_and_tilt_on = True
        elif db_txt is not '' :
            print("[Error] Unknown Request +")
            print(db_txt)
            return


if __name__ == '__main__':
    main()

