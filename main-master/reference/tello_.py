import time
import datetime
import os
import tellopy
import numpy as np
import av
import cv2
import argparse

from math import pi, atan2
from OP import *
from math import atan2, degrees, sqrt
from simple_pid import PID
from  multiprocessing import Process, Pipe, sharedctypes
from FPS import FPS
import logging
import re
import sys

import pyrebase
from easytello import tello


log = logging.getLogger("TellOpenpose")
	    
def distance (A, B):
    return int(sqrt((B[0]-A[0])**2 + (B[1]-A[1])**2))

def angle (A, B, C):
    if A is None or B is None or C is None:
        return None
    return degrees(atan2(C[1]-B[1],C[0]-B[0]) - atan2(A[1]-B[1],A[0]-B[0]))%360

def vertical_angle (A, B):
    if A is None or B is None:
        return None
    return degrees(atan2(B[1]-A[1],B[0]-A[0]) - pi/2)

def quat_to_yaw_deg(qx,qy,qz,qw):
    degree = pi/180
    sqy = qy*qy
    sqz = qz*qz
    siny = 2 * (qw*qz+qx*qy)
    cosy = 1 - 2*(qy*qy+qz*qz)
    yaw = int(atan2(siny,cosy)/degree)
    return yaw

def openpose_worker():
    print("Worker process",os.getpid())
    tello.drone.start_recv_thread()
    tello.init_db()
    tello.op = OP(number_people_max=1, min_size=25, debug=tello.debug) #예림이 수정할 부분

    while True:
        tello.fps.update()
	
        frame = np.ctypeslib.as_array(tello.shared_array).copy()
        frame.shape=tello.frame_shape
        
        frame = tello.process_frame(frame)

        cv2.imshow("Processed", frame)


def main(use_multiprocessing=False, log_level=None):

    global tello

    if use_multiprocessing:
        # Create the pipe for the communication between the 2 processes
        parent_cnx, child_cnx = Pipe()
    else:
        child_cnx = None

    tello = TelloController(use_face_tracking=True, 
			    write_log_data=False, 
                            log_level=log_level, child_cnx=child_cnx)
   
    first_frame = True  
    frame_skip = 300

    for frame in tello.container.decode(video=0):
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
        if use_multiprocessing:
            
            if first_frame:
                
                frame_as_ctypes = np.ctypeslib.as_ctypes(frame)
                tello.shared_array = sharedctypes.RawArray(frame_as_ctypes._type_, frame_as_ctypes)
                tello.frame_shape = frame.shape
                first_frame = False
                # Launch process child
                p_worker = Process(target=openpose_worker)
                p_worker.start()
            tello.shared_array[:] = np.ctypeslib.as_ctypes(frame.copy())
            if parent_cnx.poll():
                msg = parent_cnx.recv()
                if msg == "EXIT":
                    print("MAIN EXIT")
                    p_worker.join()
                    tello.drone.quit()
                    cv2.destroyAllWindows()
                    exit(0)
        else:
            frame = tello.process_frame(frame)

        if not use_multiprocessing: tello.fps.update()

        cv2.imshow('Tello', frame)

        frame_skip = int((time.time() - start_time)/time_base)

class TelloController(object):

    def __init__(self, use_face_tracking=True, 
                media_directory="media", 
		write_log_data=False, 
                child_cnx=None,
                log_level=None):
        
        self.log_level = log_level
        self.debug = log_level is not None
        self.child_cnx = child_cnx
        self.use_multiprocessing = child_cnx is not None
        self.kbd_layout = kbd_layout
        # Flight data
        self.is_flying = False
        self.battery = None
        self.fly_mode = None
        self.throw_fly_timer = 0

        self.tracking_after_takeoff = False
        self.record = False
        self.date_fmt = '%Y-%m-%d_%H%M%S'
        self.drone = tellopy.Tello(start_recv_thread=not self.use_multiprocessing)
	#self.d=stream_handler(message)

        self.axis_command = {
            "yaw": self.drone.clockwise,
            "roll": self.drone.right,
            "pitch": self.drone.forward,
            "throttle": self.drone.up
        }
        self.axis_speed = { "yaw":0, "roll":0, "pitch":0, "throttle":0}
        self.cmd_axis_speed = { "yaw":0, "roll":0, "pitch":0, "throttle":0}
        self.prev_axis_speed = self.axis_speed.copy()
        self.def_speed =  { "yaw":50, "roll":35, "pitch":35, "throttle":80}             
        self.write_log_data = write_log_data
        self.reset()
        self.media_directory = media_directory
        if not os.path.isdir(self.media_directory):
            os.makedirs(self.media_directory)

        if self.write_log_data:
            path = 'tello-%s.csv' % datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S')
            self.log_file = open(path, 'w')
            self.write_header = True

        self.init_drone()
        if not self.use_multiprocessing:
            self.init_db()
        

        # container for processing the packets into frames
        self.container = av.open(self.drone.get_video_stream())
        self.vid_stream = self.container.streams.video[0]
        self.out_file = None
        self.out_stream = None
        self.out_name = None
        self.start_time = time.time()

                
        # Setup Openpose
        if not self.use_multiprocessing:
            
            self.op = OP(number_people_max=1, min_size=25, debug=self.debug)
        self.use_openpose = False
       
        self.fps = FPS()

        self.exposure = 0

        if self.debug:
            self.graph_pid = RollingGraph(window_name="PID", step_width=2, width=2000, height=500, y_max=200, colors=[(255,255,255),(255,200,0),(0,0,255),(0,255,0)],thickness=[2,2,2,2],threshold=100, waitKey=False)
                                   

        # Logging
        self.log_level = log_level
        if log_level is not None:
            if log_level == "info":
                log_level = logging.INFO
            elif log_level == "debug":
                log_level = logging.DEBUG
            log.setLevel(log_level)
            ch = logging.StreamHandler(sys.stdout)
            ch.setLevel(log_level)
            ch.setFormatter(logging.Formatter(fmt='%(asctime)s.%(msecs)03d - %(name)s - %(levelname)s - %(message)s',
                            datefmt="%H:%M:%S"))
            log.addHandler(ch)

    def set_video_encoder_rate(self, rate):
        self.drone.set_video_encoder_rate(rate)
        self.video_encoder_rate = rate

    def reset (self):
        log.debug("RESET")
        self.ref_pos_x = -1
        self.ref_pos_y = -1
        self.ref_pos_z = -1
        self.pos_x = -1
        self.pos_y = -1
        self.pos_z = -1
        self.yaw = 0
        self.tracking = False
        self.keep_distance = None
        self.palm_landing = False
        self.palm_landing_approach = False
        self.yaw_to_consume = 0
        self.timestamp_keep_distance = time.time()
        self.wait_before_tracking = None
        self.timestamp_take_picture = None
        self.throw_ongoing = False
        self.scheduled_takeoff = None
        self.body_in_prev_frame = False
        self.timestamp_no_body = time.time()
        self.last_rotation_is_cw = True


    def init_drone(self):
        if self.log_level :
            self.drone.log.set_level(2)
        self.drone.connect()
        self.set_video_encoder_rate(2)
        self.drone.start_video()

        self.drone.subscribe(self.drone.EVENT_FLIGHT_DATA,
                             self.flight_data_handler)
        self.drone.subscribe(self.drone.EVENT_LOG_DATA,
                             self.log_data_handler)
        self.drone.subscribe(self.drone.EVENT_FILE_RECEIVED,
                             self.handle_flight_received)

    def init_db(self):

	class DBHandler:
            _db = pyrebase.initialize_app({
                "apiKey": "YOUR-apiKey",
                "authDomain": "YOUR-authDomain",
                "databaseURL": "YOUR-databaseURL",
                "storageBucket": "YOUR-storageBucket",
                "serviceAccount": "YOUR-serviceAccount",
            }).database()

	    def __init__(self) -> None:
                super().__init__()
                self._db.child("my_stuff").stream(self.stream_handler)
	    
            def stream_handler(self, message):
                if message["event"] in ("put", "patch"):
                    if message["path"] == "/": #first data(init)
			db=message["data"]
                    else:			#updata data
			print('event={m[event]}; path={m[path]}; data={m[data]}'.format(m=message)) #수정필요
	    #event=patch; path=/Device_1;  data={'0': 'right', '1': 'flip'} 딕셔너리 형태로 담긴다고 가정하였음

	    """
            def is_ready(self) -> bool:
                return self.my_stuff is not None
	    """

        self.db=DBHandler()
	"""
	while not db.is_ready:
            pass	
	"""

	def context_change(db_txt):
	    return{
            'takeoff' : lambda: self.drone.takeoff()
            'land' : lambda: self.drone.land()
            'up' : lambda: self.set_speed("throttle", self.def_speed["throttle"]),
            'down' : lambda: self.set_speed("throttle", -self.def_speed["throttle"]),
            'left' : lambda: self.set_speed("roll", -self.def_speed["roll"]),
            'right' : lambda: self.set_speed("roll", self.def_speed["roll"]),
            'forward' : lambda: self.set_speed("pitch", self.def_speed["pitch"]),
            'back' : lambda: self.set_speed("pitch", -self.def_speed["pitch"]),
            'cw' : lambda: self.set_speed("yaw", self.def_speed["yaw"]),
            'ccw' : lambda: self.set_speed("yaw", -self.def_speed["yaw"])
	    }.get(db_txt,-1)

	def context_change_flip():
	    for axis, command in self.cmd_axis_command.items():
	        if self.cmd_axis_speed[axis] > 0 :
		    where = {
		    	'roll': lambda: self.drone.flip_right(),
			'pitch':lambda: self.drone.flip_forward()
		    }.get(axis,-1)
		elif self.cmd_axis_speed[axis] < 0 :
		    where = {
			'roll': lambda: self.drone.flip_left(),
			'pitch':lambda: self.drone.flip_back()
		    }.get(axis,-1)

	    return where

	for db_key , db_txt in self.db.items() :
	    if db_txt != 'flip':
		context_change(db_txt)
	    else:
		if not context_change_flip() :
		    for axis, speed in self.cmd_axis_command:
			self.set_speed(axis,0) #불가능한명령일경우 0으로 초기화
		    

    def set_speed(self, axis, speed):
        log.info(f"set speed {axis} {speed}")
        self.cmd_axis_speed[axis] = speed

    def process_frame(self, raw_frame):

        frame = raw_frame.copy()
        h,w,_ = frame.shape
        proximity = int(w/2.6)
        min_distance = int(w/2)
        
        if self.scheduled_takeoff and time.time() > self.scheduled_takeoff:
            
            self.scheduled_takeoff = None
            self.drone.takeoff()

        self.axis_speed = self.cmd_axis_speed.copy()

        for axis, command in self.axis_command.items():
            if self.axis_speed[axis]is not None and self.axis_speed[axis] != self.prev_axis_speed[axis]:
                log.debug(f"COMMAND {axis} : {self.axis_speed[axis]}")
                command(self.axis_speed[axis])
                self.prev_axis_speed[axis] = self.axis_speed[axis]
            else:
                self.axis_speed[axis] = self.prev_axis_speed[axis]
        
        frame = self.write_hud(frame)

        
        return frame

    def write_hud(self, frame):

        class HUD:
            def __init__(self, def_color=(255, 170, 0)):
                self.def_color = def_color
                self.infos = []
            def add(self, info, color=None):
                if color is None: color = self.def_color
                self.infos.append((info, color))
            def draw(self, frame):
                i=0
                for (info, color) in self.infos:
                    cv2.putText(frame, info, (0, 30 + (i * 30)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, color, 2) #lineType=30)
                    i+=1
                

        hud = HUD()

        if self.debug: hud.add(datetime.datetime.now().strftime('%H:%M:%S'))
        hud.add(f"FPS {self.fps.get():.2f}")
        if self.debug: hud.add(f"VR {self.video_encoder_rate}")

        hud.add(f"BAT {self.battery}")
        if self.is_flying:
            hud.add("FLYING", (0,255,0))
        else:
            hud.add("NOT FLYING", (0,0,255))
        hud.add(f"TRACKING {'ON' if self.tracking else 'OFF'}", (0,255,0) if self.tracking else (0,0,255) )
        hud.add(f"EXPO {self.exposure}")
        
        if self.axis_speed['yaw'] > 0:
            hud.add(f"CW {self.axis_speed['yaw']}", (0,255,0))
        elif self.axis_speed['yaw'] < 0:
            hud.add(f"CCW {-self.axis_speed['yaw']}", (0,0,255))
        else:
            hud.add(f"CW 0")
        if self.axis_speed['roll'] > 0:
            hud.add(f"RIGHT {self.axis_speed['roll']}", (0,255,0))
        elif self.axis_speed['roll'] < 0:
            hud.add(f"LEFT {-self.axis_speed['roll']}", (0,0,255))
        else:
            hud.add(f"RIGHT 0")
        if self.axis_speed['pitch'] > 0:
            hud.add(f"FORWARD {self.axis_speed['pitch']}", (0,255,0))
        elif self.axis_speed['pitch'] < 0:
            hud.add(f"BACKWARD {-self.axis_speed['pitch']}", (0,0,255))
        else:
            hud.add(f"FORWARD 0")
        if self.axis_speed['throttle'] > 0:
            hud.add(f"UP {self.axis_speed['throttle']}", (0,255,0))
        elif self.axis_speed['throttle'] < 0:
            hud.add(f"DOWN {-self.axis_speed['throttle']}", (0,0,255))
        else:
            hud.add(f"UP 0")

        if self.use_openpose: hud.add(f"POSE: {self.pose}", (0,255,0) if self.pose else (255, 170, 0))
        if self.keep_distance: 
            hud.add(f"Target distance: {self.keep_distance} - curr: {self.shoulders_width}", (0,255,0))
            #if self.shoulders_width: self.graph_distance.new_iter([self.shoulders_width])
        if self.timestamp_take_picture: hud.add("Taking a picture", (0,255,0))
        if self.palm_landing:
            hud.add("Palm landing...", (0,255,0))
        if self.palm_landing_approach:
            hud.add("In approach for palm landing...", (0,255,0))
        if self.tracking and not self.body_in_prev_frame and time.time() - self.timestamp_no_body > 0.5:
            hud.add("Searching...", (0,255,0))
        if self.throw_ongoing:
            hud.add("Throw ongoing...", (0,255,0))
        if self.scheduled_takeoff:
            seconds_left = int(self.scheduled_takeoff - time.time())
            hud.add(f"Takeoff in {seconds_left}s")

        hud.draw(frame)
        return frame

    def take_picture(self):
        self.drone.take_picture()

    def set_exposure(self, expo):
        if expo == 0:
            self.exposure = 0
        elif expo == 1:
            self.exposure = min(9, self.exposure+1)
        elif expo == -1:
            self.exposure = max(-9, self.exposure-1)
        self.drone.set_exposure(self.exposure)
        log.info(f"EXPOSURE {self.exposure}")

    def palm_land(self):
        self.palm_landing = True
        self.sound_player.play("palm landing")
        self.drone.palm_land()

    def throw_and_go(self, tracking=False):
        self.drone.throw_and_go()      
        self.tracking_after_takeoff = tracking
        
    def delayed_takeoff(self, delay=5):
        self.scheduled_takeoff = time.time()+delay
        self.tracking_after_takeoff = True
        
    def clockwise_degrees(self, degrees):
        self.yaw_to_consume = degrees
        self.yaw_consumed = 0
        self.prev_yaw = self.yaw
        
    def toggle_openpose(self):
        self.use_openpose = not self.use_openpose
        if not self.use_openpose:
            self.toggle_tracking(tracking=False)
        log.info('OPENPOSE '+("ON" if self.use_openpose else "OFF"))

    def toggle_tracking(self, tracking=None):
        if tracking is None:
            self.tracking = not self.tracking
        else:
            self.tracking = tracking
        if self.tracking:
            log.info("ACTIVATE TRACKING")
            self.use_openpose = True
            self.pid_yaw = PID(0.25,0,0,setpoint=0,output_limits=(-100,100))
            self.pid_throttle = PID(0.4,0,0,setpoint=0,output_limits=(-80,100))
        else:
            self.axis_speed = { "yaw":0, "roll":0, "pitch":0, "throttle":0}
            self.keep_distance = None
        return

    def flight_data_handler(self, event, sender, data):
        self.battery = data.battery_percentage
        self.fly_mode = data.fly_mode
        self.throw_fly_timer = data.throw_fly_timer
        self.throw_ongoing = data.throw_fly_timer > 0

        if self.is_flying != data.em_sky:            
            self.is_flying = data.em_sky
            log.debug(f"FLYING : {self.is_flying}")
            if not self.is_flying:
                self.reset()
            else:
                if self.tracking_after_takeoff:
                    log.info("Tracking on after takeoff")
                    self.toggle_tracking(True)
                    
        log.debug(f"MODE: {self.fly_mode} - Throw fly timer: {self.throw_fly_timer}")

    def log_data_handler(self, event, sender, data):
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
        
        if self.write_log_data:
            if self.write_header:
                self.log_file.write('%s\n' % data.format_cvs_header())
                self.write_header = False
            self.log_file.write('%s\n' % data.format_cvs())

    def handle_flight_received(self, event, sender, data):
        path = f'{self.media_directory}/tello-{datetime.datetime.now().strftime(self.date_fmt)}.jpg' 
        with open(path, 'wb') as out_file:
            out_file.write(data)
        log.info('Saved photo to %s' % path)

    


if __name__ == '__main__':
    ap=argparse.ArgumentParser()
    ap.add_argument("-l","--log_level", help="select a log level (info, debug)")
    ap.add_argument("-2","--multiprocess", action='store_true', help="use 2 processes to share the workload (instead of 1)")
    args=ap.parse_args()

