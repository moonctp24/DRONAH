#TelloCV


## tracker.py 중 class Tracker부분만 먼저 보기

```python 

class Tracker:
    """
    A basic color tracker, it will look for colors in a range and
    create an x and y offset valuefrom the midpoint
    """

    def __init__(self, height, width, color_lower, color_upper):
        self.color_lower = color_lower
        self.color_upper = color_upper
        self.midx = int(width / 2)
        self.midy = int(height / 2)
        self.xoffset = 0
        self.yoffset = 0

    def draw_arrows(self, frame):
        """Show the direction vector output in the cv2 window"""
        #cv2.putText(frame,"Color:", (0, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, thickness=2)
        cv2.arrowedLine(frame, (self.midx, self.midy),
                        (self.midx + self.xoffset, self.midy - self.yoffset),
                        (0, 0, 255), 5)
        return frame

    def track(self, frame):
        """Simple HSV color space tracking"""
        # resize the frame, blur it, and convert it to the HSV
        # color space
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                self.xoffset = int(center[0] - self.midx)
                self.yoffset = int(self.midy - center[1])
            else:
                self.xoffset = 0
                self.yoffset = 0
        else:
            self.xoffset = 0
            self.yoffset = 0
        return self.xoffset, self.yoffset
```  
  
`mmt = cv2.moments(cnt)` : 공간, 중심 등의 정보를 가져올 수 있다.   
`cx = int(mmt['m10']/mmt['m00'])`와 `cy = int(mmt['m01']/mmt['m00'])`는 폐곡선의 무게중심을 가져온다.  
  
`cv.circle(img,center,radius,color,thinkness)`   
	img=원이 그려질 이미지  
	center=원의 중심좌표(x,y)  
	radius=원의 반지름   
	color=원의 색(B,G,R)  
	thickness=선굵기   


## 실시간 영상화면 재생 && 드론 명령 송신  

```python
import time
import datetime
import os
import tellopy 			#텔로(드론) 단순동작제어
import numpy
import av			#실시간스트림
import cv2			#opencv
from pynput import keyboard	#keyboard에 의한 동작명령
from tracker import Tracker

def main():
    """ Create a tello controller and show the video feed."""
    tellotrack = TelloCV() 	#class 시작

    for packet in tellotrack.container.demux((tellotrack.vid_stream,)):
        for frame in packet.decode():
            image = tellotrack.process_frame(frame)  #class 내 process_frame
            cv2.imshow('tello', image)
            _ = cv2.waitKey(1) & 0xFF

class TelloCV(object):
	"""~~~~"""

    def init_drone(self):
        """Connect, uneable streaming and subscribe to events"""
        # self.drone.log.set_level(2)
        self.drone.connect()					#드론연결
        self.drone.start_video()				#드론촬영
        self.drone.subscribe(self.drone.EVENT_FLIGHT_DATA,	#드론 비행정보 수신
                             self.flight_data_handler)	
        self.drone.subscribe(self.drone.EVENT_FILE_RECEIVED,	#드론 사진 수신
                             self.handle_flight_received)

    def flight_data_handler(self, event, sender, data):
        """Listener to flight data from the drone."""
        text = str(data)
        if self.prev_flight_data != text:	#prev_fligh_data변수에 수신된 데이터 저장
            self.prev_flight_data = text

    def handle_flight_received(self, event, sender, data):
        """Create a file in ~/Pictures/ to receive image from the drone"""
        path = '%s/Pictures/tello-%s.jpeg' % (
            os.getenv('HOME'),
            datetime.datetime.now().strftime(self.date_fmt))
        with open(path, 'wb') as out_file:
            out_file.write(data)
        print('Saved photo to %s' % path)

```  

`cv2.imshow(tital, image)` : title은 윈도우 창의 제목을 의미하며 image는 cv2.imread() 의 return값입니다  
`cv2.waitKey()`는 키보드 입력을 대기하는 함수로 0이면 key 입력이 있을때까지 무한대기합니다. 특정 시간동안 대기를 하려면 ms값을 넣어주면 됩니다. 또한 ESC를 누를 경우 27을 리턴합니다. 이런 리턴 값을 알면 버튼에 따라 다른 동작을 실시하도록 로직을 설계할 수 있습니다  

```python
class TelloCV(object):					
    def __init__(self): 		#안에서 container 부분만
        # container for processing the packets into frames
        self.container = av.open(self.drone.get_video_stream())
        self.vid_stream = self.container.streams.video[0]
       	self.out_file = None
       	self.out_stream = None
       	self.out_name = None
       	self.start_time = time.time()
```  

```python
    def process_frame(self, frame):
        """convert frame to cv2 image and show"""
        image = cv2.cvtColor(numpy.array(
            frame.to_image()), cv2.COLOR_RGB2BGR) # BGR->HSV로 변환
        image = self.write_hud(image)		  # 함수 호출
        if self.record:
            self.record_vid(frame)

        xoff, yoff = self.tracker.track(image) 
	#tracker.py = HSV color space tracking  # 중심으로부터 공의 좌표를 받아옴
        image = self.tracker.draw_arrows(image) # 벡터 표시

        distance = 100				#그에 따른 동작조절
        cmd = ""
        if self.tracking:
            if xoff < -distance:
                cmd = "counter_clockwise"	#중심으로부터 왼쪽에있다면 반시계방향회전
            elif xoff > distance:
                cmd = "clockwise"		#오른쪽이면 시계방향
            elif yoff < -distance:
                cmd = "down"			#하강
            elif yoff > distance:
                cmd = "up"			#상승
            else:
                if self.track_cmd is not "":
                    getattr(self.drone, self.track_cmd)(0)
                    self.track_cmd = ""


        if cmd is not self.track_cmd:
            if cmd is not "":			#cmd변수가 변경되었다면
                print("track command:", cmd)	
                getattr(self.drone, cmd)(self.speed)	#self.speed로 해당 명령 실행
                self.track_cmd = cmd

        return image	#현재 인식된 공의 위치에서 중심으로의 화살표가 표시된 image return 
```  

```python
    def write_hud(self, frame):
        """Draw drone info, tracking and record on frame"""
        stats = self.prev_flight_data.split('|')	#받아온 비행정보를 기반으로
        stats.append("Tracking:" + str(self.tracking))	#트래킹여부
        if self.drone.zoom:
            stats.append("VID")				#영상or사진 여부
        else:
            stats.append("PIC")
        if self.record:					#동영상촬영시간 기록
            diff = int(time.time() - self.start_time)
            mins, secs = divmod(diff, 60)
            stats.append("REC {:02d}:{:02d}".format(mins, secs))

        for idx, stat in enumerate(stats):
            text = stat.lstrip()
            cv2.putText(frame, text, (0, 30 + (idx * 30)),	#opencv를통해 값출력
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0), lineType=30)
        return frame						#정보기입된 frame 리턴
```  


## 키보드 입력 및 드론 동작 관련  

```python
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
        self.drone = tellopy.Tello()
        self.init_drone()
        self.init_controls()

	#container부분은 상단에 따로 표기설명

        # tracking a color은 고려하지 않아도 되는 부분( openpose혹은 face train data set 사용할 예정)
        green_lower = (30, 50, 50)
        green_upper = (80, 255, 255)
        #red_lower = (0, 50, 50)
        # red_upper = (20, 255, 255)
        # blue_lower = (110, 50, 50)
        # upper_blue = (130, 255, 255)
        self.track_cmd = ""
        self.tracker = Tracker(self.vid_stream.height,
                               self.vid_stream.width,
                               green_lower, green_upper)

```  


```python
    def on_press(self, keyname):
        """handler for keyboard listener"""
        if self.keydown:
            return
        try:
            self.keydown = True
            keyname = str(keyname).strip('\'')	#입력값으로부터 키 문자열
            print('+' + keyname)		
            if keyname == 'Key.esc':		#Key.esc
                self.drone.quit()			
                exit(0)					
            if keyname in self.controls:		
                key_handler = self.controls[keyname]	
                if isinstance(key_handler, str):	#key_handler가 str이면
                    getattr(self.drone, key_handler)(self.speed)	#명령실행
                else:					# == self.dron.key_handler(speed)
                    key_handler(self.speed)		
        except AttributeError:
            print('special key {0} pressed'.format(keyname))

    def on_release(self, keyname):			
        """Reset on key up from keyboard listener"""
        self.keydown = False	
        keyname = str(keyname).strip('\'')
        print('-' + keyname)
        if keyname in self.controls:
            key_handler = self.controls[keyname]
            if isinstance(key_handler, str):
                getattr(self.drone, key_handler)(0)
            else:
                key_handler(0)

    def init_controls(self):
        """Define keys and add listener"""
        self.controls = {
            'w': 'forward',
            's': 'backward',
            'a': 'left',
            'd': 'right',
            'Key.space': 'up',
            'Key.shift': 'down',
            'Key.shift_r': 'down',
            'q': 'counter_clockwise',
            'e': 'clockwise',
            'i': lambda speed: self.drone.flip_forward(),
            'k': lambda speed: self.drone.flip_back(),
            'j': lambda speed: self.drone.flip_left(),
            'l': lambda speed: self.drone.flip_right(),
            # arrow keys for fast turns and altitude adjustments
            'Key.left': lambda speed: self.drone.counter_clockwise(speed),
            'Key.right': lambda speed: self.drone.clockwise(speed),
            'Key.up': lambda speed: self.drone.up(speed),
            'Key.down': lambda speed: self.drone.down(speed),
            'Key.tab': lambda speed: self.drone.takeoff(),
            'Key.backspace': lambda speed: self.drone.land(),
            'p': lambda speed: self.palm_land(speed),
            't': lambda speed: self.toggle_tracking(speed),
            'r': lambda speed: self.toggle_recording(speed),
            'z': lambda speed: self.toggle_zoom(speed),
            'Key.enter': lambda speed: self.take_picture(speed)
        }
        self.key_listener = keyboard.Listener(on_press=self.on_press,
                                              on_release=self.on_release)
        self.key_listener.start()
        # self.key_listener.join()
```  


```python
    def toggle_recording(self, speed):
        """Handle recording keypress, creates output stream and file"""
        if speed == 0:
            return
        self.record = not self.record	#버튼 하나로 on/off제어(?)

        if self.record:
            datename = [os.getenv('HOME'), datetime.datetime.now().strftime(self.date_fmt)]
            self.out_name = '{}/Pictures/tello-{}.mp4'.format(*datename)
            print("Outputting video to:", self.out_name)
            self.out_file = av.open(self.out_name, 'w')
            self.start_time = time.time()
            self.out_stream = self.out_file.add_stream(
                'mpeg4', self.vid_stream.rate)
            self.out_stream.pix_fmt = 'yuv420p'
            self.out_stream.width = self.vid_stream.width
            self.out_stream.height = self.vid_stream.height

        if not self.record:
            print("Video saved to ", self.out_name)
            self.out_file.close()
            self.out_stream = None

    def record_vid(self, frame):
        """
        convert frames to packets and write to file
        """
        new_frame = av.VideoFrame(
            width=frame.width, height=frame.height, format=frame.format.name)
        for i in range(len(frame.planes)):
            new_frame.planes[i].update(frame.planes[i])
        pkt = None
        try:
            pkt = self.out_stream.encode(new_frame)
        except IOError as err:
            print("encoding failed: {0}".format(err))
        if pkt is not None:
            try:
                self.out_file.mux(pkt)
            except IOError:
                print('mux failed: ' + str(pkt))

    def take_picture(self, speed):
        """Tell drone to take picture, image sent to file handler"""
        if speed == 0:
            return
        self.drone.take_picture()

    def palm_land(self, speed):
        """Tell drone to land"""
        if speed == 0:
            return
        self.drone.palm_land()

    def toggle_tracking(self, speed):
        """ Handle tracking keypress"""
        if speed == 0:  # handle key up event
            return
        self.tracking = not self.tracking
        print("tracking:", self.tracking)
        return

    def toggle_zoom(self, speed):
        """
        In "video" mode the self.drone sends 1280x720 frames.
        In "photo" mode it sends 2592x1936 (952x720) frames.
        The video will always be centered in the window.
        In photo mode, if we keep the window at 1280x720 that gives us ~160px on
        each side for status information, which is ample.
        Video mode is harder because then we need to abandon the 16:9 display size
        if we want to put the HUD next to the video.
        """
        if speed == 0:
            return
        self.drone.set_video_mode(not self.drone.zoom)
```  

