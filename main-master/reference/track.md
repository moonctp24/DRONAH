```python
                        elif self.pose == "CLOSE_HANDS_UP":
                            # Locked distance mode
                            if self.keep_distance is None:
                                if  time.time() - self.timestamp_keep_distance > 2:
                                    # The first frame of a serie to activate the distance keeping
                                    self.keep_distance = self.shoulders_width
                                    self.timestamp_keep_distance = time.time()
                                    log.info(f"KEEP DISTANCE {self.keep_distance}")
                                    self.pid_pitch = PID(0.5,0.04,0.3,setpoint=0,output_limits=(-50,50))
                                    #self.graph_distance = RollingGraph(window_name="Distance", y_max=500, threshold=self.keep_distance, waitKey=False)
                                    self.sound_player.play("keeping distance")
```
```python
                        elif self.pose == "RIGHT_HAND_ON_LEFT_EAR":
                            # Get close to the body then palm landing
                            if not self.palm_landing_approach:
                                self.palm_landing_approach = True
                                self.keep_distance = proximity
                                self.timestamp_keep_distance = time.time()
                                log.info("APPROACHING on pose")
                                self.pid_pitch = PID(0.2,0.02,0.1,setpoint=0,output_limits=(-45,45))
                                #self.graph_distance = RollingGraph(window_name="Distance", y_max=500, threshold=self.keep_distance, waitKey=False)
                                self.sound_player.play("approaching")
```

## pitch부분  (카메라와 인간의 거리 조정)
행동에 따른 거리변화를 지시하는 두 소스코드  
이 두 개를 비교하면, 거리를 정해놓은 것 같음.  proximity는 frame가로넓이에 2.7만큼 나눈 수임 (정확한 임의의 길이)  
어깨넓이가 의문인게.. 멀리있을 때 그 사람을 측정했으면 어깨가 무지 좁을텐데 무슨기준인지 궁금 --debug모드로 실행을 해봐야할 거 같음  

<p align="center">
    <img src="/reference/media/openpose_track2.png", width="480">
</p>  

```python
        if self.keep_distance: 
            hud.add(f"Target distance: {self.keep_distance} - curr: {self.shoulders_width}", (0,255,0))
```

영상(self.pose == "RIGHT_HAND_ON_LEFT_EAR"상태) 참고하면, curr가 점점 커지면서 246에 가까워짐.  
오.... 그러면 locked distance는 2초? 2ms? 간격으로 어깨너비 확인하는듯  
그러니까 self.keep_distance는 현재 원하는 거리 입력.. 이라고도 볼 수 있음  

```python
                    target = self.op.get_body_kp("Nose")
                    if target is not None:        
                        ref_x = int(w/2)
                        ref_y = int(h*0.35)
                    else:
                        target = self.op.get_body_kp("Neck")
                        if target is not None:         
                            ref_x = int(w/2)
                            ref_y = int(h/2)
                        else:
                            target = self.op.get_body_kp("MidHip")
                            if target is not None:         
                                ref_x = int(w/2)
                                ref_y = int(0.75*h)
```
### ref_x와 ref_y를 설정함.  

```python
                if self.tracking:
                    if target:
                        self.body_in_prev_frame = True
                        # We draw an arrow from the reference point to the body part we are targeting       
                        h,w,_ = frame.shape
                        xoff = int(target[0]-ref_x)
                        yoff = int(ref_y-target[1])
                        cv2.circle(frame, (ref_x, ref_y), 15, (250,150,0), 1,cv2.LINE_AA)
                        cv2.arrowedLine(frame, (ref_x, ref_y), target, (250, 150, 0), 6)
                       
                        # The PID controllers calculate the new speeds for yaw and throttle
                        self.axis_speed["yaw"] = int(-self.pid_yaw(xoff))
                        log.debug(f"xoff: {xoff} - speed_yaw: {self.axis_speed['yaw']}")
                        self.last_rotation_is_cw = self.axis_speed["yaw"] > 0

                        self.axis_speed["throttle"] = int(-self.pid_throttle(yoff))
                        log.debug(f"yoff: {yoff} - speed_throttle: {self.axis_speed['throttle']}")

                        # If in locke distance mode
                        if self.keep_distance and self.shoulders_width:   
                            if self.palm_landing_approach and self.shoulders_width>self.keep_distance:
                                # The drone is now close enough to the body
                                # Let's do the palm landing
                                log.info("PALM LANDING after approaching")
                                self.palm_landing_approach = False
                                self.toggle_tracking(tracking=False)
                                self.palm_land() 
                            else:
                                self.axis_speed["pitch"] = int(self.pid_pitch(self.shoulders_width-self.keep_distance))
                                log.debug(f"Target distance: {self.keep_distance} - cur: {self.shoulders_width} -speed_pitch: {self.axis_speed['pitch']}")
```


## yaw와 throttle 부분  (yaw = 시계반시계 회전정도 / throttle = 위 아래 높이 조절)  
일단 h,w는 프레임의 좌상단이 (0,0) 기준으로 되어, (|+가로좌표|,|-세로좌표|)처럼 기록되어있다고 생각함 (코,목,순서대로 사람인식을 통해 ref_x, ref_y값 저장 소스코드 참고)

<p align="center">
    <img src="/reference/media/openpose_track1.png", width="480">
</p>  

target[] = {해당 사람의 x좌표,y좌표}가 입력되어있다고 친다면 (영상 기준 target = self.op.get_body_kp("Nose") ),  
일단 xoff yoff는 원하는 중심과 현재 타겟(코)의 distance를 의미한다.  

<p align="center">
    <img src="/reference/media/openpose_drawArrow.png", width="480">
</p>  

즉 (ref_x, ref_y)에서 target(사람코)까지의 화살표를 그리게 됨.  
self.axis_speed["yaw"] = int(-self.pid_yaw(xoff))  
self.axis_speed["throttle"] = int(-self.pid_throttle(yoff))  
그래서 각자 xoff랑 yoff만 써서 화살표의 길이를 줄이는 방향으로 PID 사용함   

