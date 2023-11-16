## <div align="center">2023 제 21회 임베디드 소프트웨어 경진대회 - 지능형 휴머노이드 </div>
🚀 지능형 휴머노이드 로봇을 이용한 로봇 골프 경기 수행

## <div align="center">Team 서계</div>
🌟 Team Leader  이창재 (서강대학교 기계공학과 19)

🌟 Team member 강정훈 (서강대학교 기계공학과 19)

🌟 Team member 김기훈 (서강대학교 기계공학과 / 컴퓨터공학과 19)

🌟 Team member 이도헌 (서강대학교 기계공학과 19)

## <div align="center">Video</div>

🚀 See our operation video on the YouTube

[Youtube_link](https://www.youtube.com/watch?si=oTYd6pSQUsP_0D2D&v=IUE--1FHvtU&feature=youtu.be)


<p align="center"><img width="800" src="https://github.com/cobang0111/2023ESWContest_humanoid_2014/blob/main/img/youtube_main.png"></p>


## <div align="center">Competition Summary</div>

🚀 휴머노이드 정보

<p align="center"><img width="800" src="https://github.com/cobang0111/2023ESWContest_humanoid_2014/blob/main/img/MF-RAPI4.png"></p>


🚀 경기장

- PAR 3
  <p align="center"><img width="800" src="https://github.com/cobang0111/2023ESWContest_humanoid_2014/blob/main/img/par3.png"></p>
  
- PAR 4
  <p align="center"><img width="800" src="https://github.com/cobang0111/2023ESWContest_humanoid_2014/blob/main/img/par4.png"></p>

- DETAIL
  <p align="center"><img width="800" src="https://github.com/cobang0111/2023ESWContest_humanoid_2014/blob/main/img/par_detail.png"></p>


🚀 경기 규칙
- 팀별 2번의 경기(파3+파4)를 진행하여, 가장 낮은 타수의 경기를 팀 점수로 한다.

- 파3, 파4 각각 5분으로 총 10분으로 경기 시간은 한정된다.

- 세리머니 여부는 파3, 파4 경기마다 확인된다. (세리머니 동작 O : -1 , X : 0, 잘못된 세리머니 : +1 )

- 골프공을 티샷 3곳 중 임의에 위치에 놓는다.

- 로봇을 차렷 상태로, 시작 위치에 놓고 경기를 시작한다.

- 경기 시작 후 참가자는 더 이상 로봇을 만질 수 없음.

- 로봇이 동작할 때부터 시간 측정 시작

- 시작 위치는 매 경기 달라질 수 있음.

- 첫 티샷이 밖으로 나가는 경우 OB로, 해저드 티에 운영자가 공을 갖다 놓고, 2타가 추가된다.

- 세컨드 샷 이후 밖으로 나가는 것은 해저드로, 로봇 앞으로 운영자가 공을 갖다 놓고, 1타가 추가된다.

- 로봇이 밖으로 이탈하는 경우, 해당 홀 양파+1타를 점수로 받고 해당 홀을 종료한다. (예: 파3의 경우 7점)

- 공을 칠 수 없다고 판단되는 경우, 참가자는 +1타를 받고, 공의 위치를 이동시킬 수 있다. (운영자가 공을 옮긴다.)

- 경기장 진동 및 기타 요인으로 공이 이동하는 것은 인정된다. 하지만 홀인의 경우 공의 움직임은 무효 처리되어, 원래의 위치로 공을 이동시킨다.

- 경기장의 기울기는 MAX ±5도 미만의 오차를 가질 수 있다.
  <p align="center"><img width="800" src="https://github.com/cobang0111/2023ESWContest_humanoid_2014/blob/main/img/par_degree.png"></p>
  
- 홀 컵의 깊이는 약 12mm로, 공이 들어갔다 나오는 것은 인정되지 않는다.

- 중도 포기의 경우 양파 +2타로 최종 기록된다.(예: 파3의 경우 8점)


## <div align="center">Code</div>

🚀 Code - Python3 Code (r.py)

<p align = "center"> ESW_sg_v231107.py </p>


  
파일 실행 및 저장된 HSV 데이터 로드드


 
```python
# -*- coding: utf-8 -*-

import platform
import numpy as np
import argparse
import cv2
import serial
import time
import sys
from threading import Thread
import csv
import math


X_255_point = 0
Y_255_point = 0
X_Size = 0
Y_Size = 0
Area = 0
Angle = 0

#-----------------------------------------------

Top_name = 'mini CTS5 setting'

hsv_Lower = 0

hsv_Upper = 0


hsv_Lower0 = 0

hsv_Upper0 = 0


hsv_Lower1 = 0

hsv_Upper1 = 0


#----------- 

color_num = [   0,  1,  2,  3,  4]


h_max =     [ 255, 65,196,111,110]

h_min =     [  55,  0,158, 59, 74]


s_max =     [ 162,200,223,110,255]

s_min =     [ 114,140,150, 51,133]


v_max =     [  77,151,239,156,255]

v_min =     [   0,95,104, 61,104]


min_area =  [  50, 50, 50, 10, 10]

now_color = 0

serial_use = 1

serial_port =  None

Temp_count = 0

Read_RX =  0

mx,my = 0,0

threading_Time = 5/1000.

Config_File_Name ='Cts5_v1.dat'

#---------------------------------------------------------

```

Code to Real Operation 상수 및 헤드 각도를 기록하는 글로벌 변수

```python

# CONSTANT

SIZE_CONSTANT = 5.0
DIST_CONSTANT = 0.16
PIXEL_X_EPSILON = 30
PIXEL_Y_EPSILON = 20

# 사용자 global 변수
    
cur_theta = 30
cur_theta_index = 3
head_serial = [101, 102 ,103, 104, 105, 106, 107, 108, 109, 110, 112, 113, 114, 115, 116, 117, 118]


```

기본 함수

```python

#-----------------------------------------------

def nothing(x):
    pass

#-----------------------------------------------

def create_blank(width, height, rgb_color=(0, 0, 0)):
    image = np.zeros((height, width, 3), np.uint8)
    color = tuple(reversed(rgb_color))
    image[:] = color
    return image

#-----------------------------------------------

def draw_str2(dst, target, s):
    x, y = target
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, 0.8, (0, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 255), lineType=cv2.LINE_AA)

#-----------------------------------------------

def draw_str3(dst, target, s):
    x, y = target
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), lineType=cv2.LINE_AA)

#-----------------------------------------------

def draw_str_height(dst, target, s, height):
    x, y = target
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, height, (0, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, height, (255, 255, 255), lineType=cv2.LINE_AA)

#-----------------------------------------------

def clock():
    return cv2.getTickCount() / cv2.getTickFrequency()

#-----------------------------------------------

def Trackbar_change(now_color):
    global  hsv_Lower,  hsv_Upper
    hsv_Lower = (h_min[now_color], s_min[now_color], v_min[now_color])
    hsv_Upper = (h_max[now_color], s_max[now_color], v_max[now_color])

#-----------------------------------------------

def Hmax_change(a):
    h_max[now_color] = cv2.getTrackbarPos('Hmax', Top_name)
    Trackbar_change(now_color)

#-----------------------------------------------

def Hmin_change(a):
    h_min[now_color] = cv2.getTrackbarPos('Hmin', Top_name)
    Trackbar_change(now_color)

#-----------------------------------------------

def Smax_change(a):
    s_max[now_color] = cv2.getTrackbarPos('Smax', Top_name)
    Trackbar_change(now_color)

#-----------------------------------------------

def Smin_change(a):
    s_min[now_color] = cv2.getTrackbarPos('Smin', Top_name)
    Trackbar_change(now_color)

#-----------------------------------------------

def Vmax_change(a):
    v_max[now_color] = cv2.getTrackbarPos('Vmax', Top_name)
    Trackbar_change(now_color)

#-----------------------------------------------

def Vmin_change(a):
    v_min[now_color] = cv2.getTrackbarPos('Vmin', Top_name)
    Trackbar_change(now_color)

#-----------------------------------------------

def min_area_change(a):
    min_area[now_color] = cv2.getTrackbarPos('Min_Area', Top_name)
    if min_area[now_color] == 0:
        min_area[now_color] = 1
        cv2.setTrackbarPos('Min_Area', Top_name, min_area[now_color])
    Trackbar_change(now_color)

#-----------------------------------------------

def Color_num_change(a):
    global now_color, hsv_Lower,  hsv_Upper
    now_color = cv2.getTrackbarPos('Color_num', Top_name)
    cv2.setTrackbarPos('Hmax', Top_name, h_max[now_color])
    cv2.setTrackbarPos('Hmin', Top_name, h_min[now_color])
    cv2.setTrackbarPos('Smax', Top_name, s_max[now_color])
    cv2.setTrackbarPos('Smin', Top_name, s_min[now_color])
    cv2.setTrackbarPos('Vmax', Top_name, v_max[now_color])
    cv2.setTrackbarPos('Vmin', Top_name, v_min[now_color])
    cv2.setTrackbarPos('Min_Area', Top_name, min_area[now_color])
    hsv_Lower = (h_min[now_color], s_min[now_color], v_min[now_color])
    hsv_Upper = (h_max[now_color], s_max[now_color], v_max[now_color])

#----------------------------------------------- 

def TX_data(ser, one_byte):  # one_byte= 0~255
    #ser.write(chr(int(one_byte)))          #python2.7
    ser.write(serial.to_bytes([one_byte]))  #python3

#-----------------------------------------------

def RX_data(serial):
    global Temp_count
    try:
        if serial.inWaiting() > 0:
            result = serial.read(1)
            RX = ord(result)
            return RX
        else:
            return 0
    except:
        Temp_count = Temp_count  + 1
        print("Serial Not Open " + str(Temp_count))
        return 0
        pass

#-----------------------------------------------

#*************************

# mouse callback function

def mouse_move(event,x,y,flags,param):
    global mx, my
    if event == cv2.EVENT_MOUSEMOVE:
        mx, my = x, y


# *************************

def RX_Receiving(ser):
    global receiving_exit,threading_Time
    global X_255_point
    global Y_255_point
    global X_Size
    global Y_Size
    global Area, Angle

    receiving_exit = 1
    while True:
        if receiving_exit == 0:
            break
        time.sleep(threading_Time)

        while ser.inWaiting() > 0:
            result = ser.read(1)
            RX = ord(result)
            print ("RX=" + str(RX))

            
# *************************

def GetLengthTwoPoints(XY_Point1, XY_Point2):
    return math.sqrt( (XY_Point2[0] - XY_Point1[0])**2 + (XY_Point2[1] - XY_Point1[1])**2 )

# *************************

def FYtand(dec_val_v ,dec_val_h):
    return ( math.atan2(dec_val_v, dec_val_y) * (180.0 / math.pi))

# *************************

#degree 값을 라디안 값으로 변환하는 함수

def FYrtd(rad_val ):
    return  (rad_val * (180.0 / math.pi))


# *************************

# 라디안값을 degree 값으로 변환하는 함수

def FYdtr(dec_val):
    return  (dec_val / 180.0 * math.pi)

# *************************

def GetAngleTwoPoints(XY_Point1, XY_Point2):
    xDiff = XY_Point2[0] - XY_Point1[0]
    yDiff = XY_Point2[1] - XY_Point1[1]
    cal = math.degrees(math.atan2(yDiff, xDiff)) + 90

    if cal > 90:
        cal =  cal - 180
    return  cal

# *************************


# ************************

def hsv_setting_save():
    global Config_File_Name, color_num
    global color_num, h_max, h_min 
    global s_max, s_min, v_max, v_min, min_area

    try:
    #if 1:
        saveFile = open(Config_File_Name, 'w')
        i = 0
        color_cnt = len(color_num)

        while i < color_cnt:
            text = str(color_num[i]) + ","
            text = text + str(h_max[i]) + "," + str(h_min[i]) + ","
            text = text + str(s_max[i]) + "," + str(s_min[i]) + ","
            text = text + str(v_max[i]) + "," + str(v_min[i]) + ","
            text = text + str(min_area[i])  + "\n"
            saveFile.writelines(text)
            i = i + 1

        saveFile.close()
        print("hsv_setting_save OK")
        return 1

    except:
        print("hsv_setting_save Error~")
        return 0


#************************

def hsv_setting_read():
    global Config_File_Name
    global color_num, h_max, h_min 
    global s_max, s_min, v_max, v_min, min_area

    #try:
    if 1:
        with open(Config_File_Name) as csvfile:
            readCSV = csv.reader(csvfile, delimiter=',')
            i = 0

            for row in readCSV:
                color_num[i] = int(row[0])
                h_max[i] = int(row[1])
                h_min[i] = int(row[2])
                s_max[i] = int(row[3])
                s_min[i] = int(row[4])
                v_max[i] = int(row[5])
                v_min[i] = int(row[6])
                min_area[i] = int(row[7])
                i = i + 1

        csvfile.close()
        print("hsv_setting_read OK")
        return 1

    #except:
    #    print("hsv_setting_read Error~")
    #    return 0

```
 
추가 정의 함수
 

```python
    
# ******************** sg CUSTOMIZE FUNCTION *********************

# 물체를 좌우 타겟 위치에 놓기 위해 로봇 동작을 단 한 번 조절하는 함수
def obj_x_centering(serial_port, obj_x_center, robot_flag, dir_flag):
    # Current head direction left = -1 / center = 0 / right = +1
    global head_dir
    print(" **** X CENTERING ! **** ")
    print("CURRENT HEAD DIR = ", head_dir)

    # 공을 헤드를 돌린 상태에서 찾았을 때 몸 방향을 공 방향으로 바꾸도록 함
    # 현재 헤드 방향이 오른쪽 -> 로봇을 반시계방향으로 돌리고, 헤드를 중심으로 복귀
    if head_dir == 1:
        TX_data(serial_port, 19) # Robot rotation to ccw 45'
        time.sleep(3)
        print("ROBOT DIRECTION CHANGE! CounterClockwise")
        TX_data(serial_port, 21) #Head direction to center
        time.sleep(3)
        head_dir = 0
        print("HEAD DIR Changed = ", head_dir)

    # 현재 헤드 방향이 왼쪽 -> 로봇을 시계방향으로 돌리고, 헤드를 중심으로 복귀    
    elif head_dir == -1:
        TX_data(serial_port, 25) # Robot rotation to cw 45'
        time.sleep(3)
        print("ROBOT DIRECTION CHANGE! Clockwise")
        TX_data(serial_port, 21) #Head direction to center
        time.sleep(3)
        head_dir = 0
        print("HEAD DIR Changed = ", head_dir)
                            
    # Robot rotation control
    # 함수에 입력된 dir_flag 에 따라 공을 기준으로 로봇이 어느 방향으로 걸어갈 지 결정     
    # 공 왼쪽 편으로 이동하도록 
    print("dir_flag = ",dir_flag)
    if dir_flag < 0:
        if obj_x_center < ((W_View_size)*5/6) :
            print("object_x_center = ", obj_x_center, "-> Need to turn left!")
            TX_data(serial_port, 4)
            time.sleep(3)
        else :
            print("DIRECTION Satisfied!")
            print("object_x_center = ", obj_x_center)
            robot_flag = 1 # Mean robot satisfy the direction
    # 공 오른쪽 편으로 이동하도록
    elif dir_flag > 0:
        if obj_x_center > ((W_View_size)/6) :
            print("object_x_center = ", obj_x_center, "-> Need to turn right!")
            TX_data(serial_port, 6)
            time.sleep(3)
        else :
            print("robot already satisfy the standard!")
            print("object_x_center = ", obj_x_center)
            robot_flag = 1 # Mean robot satisfy the direction
    # 공 중심 쪽으로 이동
    else:
        if obj_x_center < ((W_View_size)/2 * 0.95) :
            print("object_x_center = ", obj_x_center, "-> Need to turn left!")
            TX_data(serial_port, 4)
            time.sleep(3)
        elif obj_x_center > ((W_View_size)/2 * 1.05) :
            print("object_x_center = ", obj_x_center, "-> Need to turn right!")
            TX_data(serial_port, 6)
            time.sleep(3)
        else :
            print("robot already satisfy the standard!")
            robot_flag = 1 

    return robot_flag


#물체를 화면 y축 가운데 놓기 위해 로봇 헤드 각도를 한 번 조절하는 함수
def obj_y_centering(serial_port, obj_y_center, head_flag):
    print(" **** Y CENTERING ! **** ")
    global cur_theta
    global cur_theta_index
    # Head position control
    if obj_y_center > (H_View_size)/2 * 1.2 :
        print("object_y_center = ", obj_y_center, "-> Need to head down!")
        if cur_theta_index >= len(head_serial)-1:
            print("HEAD LOWER LIMIT : Impossible! -> cur_theta Inintializing")
            cur_theta = 20
            cur_theta_index = 1
        else:
            cur_theta += 5
            cur_theta_index+=1
            TX_data(serial_port, head_serial[cur_theta_index])
            time.sleep(3)
    elif obj_y_center < (H_View_size)/2 * 0.8:
        print("object_y_center = ", obj_y_center, "-> Need to head up!")
        if cur_theta_index <= 0:
            print("HEAD UPPER LIMIT : Impossible! -> cur_theta Initializing")
            cur_theta = 90
            cur_theta_index = 15
        else:
            cur_theta -= 5
            cur_theta_index-=1
            TX_data(serial_port, head_serial[cur_theta_index])
            time.sleep(3)
    else:
        print("Head already satisfy the standard!")
        head_flag = 1 # Mean head degree satisfied

    return head_flag


```


main Function


```python

# **************************************************

# **************************************************

# **************************************************

if __name__ == '__main__':



    #-------------------------------------

    print ("-------------------------------------")

    print ("(2020-1-20) mini CTS5 Program.  MINIROBOT Corp.")

    print ("-------------------------------------")

    print ("")

    os_version = platform.platform()

    print (" ---> OS " + os_version)

    python_version = ".".join(map(str, sys.version_info[:3]))

    print (" ---> Python " + python_version)

    opencv_version = cv2.__version__

    print (" ---> OpenCV  " + opencv_version)   

    #-------------------------------------

    #---- user Setting -------------------

    #-------------------------------------

    W_View_size = 800 #320  #320  #640
    H_View_size = 500
    #H_View_size = int(W_View_size / 1.777)
    #H_View_size = 600  #int(W_View_size / 1.333)
    
    BPS =  4800  # 4800,9600,14400, 19200,28800, 57600, 115200
    serial_use = 1
    View_select = 0
    
    #-------------------------------------
    
    now_color = 0 #0 = Ball , 1 = Flag
    f_count = 0 # frame count variable
    
    #-------------------------------------

    print(" ---> Camera View: " + str(W_View_size) + " x " + str(H_View_size) )
    print ("")
    print ("-------------------------------------")

    #-------------------------------------

    try:
        hsv_setting_read()

    except:
        hsv_setting_save()

    #-------------------------------------

    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video",
                    help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=64,
                    help="max buffer size")
    args = vars(ap.parse_args())
    img = create_blank(320, 100, rgb_color=(0, 0, 255))
    cv2.namedWindow(Top_name)
    cv2.moveWindow(Top_name,0,0)
    cv2.createTrackbar('Hmax', Top_name, h_max[now_color], 255, Hmax_change)
    cv2.createTrackbar('Hmin', Top_name, h_min[now_color], 255, Hmin_change)
    cv2.createTrackbar('Smax', Top_name, s_max[now_color], 255, Smax_change)
    cv2.createTrackbar('Smin', Top_name, s_min[now_color], 255, Smin_change)
    cv2.createTrackbar('Vmax', Top_name, v_max[now_color], 255, Vmax_change)
    cv2.createTrackbar('Vmin', Top_name, v_min[now_color], 255, Vmin_change)
    cv2.createTrackbar('Min_Area', Top_name, min_area[now_color], 255, min_area_change)
    cv2.createTrackbar('Color_num', Top_name,color_num[now_color], 4, Color_num_change)

    Trackbar_change(now_color)

    draw_str3(img, (15, 25), 'MINIROBOT Corp.')
    draw_str2(img, (15, 45), 'space: Fast <=> Video and Mask.')
    draw_str2(img, (15, 65), 's, S: Setting File Save')
    draw_str2(img, (15, 85), 'Esc: Program Exit')

    cv2.imshow(Top_name, img)

    #---------------------------

    if not args.get("video", False):
        camera = cv2.VideoCapture(0)

    else:
        camera = cv2.VideoCapture(args["video"])

    #---------------------------

    camera.set(3, W_View_size)
    camera.set(4, H_View_size)
    camera.set(5, 60)
    time.sleep(0.5)

    #---------------------------

    (grabbed, frame) = camera.read()
    draw_str2(frame, (5, 15), 'X_Center x Y_Center =  Area' )
    draw_str2(frame, (5, H_View_size - 5), 'View: %.1d x %.1d.  Space: Fast <=> Video and Mask.'
                      % (W_View_size, H_View_size))
    draw_str_height(frame, (5, int(H_View_size/2)), 'Fast operation...', 3.0 )
    mask = frame.copy()
    cv2.imshow('mini CTS5 - Video', frame )
    cv2.imshow('mini CTS5 - Mask', mask)
    cv2.moveWindow('mini CTS5 - Mask',322 + W_View_size,36)
    cv2.moveWindow('mini CTS5 - Video',322,36)
    cv2.setMouseCallback('mini CTS5 - Video', mouse_move)

    #---------------------------

    if serial_use != 0:  # python3
    #if serial_use <> 0:  # python2.7
        BPS =  4800  # 4800,9600,14400, 19200,28800, 57600, 115200

        #---------local Serial Port : ttyS0 --------

        #---------USB Serial Port : ttyAMA0 --------

        serial_port = serial.Serial('/dev/ttyS0', BPS, timeout=0.01)
        serial_port.flush() # serial cls
        time.sleep(0.5)

        serial_t = Thread(target=RX_Receiving, args=(serial_port,))
        serial_t.daemon = True
        serial_t.start()


    # First -> Start Code Send 

    TX_data(serial_port, 250)
    TX_data(serial_port, 250)
    TX_data(serial_port, 250)
    TX_data(serial_port, 21)

    old_time = clock()

    View_select = 0
    msg_one_view = 0

```

Main Operation Loop 

```python

    # -------- Main Loop Start --------

    detect_count_ball = 0 # Ball frame count 변수
    detect_count_flag = 0 # Flag frame count 변수
    
    detect_count_2 = 0 # non-use
    detect_count_3 = 0 # non-use
    detect_count_4 = 0 # non-use

    non_detect_count = 0 # nothing detected frame count 변수

    head_condition = 0 # 로봇 시야 Y 중심에 공이 위치했는지 판단하는 bool 변수
    robot_condition = 0 # 로봇 시야 X 중심에 공이 위치했는지 판단하는 bool 변수
    
    head_dir = 0 # 현재 로봇의 헤드 방향 / -1 = left / 0 = center / 1 = right

    ball_theta = 0 # 공이 인식된 헤드 상하 각도 기록 변수
    ball_theta_index = -1 # 공의 serial array index 기록 변수
    
    same_flag = 0 # 깃발 - 공 - 로봇 순으로 일렬 배열된 상태인지 판단하는 bool 변수
    shot_flag = 0 # same_flag = 1일 때, shot 이 가능한 거리인지 판단하는 bool 변수
    shot_turn_flag = 0 #shot_flag = 1일 때, turn 하여 깃발이 로봇 좌측에 있음을 나타내는 bool 변수
    far_flag = 0 # 공과 로봇의 거리가 먼 상태인지 판단하는 bool 변수

    flag_detected = 0 # 초기 상태에서 공 탐지 이전에 깃발을 탐지했음을 나타내는 bool 변수 
    flag_check = 0 # 공과 깃발이 한 시야에 들어오지 않을 때 깃발 좌표를 기록했음을 나타내는 bool 변수
    saming = 0 #공과 깃발이 한 시야에 들어오지 않을 때 깃발- 공 - 로봇 일렬 정렬을 맞추는 과정을 수행 중임을 나타내는 bool 변수
    
    ball_flag = 0 # 공이 인식된 헤드 좌우 방향을 기록하는 변수
    flag_flag = 0 # 깃발이 인식된 헤드 좌우 방향을 기록하는 변수
    long = 0 # 공과 깃발이 반대 방향에 있는 경우를 기록하는 변수

    direction_flag = 0 # 로봇 보행 방향을 결정하는 변수 / -1 = 공의 왼쪽 / 0 = 공 정면 / 1 = 공의 오른쪽 
    
    flag_x_center = 0 # 깃발의 화면 상 pixel x 좌표를 저장하는 변수
    flag_y_center = 0 # 깃발의 화면 상 pixel y 좌표를 저장하는 변수
    ball_x_center = 0 # 공의 화면 상 pixel x 좌표를 저장하는 변수
    ball_y_center = 0 # 공의 화면 상 pixel y 좌표를 저장하는 변수
    
    # MAIN LOOP until ceremony
    while True:

        # grab the current frame
        (grabbed, frame) = camera.read()
        if args.get("video") and not grabbed:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)  # HSV => YUV
        # Mask using now_color
        # Not affect to real
        mask = cv2.inRange(hsv, hsv_Lower, hsv_Upper)
        hsv_Lower = (h_min[now_color], s_min[now_color], v_min[now_color])
        hsv_Upper = (h_max[now_color], s_max[now_color], v_max[now_color])
        
        # mask0 -> ball mask
        mask0 = cv2.inRange(hsv, (h_min[0], s_min[0], v_min[0]), (h_max[0], s_max[0], v_max[0]))
        # mask1 -> flag mask
        mask1 = cv2.inRange(hsv, (h_min[1], s_min[1], v_min[1]), (h_max[1], s_max[1], v_max[1]))
        '''
        mask2 = cv2.inRange(hsv, (h_min[2], s_min[2], v_min[2]), (h_max[2], s_max[2], v_max[2]))
        mask3 = cv2.inRange(hsv, (h_min[3], s_min[3], v_min[3]), (h_max[3], s_max[3], v_max[3]))
        mask4 = cv2.inRange(hsv, (h_min[4], s_min[4], v_min[4]), (h_max[4], s_max[4], v_max[4]))
        '''

        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)
        mask0 = cv2.erode(mask0, None, iterations=1)
        mask0 = cv2.dilate(mask0, None, iterations=1)
        mask1 = cv2.erode(mask1, None, iterations=1)
        mask1 = cv2.dilate(mask1, None, iterations=1)

        #mask = cv2.GaussianBlur(mask, (5, 5), 2)  # softly

        # Detecting
        #cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts0 = cv2.findContours(mask0.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts1 = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        '''
        cnts2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts3 = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts4 = cv2.findContours(mask4.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        '''
        center = None

```


시야 내 공 감지 Counting

```python

        # *********************  Ball detected  **************************
        if len(cnts0) > 0:
            c0 = max(cnts0, key=cv2.contourArea)
            #((X, Y), radius) = cv2.minEnclosingCircle(c0)

            Area0 = cv2.contourArea(c0) / min_area[0]
            if Area0 > 255:
                Area0 = 255

            if Area0 > min_area[0]:
                detect_count_ball += 1 

                x0, y0, w0, h0 = cv2.boundingRect(c0)
                
                cv2.rectangle(frame, (x0, y0), (x0 + w0, y0 + h0), (0, 0, 255), 2)
            
            else:
                TX_data(serial_port, 1) # turn left shortly
                time.sleep(2)

```

시야 내 깃발 감지 counting

```python

        # *********************  Flag detected  **************************
        if len(cnts1) > 0:
            c1 = max(cnts1, key=cv2.contourArea)
            #((X, Y), radius) = cv2.minEnclosingCircle(c1)

            Area1 = cv2.contourArea(c1) / min_area[1]
            if Area1 > 255:
                Area1 = 255

            if Area1 > min_area[1]:
                detect_count_flag += 1

                x1, y1, w1, h1 = cv2.boundingRect(c1)
                
                cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 255), 2)

            else:
                TX_data(serial_port, 1) # turn left shortly
                time.sleep(2)

```

시야 내 detecting object 존재하지 않음

```python


        # *********************  nothing detected  **************************
        if len(cnts0) <= 0 and len(cnts1) <= 0:
            x = 0
            y = 0
            X_255_point = 0
            Y_255_point = 0
            X_Size = 0
            Y_Size = 0
            Area0 = 0
            Area1 = 0
            Angle = 0
            non_detect_count += 1

```


object detection 기반 조건문 수행


공 감지 & shot_flag = 1 & shot_turn_flag = 1

공의 위치를 화면 Width 의 3/4에 도달할 때 까지 왼쪽으로 움직이고, 도달 시 공프공을 타격한다. 
타격 후에 모든 변수를 초기화 하고 공을 타격한 방향으로 몸을돌린다. 
상황에 대한 예시는 다음 그림과 같다.

<p align="center"><img width="800" src="https://github.com/cobang0111/2023ESWContest_humanoid_2014/blob/main/img/1_shot_turn.png"></p>
 

```python

        # *********************  로봇 동작 조건 분기 시작  **************************

        # ********* 모든 동작은 안정성을 위해 20프레임 감지 되었을 때 수행됨 ***********


        # ********************* Action : 로봇이 골프공을 타격  **************************
        # Shot possible (조건 : 공 감지, shot flag, shot turn flag -> 결과 : 공 위치 맞춰 shot)

        if detect_count_ball > 20 and shot_flag and shot_turn_flag:
            
            print("******* ROBOT SHOT! *********")

            detect_count_ball = 0
            detect_count_flag = 0
            ball_x_center = x0 + w0/2
            ball_y_center = y0 + h0/2
            
            # 정확한 타격이 이루어지는 범위까지 로봇을 왼쪽으로 이동시킴
            if ball_x_center < W_View_size * 0.75:
                TX_data(serial_port, 15) # Move left shortly
                time.sleep(2)

            # 도달 시 타격 후 변수 초기화
            else:
                TX_data(serial_port, 2) # SHOT
                time.sleep(3)

                # 변수 초기화
                shot_turn_flag = 0
                shot_flag = 0
                robot_condition = 0
                head_condition = 0
                flag_detected = 0
                flag_check = 0
                same_flag = 0
                saming = 0
                far_flag = 0
                ball_theta = 0
                ball_theta_index = -1

                # 헤드 상하 각도 초기화
                cur_theta = 30
                cur_theta_index = 3
                TX_data(serial_port, head_serial[cur_theta_index]) # Head up to 30'
                time.sleep(2)

                # 타격한 왼쪽 방향으로 로봇 회전
                TX_data(serial_port, 25)
                time.sleep(3)
                TX_data(serial_port, 25)
                time.sleep(3)

```


```python

        # ********** Action : 로봇이 골프공을 올바른 방향으로 타격하기 위해 회전  ***********
        # Shot possible but turn is needed (조건 : 공 감지, shot flag -> 결과 : 왼쪽으로 이동 후 회전 )
        elif detect_count_ball > 20 and shot_flag:
            
            print("*** ROBOT TURN BEFORE SHOT! ***")

            detect_count_ball = 0
            detect_count_flag = 0
            ball_x_center = x0 + w0/2
            ball_y_center = y0 + h0/2

            # 공이 시야 중심에 있지 않을 경우 이를 맞춤
            if not robot_condition:
                robot_condition = obj_x_centering(serial_port, ball_x_center, robot_condition, 0)
                continue

            # 공 왼쪽편으로 이동하여 회전할 수 있도록 로봇을 왼쪽으로 이동시킴
            if ball_x_center < W_View_size * 0.95:
                TX_data(serial_port, 15) #Move left shortly
                time.sleep(2)

            # 도달 시 왼쪽으로 4번, 시계 방향으로 90도 회전
            else:
                TX_data(serial_port, 15) # Move left shortly
                time.sleep(2)
                TX_data(serial_port, 15) # Move left shortly
                time.sleep(2) 
                TX_data(serial_port, 15) # Move left shortly
                time.sleep(2)
                TX_data(serial_port, 15) # Move left shortly
                time.sleep(2)
                
                TX_data(serial_port, 24) # CW 45'
                time.sleep(3)
                TX_data(serial_port, 24) # CW 45'
                time.sleep(3)
                TX_data(serial_port, 24) # CW 45'
                time.sleep(3)

                # 깃발을 바라볼 수 있도록 헤드 방향 지정
                cur_theta = 20
                cur_theta_index = 1
                TX_data(serial_port, head_serial[cur_theta_index])
                time.sleep(2)
                TX_data(serial_port, 28) # Head direction to left
                time.sleep(2)
                shot_turn_flag = 1 # shot_turn_flag is here

```


```python

        # ******* Action : 로봇과 공이 멀 때, 로봇이 골프공을 타격하기 좋은 근처로 이동  ********
        # Case : ball is far from robot (조건 : 공 감지, far_flag -> 결과 : 공 위치 근처로 이동)
        elif detect_count_ball > 20 and far_flag:

            print("*** ROBOT FAR FROM BALL! ***")

            detect_count_ball = 0
            detect_count_flag = 0
            ball_x_center = x0 + w0/2
            ball_y_center = y0 + h0/2

            # 공이 화면 Y 중심에 있지 않을 때        
            if not head_condition:
                head_condition = obj_y_centering(serial_port, ball_y_center, head_condition)
            
            # Y 중심이 일치할 경우 공과의 거리 계산 및 공 방향으로 회전
            else:
                # 거리 계산
                ball_dist = 33/math.tan(math.radians(cur_theta))
                print("Ball - robot distance = {:.2f} cm".format(ball_dist))
                
                # Case : 깃발 왼쪽, 공 오른쪽
                if flag_flag < 0 and ball_flag > 0:
                    direction_flag = 1 # robot direction이 공의 오른쪽으로 이동하도록

                # Case : 깃발 오른쪽, 공 왼쪽
                elif flag_flag > 0 and ball_flag < 0:
                    direction_flag = -1 # robot direction이 공의 왼쪽으로 이동하도록
                    
                robot_condition = obj_x_centering(serial_port, ball_x_center, robot_condition, direction_flag)
  
                
            # 보행할 준비가 완료된 상태(방향과 거리가 결정된 상태)
            if head_condition and robot_condition :
                
                print("**** ROBOT MOVE! ****")

                # 헤드 상하 각 초기화 및 이동
                cur_theta = 30
                cur_theta_index = 3
                TX_data(serial_port, 11) # Move In front
                time.sleep(ball_dist*DIST_CONSTANT*1.3)
                TX_data(serial_port, 26) # Basic Status
                time.sleep(3)
                TX_data(serial_port, 26) # Basic Status
                time.sleep(3)

                # 로봇을 공 방향으로 회전
                if direction_flag > 0 :
                    TX_data(serial_port, 22) # CCW 45'
                    time.sleep(3)
                    TX_data(serial_port, 22) # CCW 45'
                    time.sleep(3)
                    TX_data(serial_port, 22) # CCW 45'
                    time.sleep(3)
                    
                elif direction_flag < 0:
                    TX_data(serial_port, 24) # CW 45'
                    time.sleep(3)
                    TX_data(serial_port, 24) # CW 45'
                    time.sleep(3)
                    TX_data(serial_port, 24) # CW 45'
                    time.sleep(3)

                # 변수 초기화
                head_condition = 0
                robot_condition = 0
                head_dir = 0
                far_flag = 0    
                ball_flag = 0
                flag_flag = 0
                direction_flag = 0
                long = 0
                flag_detected = 0
                flag_check = 0
                saming = 0
                ball_theta = 0
                ball_theta_index = -1

                flag_x_center = -100
                flag_y_center = -100
                ball_x_center = -1000
                ball_y_center = -1000

                # 공 근처로 헤드 상하 각도 조절
                cur_theta = 70
                cur_theta_index = 11
                TX_data(serial_port, head_serial[cur_theta_index]) 
                time.sleep(2)


```


```python

        # *********** Action : 공과 깃발이 일렬 선상에 위치하도록 맞춤(1)  *************
        # Both detected during 20 frame but flag and ball are not in same line
        # 조건 : 둘 다 감지, 일렬 아닌 경우 -> 결과 : 일렬 정렬 시도
        elif detect_count_ball > 20 and detect_count_flag > 30 and not same_flag:
            
            print("****** ROBOT SORTING! ******")

            detect_count_ball = 0
            detect_count_flag = 0
            ball_x_center = x0 + w0/2
            ball_y_center = y0 + h0/2
            flag_x_center = x1 + w1/2
            flag_y_center = y1 + h1/2

            # 공이 깃발 사각형 안에 위치 -> 세레모니
            if (x1 < x0) and (x0+w0 < x1+w1) and (y1 < y0) and (y0+h0 < y1+h1):
                print("SUCCESS!")
                TX_data(serial_port, 119) #ceremony action
                time.sleep(3)
                break

            # 공과 flag 의 y좌표가 동일할 경우 공과 먼 경우로 설정
            if (abs(flag_y_center-ball_y_center) < PIXEL_Y_EPSILON):
                far_flag = 1 # far_flag is here
            
            # 공과 flag 의 x좌표가 많이 떨어져 있을 경우 공과 먼 경우로 설정
            elif (abs(flag_x_center-ball_x_center) > W_View_size*3/4):
                far_flag = 1

            else:
                far_flag = 0

            # 공이 멀지 않은 경우
            if not far_flag:

                print("******** ROBOT IS CLOSE WITH BALL **********")

                # 헤드 방향이 오른쪽일 경우 몸을 시계로 돌리고 중심을 보도록
                if head_dir > 0:
                    TX_data(serial_port, 19) # Robot rotation to cw 45'
                    time.sleep(3)
                    print("ROBOT DIRECTION CHANGE! Clockwise")
                    TX_data(serial_port, 21) #Head direction to center
                    time.sleep(3)
                    head_dir = 0
                    continue

                # 헤드 방향이 왼쪽일 경우 몸을 시계로 돌리고 중심을 보도록
                elif head_dir < 0:
                    TX_data(serial_port, 25) # Robot rotation to ccw 45'
                    time.sleep(3)
                    print("ROBOT DIRECTION CHANGE! CounterClockwise")
                    TX_data(serial_port, 21) #Head direction to center
                    time.sleep(3)
                    head_dir = 0
                    continue

                # 깃발 왼쪽, 공 오른쪽 -> 오른쪽으로 1보
                if ball_x_center - flag_x_center > PIXEL_X_EPSILON:
                    TX_data(serial_port, 20) #Move Right
                    time.sleep(3)

                # 공 왼쪽, 깃발 오른쪽 -> 왼쪽으로 1보
                elif ball_x_center - flag_x_center < -PIXEL_X_EPSILON:
                    TX_data(serial_port, 15) #Move Left
                    time.sleep(3)

                # Case : 일렬 정렬 완료
                else:
                    print("ROBOT LOCATION IS ACCEPTABLE - SORTING DONE")
                    same_flag = 1
                    robot_condition = 1
                    continue
                
                # 공이 화면 X 중심에 오도록 로봇 회전
                robot_condition = obj_x_centering(serial_port, ball_x_center, robot_condition, 0)
                robot_condition = 0

            # 공과의 거리가 먼 경우
            else:
                # far flag 조건문 수행을 위해 공과 깃발의 상대적 위치 지정
                # 공 왼쪽 깃발 오른쪽
                if ball_x_center < flag_x_center:
                    flag_flag = 1
                    ball_flag = -1

                # 공 오른쪽 깃발 왼쪽
                elif ball_x_center > flag_x_center:
                    flag_flag = -1
                    ball_flag = 1

                # 그 외
                else:
                    flag_flag = 0
                    ball_flag = 0

```


```python

        # ********** Action : 공과 깃발이 일렬 선상에 위치하도록 맞춤(2)  *************
        # Ball detected during 50 frame but flag and ball are not in same line
        # 조건 : 공만 감지, 거리가 가깝지만 일렬 아닌 경우 -> 결과 : 공과 깃발을 번갈아 가며 확인하여 일렬 정렬 시도
        elif detect_count_ball > 20 and cur_theta >= 60 and not same_flag:
            
            print("**** SAMING PROCESS START ****")

            saming = 1
            # ball 시야 각도 저장
            ball_flag = head_dir
            ball_theta = cur_theta
            ball_theta_index = cur_theta_index
            detect_count_flag = 0
            detect_count_ball = 0

            # 헤드 방향이 오른쪽 일 때
            if head_dir > 0:
                print("HEAD ON RIGHT")

                TX_data(serial_port, 21) #Head direction to center
                time.sleep(2)
                head_dir = 0
                    
                # 오른쪽으로 3보
                for _ in range(3):
                    TX_data(serial_port, 20) # Move Right
                    time.sleep(2)
                flag_check = 0
            
            # 헤드 방향이 왼쪽일 때
            elif head_dir < 0:
                print("HEAD ON LEFT")

                TX_data(serial_port, 21) #Head direction to center
                time.sleep(2)
                head_dir = 0
                    
                # 왼쪽으로 3보
                for _ in range(3):
                    TX_data(serial_port, 15) # Move Left
                    time.sleep(2)
                flag_check = 0
                
            # 아직 flag가 한번도 감지되지 않은 경우 -> flag 를 찾는다
            if not flag_check:
                print("flag not checked")
                # Head up to cur_theta
                cur_theta = 20
                cur_theta_index = 1
                TX_data(serial_port, head_serial[cur_theta_index])
                time.sleep(2)

            # 깃발 좌표가 check 된 경우
            else:
                print("****** FLAG CHEKED! ******")
                ball_x_center = x0 + w0/2
                ball_y_center = y0 + h0/2

                print("ball_x_center = ", ball_x_center)
                print("flag_x_center = ", flag_x_center)

                # 공이 로봇 정면에 위치할 경우 same flag
                if head_dir == 0 and ((W_View_size)/2 * 0.95) < ball_x_center < ((W_View_size)/2 * 1.05) and flag_x_center > W_View_size/2:
                    print("ROBOT LOCATION IS ACCEPTABLE")
                    same_flag = 1
                    continue

                # 공 오른쪽 , 깃발 왼쪽 -> 오른쪽으로 이동
                if ball_x_center - flag_x_center > PIXEL_X_EPSILON:
                    TX_data(serial_port, 13) #Move Right
                    time.sleep(2)
                    TX_data(serial_port, 4) # Turn left shortly
                    time.sleep(2)
                    flag_check = 0
                
                # 공 왼쪽, 깃발 오른쪽 -> 왼쪽으로 이동
                elif ball_x_center - flag_x_center < -PIXEL_X_EPSILON:
                    TX_data(serial_port, 14) #Move Left
                    time.sleep(2)
                    TX_data(serial_port, 6) # Turn right shortly
                    time.sleep(2)
                    flag_check = 0
                
                # 그 외 same flag
                else:
                    print("ROBOT LOCATION IS ACCEPTABLE")
                    same_flag = 1
                    continue


```


```python

        # ********** Action : 공이 감지되고 일렬 정렬된 상태에서는 공을 기준으로 로봇 방향을 맞춤  *************
        # Case : ball is detected and ball and ball and flag are in same line but robot direction must be changed
        # 조건 : 공 감지, 일렬 정렬, ball x not in center -> 결과 : x centering
        elif detect_count_ball > 20 and same_flag and not robot_condition:
            
            detect_count_ball = 0
            detect_count_flag = 0
            
            ball_x_center = x0 + w0/2
            ball_y_center = y0 + h0/2
            flag_x_center = x1 + w1/2
            flag_y_center = y1 + h1/2
            
            # 공과 너무 가까울 경우 후진
            if cur_theta >= 80:
                TX_data(serial_port, 12)
                time.sleep(3)
                TX_data(serial_port, 26)
                time.sleep(2)
                cur_theta = 60
                cur_theta_index = 9
                TX_data(serial_port, head_serial[cur_theta_index])
                time.sleep(2)
            
            robot_condition = obj_x_centering(serial_port, ball_x_center, robot_condition, 0)

```


```python


        # ********** Action : 공이 감지되고 일렬 정렬된 상태에서 거리 계산 후 보행 시작  *************
        # Ready to move in front but distance not calculated
        # 조건 : 공 감지, 일렬 정렬, ball x center -> 결과 : y centering 후 거리 계산 후 shot 또는 직진 보행
        elif detect_count_ball > 20 and same_flag and robot_condition and not shot_flag:
            detect_count_ball = 0
            detect_count_flag = 0

            # 공이 화면 Y 중심에 오지 않을 경우 이를 맞춘다.
            if not head_condition:
                ball_x_center = x0 + w0/2
                ball_y_center = y0 + h0/2
                head_condition = obj_y_centering(serial_port, ball_y_center, head_condition)

            # Y centering 완료 시 거리를 계산
            else:
                # Ready to move in front
                ball_dist = 33/math.tan(math.radians(cur_theta))
                print(ball_dist, "cm")

                # 충분히 가까울 경우 -> shot flag
                if ball_dist < 16:
                    shot_flag = 1 #shot_flag is here
                    print("SHOT POSSIBLE!")
                
                # 가깝지 않은 경우 -> 이동
                else:
                    # Move in front
                    TX_data(serial_port, 11)
                    time.sleep(ball_dist*DIST_CONSTANT)
                    TX_data(serial_port, 26)
                    time.sleep(2)
                    TX_data(serial_port, 26)
                    time.sleep(2)

                    # 이동한 위치에서 즉시 공을 보도록 머리를 내림
                    cur_theta = 70
                    cur_theta_index = 11
                    TX_data(serial_port, head_serial[cur_theta_index])
                    time.sleep(2)

                #Initializing 
                head_condition = 0
                robot_condition = 0
                ball_theta = 0
                ball_theta_index = -1

```


```python
        # ********** Action : 공이 처음 감지된 경우 깃발을 먼저 찾고, 이미 깃발 위치가 존재하는 경우 공 위치로 이동 *************
        # only ball detected and upper condition is not true
        # 조건 : 위 조건문들을 통과하지 않고 공만 감지된 경우 -> 결과 : flag를 찾도록 고개를 들어올리거나, 공 위치로 이동
        elif detect_count_ball > 20 and detect_count_flag < 10:
            
            print(" ****** FAR BALL DETECTED ! ******")
            
            # 공 위치 정보를 저장
            ball_flag = head_dir
            ball_theta = cur_theta
            ball_theta_index = cur_theta_index
            detect_count_flag = 0
            detect_count_ball = 0

            # 아직 flag가 한번도 감지되지 않은 경우 -> flag 를 찾는다
            if not flag_detected:
                # Head up to 20'
                cur_theta_index = 20
                cur_theta_index = 1
                TX_data(serial_port, head_serial[cur_theta_index])
                time.sleep(2)
                
                # Case : current head = left 45'
                if head_dir == -1:
                    TX_data(serial_port, 30) #Head to right 45'
                    time.sleep(2)
                    head_dir = 1
                # Case : current head = 0'        
                elif head_dir == 0:
                    TX_data(serial_port, 28) #Head to left 45'
                    time.sleep(2)
                    head_dir = -1
                # Case : current head = right 45'
                else:
                    head_dir = 0
                    TX_data(serial_port, 21) #Head to center
                    time.sleep(2)
                
                    cur_theta += 10
                    cur_theta_index += 2
                
                    cur_theta = cur_theta % 100
                    cur_theta_index = cur_theta_index % len(head_serial)
                
                    TX_data(serial_port, head_serial[cur_theta_index])
                    time.sleep(2)
                    non_detect_count = 0
                    print("CUR_HEAD_DIR = ", head_dir)

            # flag 가 한번이라도 탐지된 경우 해당 방향을 이용하여 일렬 정렬이 가능한 위치로 움직인다.
            else:
                detect_count_ball = 0
                detect_count_flag = 0

                ball_x_center = x0 + w0/2
                ball_y_center = y0 + h0/2
                        
                # 공이 Y 중심에 있지 않을 경우 위치를 맞춘다
                if not head_condition:
                    head_condition = obj_y_centering(serial_port, ball_y_center, head_condition)

                # 공과의 거리를 계산하고 이동
                else:
                    ball_dist = 33/math.tan(math.radians(cur_theta))
                    print("Distance with ball = {:.2f} cm".format(ball_dist))

                    # 깃발 왼쪽, 공 왼쪽 -> robot direction이 공의 오른쪽으로 이동하도록
                    if flag_flag < 0 and ball_flag < 0 :
                        direction_flag = 1
                    # 깃발 왼쪽, 공 가운데 -> robot direction이 공의 오른쪽으로 이동하도록
                    elif flag_flag < 0 and ball_flag == 0:
                        direction_flag = 1
                    # 깃발 왼쪽, 공 오른쪽 -> robot direction이 공의 오른쪽으로 이동하도록
                    elif flag_flag < 0 and ball_flag > 0:
                        direction_flag = 1
                        long = 1
                    # 깃발 오른쪽, 공 왼쪽 -> robot direction이 공의 왼쪽으로 이동하도록
                    elif flag_flag > 0 and ball_flag < 0:
                        direction_flag = -1
                        long = 1
                    # 깃발 오른쪽, 공 가운데 -> robot direction이 공의 왼쪽으로 이동하도록
                    elif flag_flag > 0 and ball_flag == 0:
                        direction_flag = -1
                    # 깃발 오른쪽, 공 오른쪽 -> robot direction이 공의 왼쪽으로 이동하도록
                    elif flag_flag > 0 and ball_flag > 0:
                        direction_flag = -1
                    # 깃발 가운데, 공 왼쪽 -> robot direction이 공의 왼쪽으로 이동하도록
                    elif flag_flag == 0 and ball_flag < 0:
                        direction_flag = -1
                    # 깃발 가운데, 공 가운데 -> robot direction이 공의 정면으로 이동하도록
                    elif flag_flag == 0 and ball_flag == 0:
                        direction_flag = 0
                    # 깃발 가운데, 공 오른쪽 -> robot direction이 공의 오른쪽으로 이동하도록
                    elif flag_flag == 0 and ball_flag > 0:
                        direction_flag = 1
                    # 그 외 
                    else:
                        print("Error! Direction flag error")
                        direction_flag = 0

                    if direction_flag > 0 :
                        print("Robot move to right of ball")
                    elif direction_flag < 0 :
                        print("Robot move to left of ball")
                    else:
                        print("Robot move to in front of ball") 

                    robot_condition = obj_x_centering(serial_port, ball_x_center, robot_condition, direction_flag)

                    # 보행할 준비가 완료된 상태(방향과 거리가 결정된 상태)
                    if head_condition and robot_condition :
                        # 공 위치로 이동
                        cur_theta = 30
                        cur_theta_index = 3
                        TX_data(serial_port, 11) # 정면 연속 보행
                        if long:
                            time.sleep(ball_dist*DIST_CONSTANT*1.6)
                            print("Long")
                        else:
                            time.sleep(ball_dist*DIST_CONSTANT*1.4)
                        TX_data(serial_port, 26) # 정지
                        time.sleep(3)
                        TX_data(serial_port, 26) # 정지
                        time.sleep(3)

                        # 공 오른쪽 편으로 이동했을 경우
                        if direction_flag > 0 :
                            TX_data(serial_port, 22) # ccw 45'
                            time.sleep(3)
                            TX_data(serial_port, 22) # ccw 45'
                            time.sleep(3)
                            TX_data(serial_port, 22) # ccw 45'
                            time.sleep(3)

                        elif direction_flag < 0:
                            TX_data(serial_port, 24) # cw 45'
                            time.sleep(3)
                            TX_data(serial_port, 24) # cw 45'
                            time.sleep(3)
                            TX_data(serial_port, 24) # cw 45'
                            time.sleep(3)

                        #Initializing 
                        head_condition = 0
                        robot_condition = 0

                        head_dir = 0
                        ball_flag = 0
                        flag_flag = 0
                        direction_flag = 0
                        flag_detected = 0
                        flag_check = 0
                        saming = 0
                        long = 0

                        ball_theta = 0
                        ball_theta_index = -1

                        flag_x_center = -100
                        flag_y_center = -100
                        ball_x_center = -1000
                        ball_y_center = -1000

                        # 바로 공 근처를 보도록 지정
                        cur_theta = 70
                        cur_theta_index = 11
                        TX_data(serial_port, head_serial[cur_theta_index])
                        time.sleep(2)
```


```python

        # ********** Action : shot turn 상태에서 깃발 위치를 기준으로 shot 방향을 정확하게 맞춤 *************
        # flag detected and shot_turn_flag, then control the robot direction 
        # 조건 : 깃발이 감지되었고 shot_turn_flag 상태인 경우 -> 결과 : shot에 적절한 flag 위치까지 로봇 각도를 조절 
        elif detect_count_flag > 20 and shot_turn_flag:
            detect_count_flag = 0
            detect_count_ball = 0

            flag_x_center = x1+w1/2
            flag_y_center = y1+h1/2

            # flag가 화면 Y 중심에 오지 않을 경우 이를 맞춤
            if not head_condition:
                head_condition = obj_y_centering(serial_port, flag_y_center, head_condition)
                continue

            # 깃발 거리 계산
            flag_dist = 33/math.tan(math.radians(cur_theta))
            print("Distance with flag = {:.2f} cm".format(flag_dist))
            
            # 거리가 4cm 이상이고, 정확한 위치가 아닐 경우
            if (flag_dist > 40) and (x1+w1 > W_View_size*(3/16)) :
                TX_data(serial_port, 6) # Robot turn left shortly
                time.sleep(2)
            
            # 그 외 경우에 대해서는 즉시 shot 과정 수행
            else:
                TX_data(serial_port, 21)  #Head direction to center
                time.sleep(2)

                # 공을 다시 바라보도록 헤드를 내려줌
                cur_theta = 70
                cur_theta_index = 11
                TX_data(serial_port, head_serial[cur_theta_index])
                time.sleep(2)

        # ********** Action : 고개를 번갈아가며 공과 깃발이 위치를 일치 시킴 *************
        # flag detected during sorting process, then head down to ball and compare with flag coordinate
        # 조건 : 일렬 정렬 과정 중에 flag가 보일 경우 -> 결과 : 다시 고개를 내려 로봇의 위치를 옮김
        elif detect_count_flag > 20 and saming:
            print("flag detected! during saming process")
            
            flag_flag = head_dir
            detect_count_flag = 0
            detect_count_ball = 0

            # flag가 왼쪽 방향에서 탐지된 경우 -> 헤드를 중심, 몸을 좌측으로 돌린다
            if head_dir == -1:
                TX_data(serial_port, 21) #Head to center
                time.sleep(2)
                TX_data(serial_port, 25) #body to left 45'
                time.sleep(2)
                head_dir = 0
                # 다시 공을 보도록 설정
                cur_theta = ball_theta
                cur_theta_index = ball_theta_index
                TX_data(serial_port, head_serial[cur_theta_index])
                time.sleep(2)

            # flag가 오른쪽 방향에서 탐지된 경우 -> 헤드를 중심, 몸을 우측으로 돌린다      
            elif head_dir == 1:
                TX_data(serial_port, 21) #Head to center
                time.sleep(2)
                TX_data(serial_port, 19) #body to right 45'
                time.sleep(2)
                head_dir = 0
                # 다시 공을 보도록 설정
                cur_theta = ball_theta
                cur_theta_index = ball_theta_index
                TX_data(serial_port, head_serial[cur_theta_index])
                time.sleep(2)
            
            # 현재 각도가 공을 보는 각도보다 위쪽일 경우 -> 헤드를 공을 보던 각도로 내린다
            elif cur_theta < ball_theta:
                flag_check = 1
                flag_x_center = x1 + w1/2 # flag x 좌표 저장
                cur_theta = ball_theta
                cur_theta_index = ball_theta_index
                TX_data(serial_port, head_serial[cur_theta_index])
                time.sleep(2)
                flag_y_center = y1 + h1/2 # flag y 좌표 저장

            # 위 경우가 적용되지 않았으나 flag가 중심에서 탐지된 경우 -> 헤드를 5도씩 내린다
            else:
                flag_check = 1
                flag_x_center = x1 + w1/2 # flag x 좌표 저장
                cur_theta += 5
                cur_theta_index +=1
                TX_data(serial_port, head_serial[cur_theta_index])
                time.sleep(2)
                flag_y_center = y1 + h1/2 # flag y 좌표 저장

```


```python            

        # ********** Action : 깃발이 처음 관측되었을 때 정보를 저장하고 다시 공을 찾는다. *************
        # flag detected during sorting process, then head down to ball and compare with flag coordinate
        # 조건 : flag 가 처음으로 detect 된 경우 -> 결과 : flag 좌표 정보를 저장
        elif detect_count_flag > 20 and detect_count_ball < 10:
            flag_detected = 1
            flag_flag = head_dir
            detect_count_flag = 0
            detect_count_ball = 0
            flag_x_center = x1 + w1/2

            # 공 정보가 이미 존재할 경우 해당 상태로 돌아감
            if ball_theta:
                cur_theta = ball_theta
                cur_theta_index = ball_theta_index
                TX_data(serial_port, 21) #Head to center
                head_dir = 0
                time.sleep(2)
                # Head down to ball_theta
                TX_data(serial_port, head_serial[cur_theta_index])
                time.sleep(2)
                continue
            
            # Case : current head = left 45'
            if head_dir == -1:
                TX_data(serial_port, 30) #Head to right 45'
                time.sleep(2)
                head_dir = 1

            # Case : current head = center     
            elif head_dir == 0:
                TX_data(serial_port, 28) #Head to left 45'
                time.sleep(2)
                head_dir = -1
                    
            # Case : current head = right 45'
            else:
                head_dir = 0
                TX_data(serial_port, 21) #Head to center
                time.sleep(2)
            
                cur_theta += 10
                cur_theta_index += 2
            
                cur_theta = cur_theta % 100
                cur_theta_index = cur_theta_index % len(head_serial)
            
                TX_data(serial_port, head_serial[cur_theta_index])
                time.sleep(3)
                print("CUR_HEAD_DIR = ", head_dir)

```


```python

        # ********** Action : shot_turn_flag 상태에서 아무것도 관측되지 않을 때 원래 상태로 돌린다. *************
        # nothing detected when shot_turn_flag = 1, then change the head direction 
        # 조건 : shot_turn_flag에서 아무것도 탐지되지 않는 경우 -> 결과 : 다시 공을 바라보도록 돌려줌
        elif non_detect_count > 20 and shot_turn_flag:
            detect_count_flag = 0
            detect_count_ball = 0
            non_detect_count = 0

            TX_data(serial_port, 21)  #Head direction to center
            time.sleep(2)

            # 공을 다시 바라보도록 헤드를 내려줌
            cur_theta = 70
            cur_theta_index = 11
            TX_data(serial_port, head_serial[cur_theta_index])
            time.sleep(2)

```


```python

        # ********** Action : 아무것도 관측되지 않을 때 object를 탐색한다 *************
        # nothing detected, then change the head direction 
        # 조건 : 아무것도 탐지되지 않는 경우 -> 결과 : flag 좌표 정보를 저장
        elif non_detect_count > 20:

            detect_count_flag = 0
            detect_count_ball = 0
            non_detect_count = 0

            # Case : current head = left 45'
            if head_dir == -1:
                TX_data(serial_port, 30) #Head to right 45'
                time.sleep(2)
                head_dir = 1
                    
            elif head_dir == 0:
                TX_data(serial_port, 28) #Head to left 45'
                time.sleep(2)
                head_dir = -1
                    
            # Case : current head = right 45'
            else:
                head_dir = 0
                TX_data(serial_port, 21) #Head to center
                time.sleep(2)
            
                cur_theta += 10
                cur_theta_index += 2
            
                cur_theta = cur_theta % 100
                cur_theta_index = cur_theta_index % len(head_serial)
            
                TX_data(serial_port, head_serial[cur_theta_index])
                time.sleep(2)
                print("CUR_HEAD_DIR = ", head_dir)
        

```

조건 분기 종료, frame check & key interrupt check

```python


        # ********* 로봇 동작 조건 분기 종료 *************



        # ***** 종료 명령 , fps 출력 등 후 처리 과정 ******

        Frame_time = (clock() - old_time) * 1000.

        old_time = clock()

        f_count += 1
           

        if f_count >= 20: # Fast operation 

            print(" " + str(W_View_size) + " x " + str(H_View_size) + " =  %.1f fps" % (1000/Frame_time))
            print("cur_theta = ", cur_theta, "cur_head_dir = ", head_dir)
            f_count = 0
            #temp = Read_RX

            pass

            
        if View_select == 1: # Debug

            if msg_one_view > 0:

                msg_one_view = msg_one_view + 1

                cv2.putText(frame, "SAVE!", (50, int(H_View_size / 2)),

                            cv2.FONT_HERSHEY_PLAIN, 5, (255, 255, 255), thickness=5)

                

                if msg_one_view > 10:

                    msg_one_view = 0                

                                

            draw_str2(frame, (3, 15), 'X: %.1d, Y: %.1d, Area: %.1d' % (X_255_point, Y_255_point, Area))

            draw_str2(frame, (3, H_View_size - 5), 'View: %.1d x %.1d Time: %.1f ms  Space: Fast <=> Video and Mask.'

                      % (W_View_size, H_View_size, Frame_time))

                      

            #------mouse pixel hsv -------------------------------

            mx2 = mx

            my2 = my

            if mx2 < W_View_size and my2 < H_View_size:

                pixel = hsv[my2, mx2]

                set_H = pixel[0]

                set_S = pixel[1]

                set_V = pixel[2]

                pixel2 = frame[my2, mx2]

                if my2 < (H_View_size / 2):

                    if mx2 < (W_View_size / 2):

                        x_p = -30

                    elif mx2 > (W_View_size / 2):

                        x_p = 60

                    else:

                        x_p = 30

                    draw_str2(frame, (mx2 - x_p, my2 + 15), '-HSV-')

                    draw_str2(frame, (mx2 - x_p, my2 + 30), '%.1d' % (pixel[0]))

                    draw_str2(frame, (mx2 - x_p, my2 + 45), '%.1d' % (pixel[1]))

                    draw_str2(frame, (mx2 - x_p, my2 + 60), '%.1d' % (pixel[2]))

                else:

                    if mx2 < (W_View_size / 2):

                        x_p = -30

                    elif mx2 > (W_View_size / 2):

                        x_p = 60

                    else:

                        x_p = 30

                    draw_str2(frame, (mx2 - x_p, my2 - 60), '-HSV-')

                    draw_str2(frame, (mx2 - x_p, my2 - 45), '%.1d' % (pixel[0]))

                    draw_str2(frame, (mx2 - x_p, my2 - 30), '%.1d' % (pixel[1]))

                    draw_str2(frame, (mx2 - x_p, my2 - 15), '%.1d' % (pixel[2]))

            #----------------------------------------------

            

            cv2.imshow('mini CTS5 - Video', frame )

            cv2.imshow('mini CTS5 - Mask', mask)



        key = 0xFF & cv2.waitKey(1)

        

        if key == 27:  # ESC  Key

            break

        elif key == ord(' '):  # spacebar Key

            if View_select == 0:

                View_select = 1

            else:

                View_select = 0

        elif key == ord('s') or key == ord('S'):  # s or S Key:  Setting valus Save

            hsv_setting_save()

            msg_one_view = 1



    # cleanup the camera and close any open windows

    receiving_exit = 0

    time.sleep(0.5)

    

    camera.release()

    cv2.destroyAllWindows()

```
			  
			  

  


