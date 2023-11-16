## <div align="center">2023 제 21회 임베디드 소프트웨어 경진대회 지능형 휴머노이드 부문 </div>
🚀 지능형 휴머노이드 로봇을 이용한 로봇 골프 경기 수행

## <div align="center">Team 서계</div>
🌟 Team Leader 이창재 (서강대학교 기계공학과 19)

🌟 Team member 강정훈 (서강대학교 기계공학과 19)

🌟 Team member 김기훈 (서강대학교 기계공학과 / 컴퓨터공학과 19)

🌟 Team member 이도헌 (서강대학교 기계공학과 19)

## <div align="center">Video</div>

🚀 See our operation video on the YouTube

[Youtube_link](https://www.youtube.com/watch?si=oTYd6pSQUsP_0D2D&v=IUE--1FHvtU&feature=youtu.be)


<p align="center"><img width="800" src=""></p>


## <div align="center">Competition Summary</div>

🚀 파 3
-


🚀 파 3

- [Our Roboflow Data Set](https://app.roboflow.com/sgme/classify-pet-and-can/4)

- yolov5m 모델을 이용하여 [best.pt](https://drive.google.com/file/d/1xFNFxLWNwAg3CrGFe8cWR2mcu1oUs7Ly/view?usp=sharing)를 제작


🚀 Take Image using WebCam - terminal (fswebcam)


```python
import os
picture = "fswebcam --no-banner --set brightness=60% Images/test1.jpg"
os.system(picture)
```

- First Image

<p align="center"><img width="800" src="https://file.notion.so/f/s/0f7580ce-3b6c-47f7-9cd3-3d457e8ed00f/test1.jpg?id=5e43ff6b-9970-4ced-9b5b-d9e23735aa74&table=block&spaceId=89f4f652-5ebd-4c52-8a2d-be12a0e49dda&expirationTimestamp=1699524000000&signature=TXT6n3OaYCqfCuVTcafm9re_H4Edqcu0jchXARqi9cs&downloadName=test1.jpg"></p>


🚀 Analysis the Image - customized yolov5 (detect.py and best.pt)



- Final Image

<p align="center"><img width="800" src="https://file.notion.so/f/s/2e627ccb-23bc-4ef4-89f9-811433130be3/test1_analysis.jpg?id=56d6ca8a-a8bb-4e76-9503-9793160dfbec&table=block&spaceId=89f4f652-5ebd-4c52-8a2d-be12a0e49dda&expirationTimestamp=1699524000000&signature=DZojfAjdkM-3_-DnIqhSLHS_LeyD_1dM2AXQ4rR-5Ls&downloadName=test1_analysis.jpg"></p>
 

- Object 좌표값 데이터를 output.txt에 저장

<p align="center">detect.py</p>

```python
if len(det):
    # Rescale boxes from img_size to im0 size
    det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
                
    #Customized part : Save det(coordinate info) to output.txt
    for i in range(len(det)):
        print(float(det[i][0]))
        print(float(det[i][1]))
        print(float(det[i][2]))
        print(float(det[i][3]))
        print(float(det[i][5]))
```

<p align="center">Terminal</p>

```bash
# Modify file location
python3 yolov5/detect.py > yolov5/output.txt --weights yolov5/best.pt --img 640 --conf 0.4 --source Images/test1.jpg
```
    
🚀 Make Object Map using 2D Matrix from output.txt Data - Python3 Code(file_read.py) 

<p align="center">file_read.py</p>

```python
#Make 2D Array using output.txt (object location data)
def map():
        f = open("/home/sgme/yolov5/output.txt", 'r') # Modify file location
        f.readline()
        object_list = []
        lines = f.read().splitlines()
        temp_list=[]
        count = 0
        for i in lines:
            if count == 5:
                count = 0
                object_list.append(temp_list)
                temp_list = []
            temp_list.append(round(float(i)))
            count+=1
        object_list.append(temp_list) #last object
        f.close

        # Make result_map to find efficient route 
        # width * height = 1020px * 720px, row x column = 24 x 34 2D MATRIX
        # Append 2 more row to each top and bottom
        # The top and bottom 2 row area will be trash area
        rows = 28
        cols = 34
        
        # Empty Area = ('e', 0)
        result_map = [[('e',0) for j in range(cols)] for i in range(rows)]
        
        # PET Area = ('p', k) k is object number
        # CAN Area = ('c', k) k is object number
        for m in range(0, rows-4):
            y_m = m*(720/(rows-4)) + (720/(2*(rows-4)))
            for n in range(0, cols):
                x_m = n*(1020/cols) + (1020/(2*cols))
                for k in range(len(object_list)):
                    if (x_m > object_list[k][0]) and (x_m < object_list[k][2]) and (y_m > object_list[k][1]) and (y_m < object_list[k][3]):
                        if object_list[k][4] == 1:
                            result_map[m+2][n] = ('p',k+1)
                            break
                        elif object_list[k][4] == 0:
                            result_map[m+2][n] = ('c',k+1)
                            break
        return result_map    
```

🚀 Find the Most Efficient Way - Python3 Code (dijkstra.py)

<details open>
<summary>dijkstra.py</summary>

```python
import heapq
import numpy as np
global INF
INF = 2550

def object_boundary(maze, row_index, col_index):
    m = len(maze) #Row of maze
    n = len(maze[0]) #Column of maze
    #valid index
    for dy, dx in [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]:
            next_y, next_x = row_index + dy, col_index + dx
            # Case : Index out of range
            if next_y < 0 or next_y >= m or next_x < 0 or next_x >= n:
                continue
            else :
              maze[next_y][next_x] = -1

def object_boundary_deletion(maze, row_index, col_index):
    m = len(maze) #Row of maze
    n = len(maze[0]) #Column of maze
    #valid index
    for dy, dx in [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]:
            next_y, next_x = row_index + dy, col_index + dx
            # Case : Index out of range
            if next_y < 0 or next_y >= m or next_x < 0 or next_x >= n:
                continue
            else :
              maze[next_y][next_x] = 0

#Function : Dijkstra - Calculate distance of all point from start point
def dijkstra(maze, start):
    m = len(maze) #Row of maze
    n = len(maze[0]) #Column of maze
    dist = np.array([[INF] * n for _ in range(m)]) #Fill the maze with INF
    dist[start[0]][start[1]] = 0 #Set the start point as 0
    heap = [(0, start)] #Initial heap which has start point and initial cost
    
    #Until heap element exists
    while heap:
        cost, pos = heapq.heappop(heap) #Pop the element from heap
        # Case : The smaller value of cost already exists
        if dist[pos[0]][pos[1]] < cost:
            continue
        
        #N, NE, E, SE, S, SW, W, NW -> 8 direction Searching from current position
        for dy, dx in [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]:
            #Next position
            next_y, next_x = pos[0] + dy, pos[1] + dx
            # Case : Index out of range
            if next_y < 0 or next_y >= m or next_x < 0 or next_x >= n:
                continue
            # Case : Obstacle
            if maze[next_y][next_x] == -1:
                continue
            # Cost of next position
            if (dy == 0 or dx == 0):
                next_cost = cost + 10 
            else : 
                next_cost = cost + 14
            # Case : Update cost of next position to minimum cost
            if next_cost < dist[next_y][next_x]:
                dist[next_y][next_x] = next_cost
                #Update heap
                heapq.heappush(heap, (next_cost, (next_y, next_x)))
    return dist

#Function : Find shortest path from start point to target point
def find_shortest_path(maze, start, target):
    object_boundary_deletion(maze, start[0], start[1])
    object_boundary_deletion(maze, target[0], target[1])
    dist = dijkstra(maze, start)
    m = len(maze) #Row of maze
    n = len(maze[0]) #Column of maze
    path = [(target[0], target[1])] #stack of path
    #Until start point
    while path[-1] != start:
        y, x = path[-1]
        min_dist = dist[y][x] #Current point
        for dy, dx in [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]:
            next_y, next_x = y + dy, x + dx
            # Case : Index out of range
            if next_y < 0 or next_y >= m or next_x < 0 or next_x >= n:
                continue
            # Case : Current point > Precede point -> update
            if dist[next_y][next_x] < min_dist:
                min_dist = dist[next_y][next_x]
                next_pos = (next_y, next_x)
        path.append(next_pos)
    object_boundary(maze, start[0], start[1])
    object_boundary(maze, target[0], target[1])
    return path

#Function : Make can and PET position index list from given map
def find_pet_can_list(given_map):
    pet_list = []
    can_list = []
    for i in range(len(given_map)):
        for j in range(len(given_map[0])):
            if given_map[i][j][0] == 'p':
                pet_list.append((i, j))
            elif given_map[i][j][0] == 'c':
                can_list.append((i, j))
    return pet_list, can_list

#Function : Make map which has shortest path from PET and Can to trash area
def find_shortest_classfy_path(maze, final_map, pet_list, can_list, start):
    #PET
    m = len(maze)
    n = len(maze[0])
    for i, j in pet_list:
        temp_maze = maze
        object_boundary_deletion(temp_maze, i, j)
        dist = dijkstra(temp_maze, (i, j))
        min_pet = INF
        #Find minimum distance from PET to trash area
        for k in range(len(maze[0])):
            if dist[0][k] < min_pet:
                min_pet = dist[0][k]
        final_map[[i],[j]] = min_pet
        object_boundary(temp_maze, i, j)
    #CAN
    for i, j in can_list:
        temp_maze = maze
        object_boundary_deletion(temp_maze, i, j)
        dist = dijkstra(temp_maze, (i, j))
        min_can = INF
        #Find minimum distance from can to trash area
        for k in range(len(maze[0])):
            if dist[len(maze)-1][k] < min_can:
                min_can = dist[len(maze)-1][k]
        final_map[[i],[j]] = min_can
        object_boundary(temp_maze, i, j)
    #Approach
    for i, j in pet_list+can_list:
        #Refactoring needed for efficiency
        temp_maze = maze
        temp_maze[i][j] = 0
        object_boundary_deletion(temp_maze, i, j)
        dist = dijkstra(temp_maze, start)
        final_map[[i],[j]] += dist[[i],[j]] 
        object_boundary(temp_maze, i, j)
    return final_map

#Function : Find most efficient index from fin al map
def find_most_efficient_index(map):
    min = INF
    min_index = (0, 0)
    for i in range(len(map)):
        for j in range(len(map[0])):
            if map[i][j] != 0 and map[i][j] < min:
                min = map[i][j]
                min_index = (i, j)
    return min_index

#Function : Classify trash using efficient path
def classify_trash(given_map, start):
    m = len(given_map)
    n = len(given_map[0]) 
    maze = np.zeros((m,n))
    final_map = np.zeros((m,n))
    
    pet_list, can_list = find_pet_can_list(given_map)
    print("REMAIN PET INDEX : ", pet_list, "\n")
    print("REMAIN CAN INDEX : ",can_list, "\n")   
    for i, j in pet_list+can_list:
        maze[i][j] = -1
        object_boundary(maze, i, j)

    final_map = find_shortest_classfy_path(maze, final_map, pet_list, can_list, start)
    print("FINAL MAP: \n", final_map, "\n")

    target = find_most_efficient_index(final_map)

    for i, j in pet_list+can_list:
        maze[i][j] = -1

    object_boundary_deletion(maze, target[0], target[1])
    dist = dijkstra(maze, target)
    final_path = []

    if given_map[target[0]][target[1]][0] == 'p':
        min_pet = INF
        min_pet_col = 0
        for i in range(len(maze[0])):
            if dist[0][i] < min_pet:
                min_pet = dist[0][i]        
                min_pet_col = i
        final_path = find_shortest_path(maze, target, (0, min_pet_col))
    else :
        min_can = INF
        min_can_col = 0
        for i in range(len(maze)):
            if dist[len(maze)-1][i] < min_can:
                min_can = dist[len(maze)-1][i]        
                min_can_col = i
        final_path = find_shortest_path(maze, target, (len(maze)-1, min_can_col))
        
    temp_path = find_shortest_path(maze, start, target)[1:]
    final_path += temp_path
    start = final_path[0]   
    return start, final_path, target, pet_list, can_list
    
#Main
def main(given_map, start_row, start_col):
    start = (start_row, start_col)
    print("START AT :", start, "\n")
    start, path, target, pet_list, can_list = classify_trash(given_map, start)
    end_row = start[0]
    end_col = start[1]
    target_number = given_map[target[0]][target[1]][1]
    target_object = given_map[target[0]][target[1]][0]
    
    print(target_object, target_number)
    print("RESULT INDEX: ")
    print(path, "\n")

    move_order = [[0,0,0]]

    row_1, col_1 = path.pop()
    while(path):
        row_2, col_2 = path.pop()
        dy = row_2 - row_1
        dx = col_2 - col_1
        #N
        if dy < 0 and dx == 0:
            new = [2, 0, 0]
        #NE
        elif dy < 0 and dx > 0:
            new = [2, 2, 1]
        #E
        elif dy == 0 and dx > 0:
            new = [0, 2, 2]
        #SE
        elif dy > 0 and dx > 0:
            new = [2, 2, 3]
        #S
        elif dy > 0 and dx == 0:
            new = [2, 0, 4]
        #SW
        elif dy > 0 and dx < 0:
            new = [2, 2, 5]
        #W
        elif dy == 0 and dx < 0:
            new = [0, 2, 6]
        #NW
        else :
            new = [2, 2, 7]

        if (move_order):
            old = move_order[-1]    
            if (old[2] == new[2]) and (old[0] + new[0] < 16) and (old[1] + new[1] < 16):
                move_order[-1][0] += new[0]
                move_order[-1][1] += new[1]
            else:
                move_order.append(new)
        else :
            move_order.append(new)
        row_1, col_1 = row_2, col_2
            
              
    for i in range(len(move_order)):
        move_order[i] = tuple(move_order[i])

    for i in range(len(given_map)):
        for j in range(len(given_map[0])):
            if (given_map[i][j] == (target_object, target_number) and (target_object, target_number) != ('e',0)) :
                given_map[i][j] = ('e', 0)
                print("delete ", i, j)
    print("RESULT MOVE ORDER : \n", move_order, "\n")
 
    return move_order, given_map, end_row, end_col, pet_list, can_list
```
 
</detail>
 

🚀 Send the Control Order to Arduino Nano by Serial Module - Python3 pyserial
 
```python
import serial

#Serial Setup
py_serial = serial.Serial(port = '/dev/ttyUSB0', baudrate = 9600)

#Get move_order from dijkstra	
	
while len(move_order):
	py_serial.write(move_order.pop(0))
	print("pop")
	time.sleep(1.0) #delay time decided by velocity and distance
```		
 
 
🚀 Recieve the Control Order in the Arduino - Arduino Serial (motor_control.ino)

	
<p align = "center">motor_control.ino</p>

```cpp
// pin
#define X_dirPin 2
#define X_stepPin 3
#define Y_dirPin 4
#define Y_stepPin 5
#define Z_dirPin 6
#define Z_stepPin 7


// const
float sPR = 3200;
int Head_dir = 0;

void setup() {
  Serial.begin(9600);
  pinMode(X_dirPin,OUTPUT);
  pinMode(Y_dirPin,OUTPUT);
  pinMode(X_stepPin,OUTPUT);
  pinMode(Y_stepPin,OUTPUT);
  pinMode(Z_dirPin,OUTPUT);
  pinMode(Z_stepPin,OUTPUT);
}

void move(int x_distance, int y_distance, int Head){

  float dx, dy;
  dx = (float)x_distance/80; //원래 100
  dy = (float)y_distance/80;

  int Head_new = Head;
  
  int angle = Head_new - Head_dir;
    

  if(angle !=0){
    if(abs(angle)<4){
      if(angle>0){
          digitalWrite(Z_dirPin, HIGH);
      }
      else{
          digitalWrite(Z_dirPin, LOW);
      }
      Head_dir = Head_new;
      for(int i=0; i<sPR*abs(angle)/8; i++){
        digitalWrite(Z_stepPin,HIGH);
        delayMicroseconds(200);
        digitalWrite(Z_stepPin,LOW);
        delayMicroseconds(200);
      }
    }
    else{
      if(angle>0){
        digitalWrite(Z_dirPin,LOW); 
      }
      else{
        digitalWrite(Z_dirPin,HIGH);
      }
        
      Head_dir = Head_new;

      for(int i=0; i<sPR*(8-abs(angle))/8; i++){
        digitalWrite(Z_stepPin,HIGH);
        delayMicroseconds(200);
        digitalWrite(Z_stepPin,LOW);
        delayMicroseconds(200);
      }    
    }
  }
       
    switch(Head_new){
      case 0:
        digitalWrite(X_dirPin, HIGH);
        digitalWrite(Y_dirPin, LOW);
        break;
      case 1:
        digitalWrite(X_dirPin, HIGH);
        digitalWrite(Y_dirPin, LOW);
        break;
      case 2:
        digitalWrite(X_dirPin, HIGH);
        digitalWrite(Y_dirPin, LOW);
        break;
      case 3:
        digitalWrite(X_dirPin, HIGH);
        digitalWrite(Y_dirPin, HIGH);
        break;
      case 4:
        digitalWrite(X_dirPin, LOW);
        digitalWrite(Y_dirPin, HIGH);
        break;
      case 5:
        digitalWrite(X_dirPin, LOW);
        digitalWrite(Y_dirPin, HIGH);
        break;
      case 6:
        digitalWrite(X_dirPin, LOW);
        digitalWrite(Y_dirPin, HIGH);
        break;
      case 7:
        digitalWrite(X_dirPin, LOW);
        digitalWrite(Y_dirPin, LOW);
        break;
        
    }
  
    // X,Y 거리, 속도
    for(int i=0, j=0; (i<100000*dx)||(j<100000*dy);i++,j++){
      if(i<100000*dx){
        digitalWrite(X_stepPin,HIGH);
        delayMicroseconds(30);
        digitalWrite(X_stepPin,LOW);
        delayMicroseconds(30);
      }
      if((j<100000*dy)){
        digitalWrite(Y_stepPin,HIGH);
        delayMicroseconds(30);
        digitalWrite(Y_stepPin,LOW);
        delayMicroseconds(30);
      }
    }
}

void loop() {
  int data[3];
  if(Serial.available() >= 3){
    for (int i = 0; i < 3; i++){
      data[i] = Serial.read();
    }
    int x_d = data[1];
    int y_d = data[0];
    int z_dir = data[2];
    move(x_d, y_d, z_dir);
  }
  Serial.println("done");
}
```
	
🚀 Integration Code - Python3 Code (recycle.py)

<p align = "center"> recycle.py </p>

```python
import os
import time
import serial
import cv2 as cv
import file_read
import dijkstra

#Serial Setup
py_serial = serial.Serial(port = '/dev/ttyUSB0', baudrate = 9600)

#Camera Setup
picture = "fswebcam --no-banner --set brightness=60% Images/test1.jpg"
os.system(picture)
img = cv.imread("Images/test1.jpg", cv.IMREAD_COLOR)
resize_img = cv.resize(img, (1020,720), interpolation=cv.INTER_AREA)
cv.imwrite("Images/test1.jpg", resize_img)

#Image Analysis
yolo = "python3 yolov5/detect.py > yolov5/output.txt --weights yolov5/best.pt --img 640 --conf 0.4 --source Images/test1.jpg"
os.system(yolo)
time.sleep(1)

#object list
lines = open('/home/sgme/yolov5/output.txt').readlines()
given_map=file_read.map()
start_row = 0
start_col = 0

while True :
	#Make move order from start position
	move_order, given_map, start_row, start_col, pet_list, can_list = dijkstra.main(given_map, start_row, start_col)
	if (len(pet_list) == 0 and len(can_list) == 0):
		print("FINISH\n")
		break
	
	while len(move_order):
		py_serial.write(move_order.pop(0))
		print("pop")
		time.sleep(1.0) #delay time decided by velocity and distance		
	
```
			  
			  

  


