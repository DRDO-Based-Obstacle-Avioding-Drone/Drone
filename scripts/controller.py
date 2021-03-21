#!/usr/bin/env python

"""
set_attitude_target.py: (Copter Only)
This example shows how to move/direct Copter and send commands
 in GUIDED_NOGPS mode using DroneKit Python.
Caution: A lot of unexpected behaviors may occur in GUIDED_NOGPS mode.
        Always watch the drone movement, and make sure that you are in dangerless environment.
        Land the drone as soon as possible when it shows any unexpected behavior.
Tested in Python 2.7.10
"""



import time
import math
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
# from vitarana_drone.msg import *
# from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf
from numpy import zeros
from collections import deque
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import rospy
import time





def lat_to_x(input_latitude):
    return 110692.0702932625*(input_latitude+35.3632591)
def long_to_x(input_longitude):
    return -105292.0089353767*(input_longitude-149.1653452)

def x_to_lat(input_x):
    return input_x/110692.0702932625 - 35.3632591
def x_to_long(input_x):
    return input_x/(-105292.0089353767) + 149.1653452

offset_x  = 50
offset_y = 0
maze = zeros([101,101],int)
factor = 0.5

def astar(maze,start,end):
    parent=zeros([101,101,2],int)
    vis=zeros([101,101],int)
    insertq=zeros([101,101],int)
    for i in range(100):
        for j in range(100):
            parent[i][j][0]=parent[i][j][1]=-1

    q=deque()
    q.append(start)
    val=start
    insertq[start[0]][start[1]]=1

    while q:
        temp=q.popleft()
        vis[temp[0]][temp[1]]=1
        if temp[0] == end[0] and temp[1] == end[1]:
            break
        if temp[0]-1>=0 and insertq[temp[0]-1][temp[1]]==0 and maze[temp[0]-1][temp[1]]==0:
            insertq[temp[0]-1][temp[1]]=1
            q.append([temp[0]-1,temp[1]])
            parent[temp[0]-1][temp[1]][0]=temp[0]
            parent[temp[0]-1][temp[1]][1]=temp[1]
        if temp[1]+1<=100 and insertq[temp[0]][temp[1]+1]==0 and maze[temp[0]][temp[1]+1]==0:
            insertq[temp[0]][temp[1]+1]=1
            q.append([temp[0],temp[1]+1])
            parent[temp[0]][temp[1]+1][0]=temp[0]
            parent[temp[0]][temp[1]+1][1]=temp[1]
        if temp[1]-1>=0 and insertq[temp[0]][temp[1]-1]==0 and maze[temp[0]][temp[1]-1]==0:
            insertq[temp[0]][temp[1]-1]=1
            q.append([temp[0],temp[1]-1])
            parent[temp[0]][temp[1]-1][0]=temp[0]
            parent[temp[0]][temp[1]-1][1]=temp[1]
        if temp[0]+1<=100 and insertq[temp[0]+1][temp[1]]==0 and maze[temp[0]+1][temp[1]]==0:
            insertq[temp[0]+1][temp[1]]=1
            q.append([temp[0]+1,temp[1]])
            parent[temp[0]+1][temp[1]][0]=temp[0]
            parent[temp[0]+1][temp[1]][1]=temp[1]
        if temp[0]+1<=100 and temp[1]+1<=100 and insertq[temp[0]+1][temp[1]+1]==0 and maze[temp[0]+1][temp[1]+1]==0:
            insertq[temp[0]+1][temp[1]+1]=1
            q.append([temp[0]+1,temp[1]+1])
            parent[temp[0]+1][temp[1]+1][0]=temp[0]
            parent[temp[0]+1][temp[1]+1][1]=temp[1]

        if temp[0]+1<=100 and temp[1]-1>=0 and insertq[temp[0]+1][temp[1]-1]==0 and maze[temp[0]+1][temp[1]-1]==0:
            insertq[temp[0]+1][temp[1]-1]=1
            q.append([temp[0]+1,temp[1]-1])
            parent[temp[0]+1][temp[1]-1][0]=temp[0]
            parent[temp[0]+1][temp[1]-1][1]=temp[1]

        if temp[0]-1>=0 and temp[1]+1<=100 and insertq[temp[0]-1][temp[1]+1]==0 and maze[temp[0]-1][temp[1]+1]==0:
            insertq[temp[0]-1][temp[1]+1]=1
            q.append([temp[0]-1,temp[1]+1])
            parent[temp[0]-1][temp[1]+1][0]=temp[0]
            parent[temp[0]-1][temp[1]+1][1]=temp[1]

        if temp[0]-1>=0 and temp[1]-1>=0 and insertq[temp[0]-1][temp[1]-1]==0 and maze[temp[0]-1][temp[1]-1]==0:
            insertq[temp[0]-1][temp[1]-1]=1
            q.append([temp[0]-1,temp[1]-1])
            parent[temp[0]-1][temp[1]-1][0]=temp[0]
            parent[temp[0]-1][temp[1]-1][1]=temp[1]
    
    temp=[end[0],end[1]]
    q1=deque()
    path=[]

    while temp[0]>=0 :
        q1.append(temp)
        temp=parent[temp[0]][temp[1]]

    #****************RETURNING PATH*******************************
    while q1:
        temp=q1.pop()
        path.append(temp)

    return path   


def operate_astar(start,end,drone,maze,obs):
    print("INSIDE OPERATE ASTAR")
    global factor
    print(start)
    print(end)
    start[0] = (int)(start[0]/factor)+50
    start[1] = (int)(start[1]/factor)
    final_dest = end
    print(final_dest)
    end[0] = (int)(end[0]/factor)+50
    end[1] = (int)(end[1]/factor)
    print(start)
    print(end)
    for p in obs:
        if abs(p[1])<1:
            cur_x = (int)((drone[0]+p[0])/factor)+50
            cur_y = (int)((drone[1]+p[2])/factor)
            k1 = -1
            while k1<=1:
                k2=-1
                while k2<=1:
                    maze[cur_x+k1][cur_y+k2]=1
                    k2=k2+1
                k1=k1+1
            # maze[(int)((drone[0]+p[0])/factor)+50][(int)((drone[1]+p[2])/factor)]=1
            # maze[(int)((drone[0]+p[0])/factor)+50+1][(int)((drone[1]+p[2])/factor)]=1
            # maze[(int)((drone[0]+p[0])/factor)+50-1][(int)((drone[1]+p[2])/factor)]=1

    print("PRINTING MAZE")
    for i in range(100):
        for j in range(100):
            if maze[i][j] == 1:
                print(i,j)
    maze[start[0]][start[1]] = 0
    path = astar(maze,start,end)
    print("start end")
    print(start)
    print(end)
    # print(path[1])
    print("PATH")
    print("-------")
    print(path)
    print("------")
    print(len(path))
    print(final_dest)
    if(len(path)<=1):
        return [factor*(final_dest[0]-50),factor*final_dest[1]]
    else:
        return [factor*(path[1][0]-50) , factor*path[1][1]]
    




class Iris():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')
        self.alt_pub = rospy.Publisher('/Thrust', Float32, queue_size=10)
        self.lat_pub = rospy.Publisher('/Roll', Float32, queue_size=10)
        self.long_pub = rospy.Publisher('/Pitch', Float32, queue_size=10)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.PoseCallBack)
        rospy.Subscriber('/depth_camera/depth/points', PointCloud2, self.callback_pointcloud)
        self.postn = [0,0,0]
        self.set_postn = [0,26,3]
        self.thrust =0
        self.roll =0
        self.pitch=0
        self.gen = [[0,0,0]]

    def callback_pointcloud(self,data):
        assert isinstance(data, PointCloud2)
        self.gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        time.sleep(1)
    # print(data.height,data.width)
    # assert isinstance(data, PointCloud2)
    # self.gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    # time.sleep(1)
    # # print type(gen)
    # # for p in gen:
    # #     print(p)
    # #     print " x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2])
        



    def PoseCallBack(self,msg):
        self.postn[0] = lat_to_x(msg.latitude)
        self.postn[1] = long_to_x(msg.longitude)
        self.postn[2] = msg.altitude - 603.35877


    def guider(self):
        self.iter = 0
        self.lat_iter=0
        self.long_iter=0
        self.prev_height_err = 0
        self.prev_lat_err =0
        self.prev_long_err =0
        self.flag = 0
        self.next_lat = 0
        self.next_long = 0
        self.condition = 0
        self.arr = [[0,0,0]]
        while(True):
            global maze
            if(abs(self.next_lat-self.postn[0])<2 and abs(self.next_long - self.postn[1])<2):
                time.sleep(3)
                val = operate_astar([self.next_lat,self.next_long],[self.set_postn[0],self.set_postn[1]],[self.postn[0],self.postn[1]],maze,self.gen)
                self.next_lat = val[0]
                self.next_long = val[1]
            self.alt_pub.publish(self.set_postn[2])
            self.lat_pub.publish((self.next_lat))
            self.long_pub.publish((self.next_long))
            time.sleep(0.1)
            print(self.postn)
            print("PUBLISHED->")
            print(self.next_lat,self.next_long)
            print(self.set_postn)

            
            









#main

if __name__ == "__main__":
    try:
        publish_cmd = Iris()
        time.sleep(1)
        publish_cmd.guider()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")

