#! /usr/bin/env python

from collections import defaultdict 

import math
from pathplan import Node
from pathplan import astar

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import numpy as np
import tf
from maze import maze
from maze import m2g
from maze import gazebo


def def_value(): 
    return ""
front = defaultdict(def_value) 
front[1] = "D"
front[-1] = "U"
side= defaultdict(def_value) 
side[1]="R"
side[-1]="L"

Dict = {"U": {"L":"L","R":"R","U":"U","D":"D"}, "L": {"L":"U","R":"D","U":"R","D":"L"}, "R": {"L":"D","R":"U","U":"L","D":"R"},"D":{"L":"R","R":"L","U":"D","D":"U"}} 


def inter(x):
    a=False
    if(x[0]%2==1 and x[1]%2==1):
        a=True
    return a

def notroom(x):
    a=True
    if(x[0]%2==0 and x[1]%2==0):
        a=False
    return a

def distance(startCoordinate,goalCoordinate):
    sx,sy = startCoordinate[0],startCoordinate[1]
    gx,gy = goalCoordinate[0],goalCoordinate[1]
    return math.sqrt((gx-sx)**2 + (gy-sy)**2)#instead of using scipy.spatial importing distance

def interpos(pos,gazebo,m2g):
    inter = 2
    for i in gazebo:
        if(distance(pos,gazebo[inter])>distance(pos,gazebo[i]) and notroom(m2g[i])):
            inter=i
    
    return inter
def interposend(pos,gazebo,m2g):
    inters = 1
    for i in gazebo:
        if(distance(pos,gazebo[inters])>distance(pos,gazebo[i])):
            inters=i
    
    return inters
def bounds(maze,curr):
    a=True
    if curr[0] > (len(maze) - 1) or curr[0] < 0 or curr[1] > (len(maze[len(maze)-1]) -1) or curr[1] < 0 :
                    a=False
    return a
def maxprobgoal(pos):
    if(pos[0] <3):
       return 1
    if(3<pos[0] <7):
       return 5
    if(7<pos[0] <11):
       return 9

def goalprob(pos,maze):
    p=[]
    if(bounds(maze,[pos[0]+1,pos[1]])):
        p.append((pos[0]+1,pos[1]))
    if(bounds(maze,[pos[0]-1,pos[1]])):
        p.append((pos[0]-1,pos[1]))
    if(bounds(maze,[pos[0],pos[1]-1])):
        p.append((pos[0],pos[1]-1))
    if(bounds(maze,[pos[0],pos[1]+1])):
        p.append((pos[0],pos[1]+1))
    if(bounds(maze,[pos[0]+1,pos[1]+1])):
        p.append((pos[0]+1,pos[1]+1))
    if(bounds(maze,[pos[0]-1,pos[1]-1])):
        p.append((pos[0]-1,pos[1]-1))
    if(bounds(maze,[pos[0]+1,pos[1]-1])):
        p.append((pos[0]+1,pos[1]-1))
    if(bounds(maze,[pos[0]-1,pos[1]+1])):
        p.append((pos[0]-1,pos[1]+1))
    # p.append(pos)
    return p
def gms_client(model_name,relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name,relative_entity_name)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
def predmaxprobgoal(pos,maze):
    y=maxprobgoal(pos)
    p=goalprob(pos,maze)
    mp=[]
    for i in p:
        if (i[0] == y):
            mp.append(i)
            # print("most prob goal is",i)
    return mp


def command(path,i_ori):
    command=""
    prev=front[path[1][0]-path[0][0]]+side[path[1][1]-path[0][1]]
    if(i_ori !=prev):
        command+=Dict[i_ori][prev]
    else:
        command+="S"
    for i in range(len(path)-1):
        dir=front[path[i+1][0]-path[i][0]]+side[path[i+1][1]-path[i][1]]
        # print("present ",dir)
        if(inter(path[i])):
            # print("Intersection")
            # print("append",Dict[prev][dir])
            # command.append(Dict[prev][dir])
            command+=Dict[prev][dir]

        prev=dir

    return command


def cutu(path):
    a=path[:2]
    for i in range(1,len(path)):
        if(path[i]!="U"):
            a=path
            break
    return a


def approx_dir(yaw):
    a=""
    if(0<yaw<0.78 or -0.78<yaw<0):
        a="R"
    if(0.78<= yaw  and yaw<=2.35):
        a="U"
    if(yaw>2.35 or yaw<=-2.35):
        a="L"
    if(-2.35<yaw<=-0.78):
        a="D"
    return a



def main():

    rospy.init_node('odom_pub')
    pr=1
    while not rospy.is_shutdown():
        res = gms_client("turtlebot3_burger", "")

        x = np.round(res.pose.position.x,3)
        y= np.round(res.pose.position.y,3)
        euler = tf.transformations.euler_from_quaternion((res.pose.orientation.x,res.pose.orientation.y,res.pose.orientation.z,res.pose.orientation.w))
        yaw=np.round(euler[2],3)
        gazebo_start=[x,y]
        gazebo_end=[-20,14]
        startnode=tuple(m2g[interpos(gazebo_start,gazebo,m2g)])#not a room 
        endnode=tuple(m2g[interposend(gazebo_end,gazebo,m2g)])#mostly a room

        i_ori=approx_dir(yaw)#initial orientation
        probgoal=goalprob(endnode,maze)
        maxgoal=predmaxprobgoal(endnode,maze)
        nonrepgoal={}
        count=1
        for i in probgoal:
            path = astar(maze, startnode, i)
            # print("path",path)
            # print(command(path,i_ori))
            nonrepgoal[cutu(command(path,i_ori))] = count
            count+=1
        if(pr%100==0):
            print("probable no of goal points surrounding it is ",len(probgoal))
            print("start node asscoiation",startnode)
            print("end node asscoiation",endnode)
            print(x,y,yaw)
            for i in nonrepgoal:
                print(i)
        count=1
        pr+=1
        # maxrepgoal={}
        # for i in maxgoal:
        #     path = astar(maze, startnode, i)
        #     print("path",path)
        #     # print(command(path,i_ori))
        #     maxrepgoal[cutu(command(path,i_ori))] = count
        #     count+=1
        # for i in maxrepgoal:
        #     print("max prob path is",i)



if __name__ == '__main__':
                    main()

