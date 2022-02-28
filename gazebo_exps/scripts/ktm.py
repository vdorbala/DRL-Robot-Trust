#! /usr/bin/env python

from collections import defaultdict 

import math
from pathplan import Node
from pathplan import astar

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import numpy as np
import tf
from maze import maze, m2g, gazebo
import random


ps=[0,0.5,0.75,1]
prob_set={}
count=1
for i in ps:
    for j in ps:
        prob_set[i*j]=count
        count=count+1

# for i in prob_set:
#   print(i)
max_prob_list=list(prob_set)
l=len(max_prob_list)
prob_dis=np.array([1,0.75,0.5,0.3])

def def_value(): 
    return ""
front = defaultdict(def_value) 
front[1] = "D"
front[-1] = "U"
side= defaultdict(def_value) 
side[1]="R"
side[-1]="L"

def prob_scores(goal): 
        init_prob=[1,0.75,0.5] 
        ipr=np.random.choice(3)
        goal_prob=[init_prob[ipr]]

        if(len(goal)==1):
            print('hi')
            return goal_prob
        else:
            i=0
            while(len(goal_prob)<=len(goal)-1):
                a=max_prob_list[np.random.choice(l,1,p=[0.1,0.15,0.05,0.3,0.1,0.15,0.15])[0]]
                if (a <=goal_prob[i]):
                    goal_prob.append(a)
                    i=i+1
        s=sum(goal_prob)
        avg=s/len(goal_prob)
        return [goal_prob,avg]

def close(a,b):
    return np.argmin(abs(prob_dis-a))==b[1]

def correct_prob(start,goal,command):
    avg=prob_scores(command)
    dp=distribution(start,goal)


    while(not close(avg[1], dp)):
        avg=prob_scores(command)

    return(avg,dp)
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
        if(distance(pos,gazebo[inters])>distance(pos,gazebo[i]) and inter(m2g[i])):
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

def interneighbour(pos,maze):
    p=[tuple(pos)]
    N=(pos[0]+2,pos[1])
    S=(pos[0]-2,pos[1])
    E=(pos[0],pos[1]+2)
    W=(pos[0],pos[1]-2)

    if(bounds(maze,N)):
        p.append(N)
    if(bounds(maze,S)):
        p.append(S)
    if(bounds(maze,E)):
        p.append(E)
    if(bounds(maze,W)):
        p.append(W)
    return p
def farneighbour(pos,maze):
    NE=(pos[0]+2,pos[1]+2)
    SW=(pos[0]-2,pos[1]-2)
    SE=(pos[0]-2,pos[1]+2)
    NW=(pos[0]+2,pos[1]-2)

    if(bounds(maze,NE)):
        p.append(NE)
    if(bounds(maze,SE)):
        p.append(SE)
    if(bounds(maze,SW)):
        p.append(SW)
    if(bounds(maze,NW)):
        p.append(NW)
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
    return mp


def command(path,i_ori):
    command=""
    if(len(path))<2:
        return command
    prev=front[path[1][0]-path[0][0]]+side[path[1][1]-path[0][1]]
    command = prev
    for i in range(len(path)-1):
        dir=front[path[i+1][0]-path[i][0]]+side[path[i+1][1]-path[i][1]]
        if(inter(path[i])):
            command = command + dir

    return command

#End of corridor cut simultaneous ups
def cutu(path):
    a=path[:2]
    for i in range(1,len(path)):
        if(path[i]!="U"):
            a=path
            break
    return a


def approx_dir(yaw):
    a=""
    if(0<yaw<0.78 or -0.78<yaw<=0):
        a="R"
    if(0.78<= yaw  and yaw<=2.35):
        a="U"
    if(yaw>2.35 or yaw<=-2.35):
        a="L"
    if(-2.35<yaw<=-0.78):
        a="D"
    return a

def rd(a,b):
    return(b[0]-a[0],b[1]-a[1])

def distribution(start,goal):
    if(distance(start,goal)<=3):
        return [prob_dis[0],0]
    if(3<distance(start,goal)<=6):
        return [prob_dis[1],1]
    if(6<distance(start,goal)<=9):
        return [prob_dis[2],2]
    if(9<distance(start,goal)):
        return [prob_dis[3],3]

def get_key(val,my_dict): 
    for key, value in my_dict.items(): 
         if val == value: 
             return key 

def main(END_GOAL, res):
    pr=1
    global reset

    if res==None:
        return
    x = np.round(res.pose.position.x,3)
    y = np.round(res.pose.position.y,3)

    euler = tf.transformations.euler_from_quaternion((res.pose.orientation.x,res.pose.orientation.y,res.pose.orientation.z,res.pose.orientation.w))
    yaw=np.round(euler[2],3)
    gazebo_start=[x,y]
    gazebo_end=END_GOAL
    startnode=tuple(m2g[interpos(gazebo_start,gazebo,m2g)])  #room point should not be chosen
    endnode=tuple(m2g[interposend(gazebo_end,gazebo,m2g)])  #choose closest intersection point
    i_ori=approx_dir(yaw)  #initial orientation
    confidence=distribution(startnode,endnode)
    print("start end initial orientation confidence")
    print(startnode,endnode,i_ori,confidence[0])
    # probgoal=goalprob(endnode,maze)
    if (confidence[0]==1):
        path = astar(maze, startnode, endnode)
        goal_command=command(path,i_ori)
        goal_pt=endnode
        print(command(path,i_ori))
        out=correct_prob(startnode,goal_pt,goal_command)
        print("final prob distribution",out[0][0],"avg prob",out[0][1])
    probgoal=interneighbour(endnode,maze)
    if(confidence[0]==0.3):
        probgoal=farneighbour(endnode,maze)
    # maxgoal=predmaxprobgoal(endnode,maze)
    nonrepgoal={}
    count=1
    # print(probgoal)
    if probgoal == None:
        return
    dst=[]
    dd={}
    for i in probgoal:
        if(notroom(i)):
          path = astar(maze, startnode, i)
          nonrepgoal[command(path,i_ori)] = distance(startnode,i)# with cutting 
          dst.append(distance(startnode,i))
          dd[distance(startnode,i)]=i

          count+=1

    dst.sort()
    # print(dst)
    if(confidence[0]==0.75):
        if(len(dst)>2):
           ch=np.random.choice(dst[:(len(dst)/2)+1], 1)[0]
        else:
           ch=np.random.choice(dst, 1)[0]
        goal_command=get_key(ch,nonrepgoal)
        goal_pt=dd[ch]
        print("goal_command",goal_command,goal_pt)
        out=correct_prob(startnode,goal_pt,goal_command)
        print("final prob distribution",out[0][0],"avg prob",out[0][1])

    if(confidence[0]==0.5):
        if(len(dst)>2):
           ch=np.random.choice(dst[(len(dst)/2)-1:], 1)[0]
        else:
            ch=np.random.choice(dst, 1)[0]
        goal_pt=dd[ch]
        goal_command=get_key(ch,nonrepgoal)
        print("goal_command",goal_command,goal_pt)
        out=correct_prob(startnode,goal_pt,goal_command)
        print("final prob distribution",out[0][0],"avg prob",out[0][1])

    if(confidence[0]==0.3):
        ch=np.random.choice(dst, 1)[0]
        goal_pt=dd[ch]
        goal_command=get_key(ch,nonrepgoal)
        print("goal_command",goal_command,goal_pt)
        out=correct_prob(startnode,goal_pt,goal_command)
        print("final prob distribution",out[0][0],"avg prob",out[0][1])
    print("All possible goal")
    print("command goal prob/dist from start to goal")
    for i in nonrepgoal:
                print(i,dd[nonrepgoal[i]],nonrepgoal[i])
    
    # use goal_command for global command directions start symbol is the orientation change
    # use out[0][0]) for correspondind prob distribution
    print(out[0][0])
    sym_pub.publish(str(goal_command))

    if rospy.has_param('goal_reached'):
        reset = rospy.get_param("goal_reached")
        return

if __name__ == '__main__':

        rospy.init_node("goal_sender")

        rospy.set_param('goal_reached', True)

        sym_pub = rospy.Publisher('/symbols', String, queue_size=0)

        x_size = 22
        y_size = 17

        while not rospy.is_shutdown():
            res = gms_client("turtlebot3_burger", "link")
            x = random.uniform(-x_size, x_size)
            y = random.uniform(-y_size, y_size)

            reset = rospy.get_param("goal_reached")

            END_GOAL = [np.round(x,2), np.round(y,2)]
            while reset:
                print("Sending goal {}".format(END_GOAL))
                main(END_GOAL, res)
                rospy.set_param("goal_reached", False)
                reset = rospy.get_param("goal_reached")
