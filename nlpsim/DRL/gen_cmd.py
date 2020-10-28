import numpy as np
from gym_minigrid.envs.maze import maze,m2g, gazebo ,inter_x,mid_x
from gym_minigrid.envs.pathplan import astar
from collections import defaultdict 
import math



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


Dict = {"U": {"L":"L","R":"R","U":"U","D":"D"}, "L": {"L":"U","R":"D","U":"R","D":"L"}, "R": {"L":"D","R":"U","U":"L","D":"R"},"D":{"L":"R","R":"L","U":"D","D":"U"}} 

dir_vec={'R':0,'D':1,'L':2,'U':3}






def str_nparray(st):
    lt=[]
    for i in st:
        lt.append(dir_vec[i])
    return np.array(lt)


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
def notroom(x):
        a=True
        if(x[0]%2==1 and x[1]%2==1):
            a=False
        return a

def distribution(start,goal):
    if(distance(start,goal)<=6):
        return [prob_dis[0],0]
    if(6<distance(start,goal)<=12):
        return [prob_dis[1],1]
    if(12<distance(start,goal)<=15):
        return [prob_dis[2],2]
    if(15<distance(start,goal)):
        return [prob_dis[3],3]

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

def bounds(maze,curr):
    a=True
    if curr[0] > (len(maze) - 1) or curr[0] < 0 or curr[1] > (len(maze[len(maze)-1]) -1) or curr[1] < 0 :
                    a=False
    return a
def close(a,b):
    return np.argmin(abs(prob_dis-a))==b[1]

def correct_prob(start,goal,command):
    avg=prob_scores(command)
    dp=distribution(start,goal)


    while(not close(avg[1], dp)):
        avg=prob_scores(command)

    return(avg,dp)
DIR_TO_STR = {
            0: 'R',
            1: 'D',
            2: 'L',
            3: 'U'
        }

def farneighbour(pos,maze):
    NE=(pos[0]+2,pos[1]+2)
    SW=(pos[0]-2,pos[1]-2)
    SE=(pos[0]-2,pos[1]+2)
    NW=(pos[0]+2,pos[1]-2)
    p=[]
    if(bounds(maze,NE)):
        p.append(NE)
    if(bounds(maze,SE)):
        p.append(SE)
    if(bounds(maze,SW)):
        p.append(SW)
    if(bounds(maze,NW)):
        p.append(NW)
    return p
def get_key(val,my_dict): 
        for key, value in my_dict.items(): 
             if val == value: 
                 return key 

def inter(x):
    a=False
    if(x[0]%2==0 and x[1]%2==0):
        a=True
    return a

# def command(path,i_ori):
#     command=""
#     if(len(path))<2:
#         return command
#     prev=front[path[1][0]-path[0][0]]+side[path[1][1]-path[0][1]]
#     command = prev
#     for i in range(len(path)-1):
#         dir=front[path[i+1][0]-path[i][0]]+side[path[i+1][1]-path[i][1]]
#         if(inter(path[i])):
#             command = command + dir

#     return command



def command(path,i_ori):
    command=""
    # print(path)
    if(len(path))<2:
        return command
    prev=front[path[1][0]-path[0][0]]+side[path[1][1]-path[0][1]]
    if(i_ori !=prev):
        command= command + Dict[i_ori][prev]
    # else:
    #     command= command + "S"
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




def distance(startCoordinate,goalCoordinate):
    sx,sy = startCoordinate[0],startCoordinate[1]
    gx,gy = goalCoordinate[0],goalCoordinate[1]
    return math.sqrt((gx-sx)**2 + (gy-sy)**2)#instead of using scipy.spatial importing distance


def check_room(gp):
    x = np.arange(4, 60, 8)
    for i in x:
          if(i<=gp[0]<i+5):
              for j in x:
                  if (j<=gp[1]<j+5):
                          return True
    return False

def gridclose(a,b):
      x = abs(np.array(b)-a[0])
      y = abs(np.array(b)-a[1])
      pos_x=[]
      pos_y=[]
      for i in range(len(x)):
        if (x[i]<3):
            pos_x.append(i)
        if (y[i]<3):
            pos_y.append(i)
      for idx in pos_x:
        for idy in pos_y:
             if not check_room([b[idx], b[idy]]):
                # print([b[idx], b[idy]],self.check_inter([b[idx], b[idy]],b))
                # print(m2g[self.get_key([b[idx], b[idy]],gazebo)])
                return tuple(m2g[get_key([b[idx], b[idy]],gazebo)])

def check_mid(gp):
    if((gp[0] in mid_x) and (gp[1] in mid_x)):
            return True
    return False


def get_command(agentpos, agentdir, goalpos):
        print("In the commd")

        startnode=gridclose(agentpos,inter_x)

        endnode=gridclose(goalpos,inter_x)

        i_ori=DIR_TO_STR[agentdir]
        path = astar(maze, startnode, endnode)
        # print(command(path,i_ori))
        goal_command=command(path,i_ori)
        confidence=distribution(startnode,endnode)
        print(confidence)

        out=correct_prob(startnode,endnode,goal_command)
        print("final prob distribution",out[0][0],"avg prob",out[0][1])
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
        nonrepgoal={}
        count=1
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
        print(dst)
        if(confidence[0]==0.75):
            if(len(dst)>2):
               ch=np.random.choice(dst[:int(len(dst)/2)+1], 1)[0]
            else:
               print(dst[:(len(dst)/2)+1])
               ch=np.random.choice(dst, 1)[0]
            goal_command=get_key(ch,nonrepgoal)
            goal_pt=dd[ch]
            print("goal_command",goal_command,goal_pt)
            out=correct_prob(startnode,goal_pt,goal_command)
            print("final prob distribution",out[0][0],"avg prob",out[0][1])

        if(confidence[0]==0.5):
            if(len(dst)>2):
               ch=np.random.choice(dst[int(len(dst)/2)-1:], 1)[0]
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
        # print("All possible goal")
        # print("command goal prob/dist from start to goal")
        # for i in nonrepgoal:
        #             print(i,dd[nonrepgoal[i]],nonrepgoal[i])
        
        # use goal_command for global command directions start symbol is the orientation change
        # use out[0][0]) for correspondind prob distribution

        return [str_nparray(goal_command), out[0][1]]



