from gym_minigrid.minigrid import *
from gym_minigrid.register import register
from operator import add
import random
import numpy as np

from gym_minigrid.envs.gen_cmd import get_dir,get_conf


from gym_minigrid.envs.maze import maze,m2g, gazebo ,inter_x,mid_x
from gym_minigrid.envs.pathplan import astar
# from gym_minigrid.envs.pos import check_room,check_mid

from collections import defaultdict 

# def check_room(gp):
#     x = np.arange(4, 60, 8)
#     for i in x:
#           if(i<=gp[0]<i+5):
#               for j in x:
#                   if (j<=gp[1]<j+5):
#                           return True
#     return False

# def check_mid(gp):
#     if(gp[0] in mid_x) and (gp[1] in mid_x) :
#             return True
#     return False

# gp =(random.randint(1,59), random.randint(1,59))


#         # CHECK if GOAL lies inside room or on walls.
# # gp=self.goal_pos

# while((not check_mid(gp))):
#     gp=(random.randint(1,59), random.randint(1,59))
# # while(self.check_room(gp)):
# #     gp=(random.randint(1,59), random.randint(1,59))
# goalpos=gp

# gp =(random.randint(1,59), random.randint(1,59))
# while((not check_mid(gp))):
#     gp=(random.randint(1,59), random.randint(1,59))
# # while(self.check_room(gp)):
# #     gp=(random.randint(1,59), random.randint(1,59))
# agentstartpos=gp
# # mid_x=[ 2, 10, 18, 26, 34, 42, 50, 58]


# print("generated start",agentstartpos)
# print("generated end",goalpos)

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

    if(bounds(maze,NE)):
        p.append(NE)
    if(bounds(maze,SE)):
        p.append(SE)
    if(bounds(maze,SW)):
        p.append(SW)
    if(bounds(maze,NW)):
        p.append(NW)
    return p

def inter(x):
    a=False
    if(x[0]%2==0 and x[1]%2==0):
        a=True
    return a

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

def distance(startCoordinate,goalCoordinate):
    sx,sy = startCoordinate[0],startCoordinate[1]
    gx,gy = goalCoordinate[0],goalCoordinate[1]
    return math.sqrt((gx-sx)**2 + (gy-sy)**2)#instead of using scipy.spatial importing distance

mid_x=[ 2, 10, 18, 26, 34, 42, 50, 58]

class DynamicObstaclesEnv(MiniGridEnv):
    """
    Single-room square grid environment with moving obstacles
    """

    def __init__(
            self,
            size=8,
            agent_start_pos=(1, 1),
            agent_start_dir=0,
            n_obstacles=4,
            goal_pos=(15,15),
    ):
        print("Starting position is {}\n. Direction is {}".format(agent_start_pos, agent_start_dir))
        self.agent_start_pos = agent_start_pos
        self.agent_start_dir = agent_start_dir
        self.goal_pos = goal_pos
      
        # Number of cells (width and height) in the agent view
        # assert self.agent_view_size % 2 == 1
        # assert self.agent_view_size >= 3

        # self.agent_view_size = agent_view_size

        self.commands = get_dir()

        self.confidence = get_conf()

        self.icount = 0

        self.gcount = 0

        self.human_detect = False

        self.gateway_detect = False

        # Reduce obstacles if there are too many
        if n_obstacles <= size/2 + 1:
            self.n_obstacles = int(n_obstacles)
        else:
            self.n_obstacles = int(size/2)

        super().__init__(
            grid_size=size,
            max_steps=4 * size * size,
            # Set this to True for maximum speed
            see_through_walls=True,
        )

        print(self.icount)

        # Allow only 5 actions permitted: left (0), right (1), forward (2), backward(-1), and interact (7)
        self.action_space = spaces.Discrete(5) # L, R, U, D, I
        # self.action_space = [-1, 0, 1, 2, 7]

        print("Action space is {}".format(self.action_space))

        self.reward_range = (-200, 500)


        # self.observation_space = spaces.Tuple((spaces.MultiDiscrete([(0, 3), (0, 3), (0, 3), (0, 3)]), 
        #                                 spaces.Box(0,1,shape=(1,)),
        #                                 spaces.Discrete(10),
        #                                 spaces.Discrete(10),
        #                                 spaces.Discrete(2),
        #                                 spaces.Discrete(2)))


        self.observation_space = spaces.Dict({
            'commands': spaces.MultiDiscrete([3, 3, 3, 3]),
            'confidence': spaces.Box(0,1,shape=(1,)),
            'icount': spaces.Discrete(10),
            'gcount': spaces.Discrete(10),
            'human_detect': spaces.Discrete(2),
            'gateway_detect': spaces.Discrete(2)
        })


    @property
    def front_pos(self):
        """
        Get the position of the cell that is right in front of the agent
        """

        return self.agent_pos + self.dir_vec

    @property
    def back_pos(self):
        """
        Get the position of the cell that is right in front of the agent
        """

        return self.agent_pos - self.dir_vec

    def generate_obs(self):
        """
        Generate the agent's view (partially observable, low-resolution encoding)
        """
        print("Commands are {}, {}, {}, {}, {}, {}".format(self.commands, self.confidence, self.icount, self.gcount, self.human_detect, self.gateway_detect))
        # Observations are dictionaries containing:
        # - Human confidence levels
        # - Human language commands
        # - A count of number of gateways
        # - A count of the number of interactions
        # - If humans have been detected or not
        # - If intersections have been detected or not
        # self.commands = spaces.MultiDiscrete(self.commands)

        obs = spaces.Dict({
            'commands': self.commands,
            'confidence': self.confidence,
            'icount': self.icount,
            'gcount': self.gcount,
            'human_detect': self.human_detect,
            'gateway_detect': self.gateway_detect
        })


        return obs


    # def check_room(self, gp):
    #     x = np.arange(4, 60, 8)
    #     # y = np.arange(4, 60, 8)

    #     for i in x:
    #         # for j in y:
    #           if(i<=gp[0]<i+5) and (i<=gp[1]<i+5):
    #                return True
    #     return False
    def check_room(gp):
        x = np.arange(4, 60, 8)
        for i in x:
              if(i<=gp[0]<i+5):
                  for j in x:
                      if (j<=gp[1]<j+5):
                              return True
        return False
    # def check_mid(self, gp):
    #     if(gp[0] in inter_x) or (gp[1] in inter_x) :
    #             return True
    #     return False
    def check_inter(self,a,b):
        if ((a[0] in b[0:-1:2]) and (a[1] in b[0:-1:2])):
                 return True
        return False

    def check_gateway(self):
          # a=self.grid.get(*self.front_pos)
        a=self.agent_pos
        print("a is",a)
        x = inter_x[0:-1:2]
        for i in x:
          if(i-1<=a[0]<i+2):
                  for j in x:
                      if (j-1<=a[1]<j+2):
                              print("inter")
                              return 1
        return 2
                    # print(m2g[self.get_key([b[idx], b[idy]],gazebo)])

    def check_room(self, gp):
        x = np.arange(4, 60, 8)
        for i in x:
              if(i<=gp[0]<i+5):
                  for j in x:
                      if (j<=gp[1]<j+5):
                              return True
        return False
    # function to return key for any value 
    def get_key(self,val,my_dict): 
        for key, value in my_dict.items(): 
            if val == value: 
                  return key 
        return "key doesn't exist"

    def gridclose(self,a,b):
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
                 if not self.check_room([b[idx], b[idy]]):
                    # print([b[idx], b[idy]],self.check_inter([b[idx], b[idy]],b))
                    # print(m2g[self.get_key([b[idx], b[idy]],gazebo)])
                    return tuple(m2g[self.get_key([b[idx], b[idy]],gazebo)])

    def check_mid(self, gp):
        if((gp[0] in mid_x) and (gp[1] in mid_x)):
                return True
        return False

    def distribution(self,start,goal):
        if(distance(start,goal)<=6):
            return [prob_dis[0],0]
        if(6<distance(start,goal)<=12):
            return [prob_dis[1],1]
        if(12<distance(start,goal)<=15):
            return [prob_dis[2],2]
        if(15<distance(start,goal)):
            return [prob_dis[3],3]

    def _gen_grid(self, width, height):
        # Create an empty grid
        self.grid = Grid(width, height)

        # Generate the surrounding walls
        self.grid.wall_rect(0, 0, width, height)

        x = np.arange(4, 60, 8)
        # y = np.arange(4, 44, 10)
        y = np.arange(4, 60, 8)
        # xx, yy = np.meshgrid(x, y)
        # Generate rooms
        for i in range(0, len(x)):
            for j in range(0, len(y)):
                self.grid.wall_rect(x[i], y[j], 5, 5)


        # CHECK if GOAL lies inside room or on walls.
        gp=self.goal_pos
        while(not self.check_mid(gp)):
            gp=(random.randint(1,59), random.randint(1,59))
        # while(self.check_room(gp)):
        #     gp=(random.randint(1,59), random.randint(1,59))
        self.goal_pos=gp

        gp=self.agent_start_pos
        while(not self.check_mid(gp)):
            gp=(random.randint(1,59), random.randint(1,59))
        # while(self.check_room(gp)):
        #     gp=(random.randint(1,59), random.randint(1,59))
        self.agent_start_pos=gp
        

        # Place a goal square at a random location
        self.grid.set(self.goal_pos[0], self.goal_pos[1], Goal())

        # CHECK if AGENT lies inside room or on walls.
        # self.agent_start_pos !=:
        print("mod start pos is",self.agent_start_pos)
        print("mod goal pos is",self.goal_pos)

        startnode=self.gridclose(self.agent_start_pos,inter_x)
        endnode=self.gridclose(self.goal_pos,inter_x)
        i_ori=DIR_TO_STR[self.agent_start_dir]
        path = astar(maze, startnode, endnode)
        print(command(path,i_ori))
        goal_command=command(path,i_ori)
        confidence=distribution(startnode,endnode)
        print(confidence)

        out=correct_prob(startnode,endnode,goal_command)
        print("final prob distribution",out[0][0],"avg prob",out[0][1])

########################################################################

        # i_ori=DIR_TO_STR[self.agent_start_dir]  #initial orientation
        # confidence=self.distribution(startnode,endnode)
        # print("start end initial orientation confidence")
        # print(startnode,endnode,i_ori,confidence[0])
        # # probgoal=goalprob(endnode,maze)
        # if (confidence[0]==1):
        #     path = astar(maze, startnode, endnode)
        #     goal_command=self.command(path,i_ori)
        #     goal_pt=endnode
        #     print(self.command(path,i_ori))
        #     out=self.correct_prob(startnode,goal_pt,goal_command)
        #     print("final prob distribution",out[0][0],"avg prob",out[0][1])
        # probgoal=interneighbour(endnode,maze)
        # if(confidence[0]==0.3):
        #     probgoal=farneighbour(endnode,maze)
        # # maxgoal=predmaxprobgoal(endnode,maze)
        # nonrepgoal={}
        # count=1
        # # print(probgoal)
        # if probgoal == None:
        #     return
        # dst=[]
        # dd={}
        # for i in probgoal:
        #     if(notroom(i)):
        #       path = astar(maze, startnode, i)
        #       nonrepgoal[command(path,i_ori)] = distance(startnode,i)# with cutting 
        #       dst.append(distance(startnode,i))
        #       dd[distance(startnode,i)]=i

        #       count+=1

        # dst.sort()
        # # print(dst)
        # if(confidence[0]==0.75):
        #     if(len(dst)>2):
        #        ch=np.random.choice(dst[:(len(dst)/2)+1], 1)[0]
        #     else:
        #        ch=np.random.choice(dst, 1)[0]
        #     goal_command=get_key(ch,nonrepgoal)
        #     goal_pt=dd[ch]
        #     print("goal_command",goal_command,goal_pt)
        #     out=correct_prob(startnode,goal_pt,goal_command)
        #     print("final prob distribution",out[0][0],"avg prob",out[0][1])

        # if(confidence[0]==0.5):
        #     if(len(dst)>2):
        #        ch=np.random.choice(dst[(len(dst)/2)-1:], 1)[0]
        #     else:
        #         ch=np.random.choice(dst, 1)[0]
        #     goal_pt=dd[ch]
        #     goal_command=get_key(ch,nonrepgoal)
        #     print("goal_command",goal_command,goal_pt)
        #     out=correct_prob(startnode,goal_pt,goal_command)
        #     print("final prob distribution",out[0][0],"avg prob",out[0][1])

        # if(confidence[0]==0.3):
        #     ch=np.random.choice(dst, 1)[0]
        #     goal_pt=dd[ch]
        #     goal_command=get_key(ch,nonrepgoal)
        #     print("goal_command",goal_command,goal_pt)
        #     out=correct_prob(startnode,goal_pt,goal_command)
        #     print("final prob distribution",out[0][0],"avg prob",out[0][1])
        # print("All possible goal")
        # print("command goal prob/dist from start to goal")
        # for i in nonrepgoal:
        #             print(i,dd[nonrepgoal[i]],nonrepgoal[i])
        
        # # use goal_command for global command directions start symbol is the orientation change
        # # use out[0][0]) for correspondind prob distribution
        # print(out[0][0])
        # sym_pub.publish(str(goal_command))








############################################################################
        path = astar(maze, startnode, endnode)
        print(startnode)
        print(endnode)
        print(path)

        # Place the agent
        if self.agent_start_pos is not None:
            self.agent_pos = self.agent_start_pos
            self.agent_dir = self.agent_start_dir
        else:
            self.place_agent()

        # Place obstacles
        self.obstacles = []
        for i_obst in range(self.n_obstacles):
            self.obstacles.append(Ball())
            self.place_obj(self.obstacles[i_obst], max_tries=100)

        self.mission = "get to the green goal square using human instructions"

    # def _gen_grid(self, width, height):
    #     # Create an empty grid
    #     self.grid = Grid(width, height)

    #     # Generate the surrounding walls
    #     self.grid.wall_rect(0, 0, width, height)

    #     x = np.arange(4, 60, 8)
    #     # y = np.arange(4, 44, 10)
    #     y = np.arange(4, 60, 8)
    #     # xx, yy = np.meshgrid(x, y)
    #     # Generate rooms
    #     for i in range(0, len(x)):
    #         for j in range(0, len(y)):
    #             self.grid.wall_rect(x[i], y[j], 5, 5)


    #     # CHECK if GOAL lies inside room or on walls.
    #     gp=self.goal_pos
    #     while(self.check_room(gp) and self.check_mid(gp)):
    #         gp=(random.randint(1,59), random.randint(1,59))
    #     self.goal_pos=gp

    #     gp=self.agent_start_pos
    #     while(self.check_room(gp) and self.check_mid(gp)):
    #         gp=(random.randint(1,59), random.randint(1,59))
    #     self.agent_start_pos=gp
        

    #     # Place a goal square at a random location
    #     self.grid.set(self.goal_pos[0], self.goal_pos[1], Goal())

    #     # CHECK if AGENT lies inside room or on walls.
    #     # self.agent_start_pos !=:
        

    #     # Place the agent
    #     if self.agent_start_pos is not None:
    #         self.agent_pos = self.agent_start_pos
    #         self.agent_dir = self.agent_start_dir
    #     else:
    #         self.place_agent()

    #     # Place obstacles
    #     self.obstacles = []
    #     for i_obst in range(self.n_obstacles):
    #         self.obstacles.append(Ball())
    #         self.place_obj(self.obstacles[i_obst], max_tries=100)

    #     self.mission = "get to the green goal square using human instructions"


    def update_humans(self):
        for i_obst in range(len(self.obstacles)):
            old_pos = self.obstacles[i_obst].cur_pos
            top = tuple(map(add, old_pos, (-1, -1)))

            try:
                self.place_obj(self.obstacles[i_obst], top=top, size=(3,3), max_tries=100)
                self.grid.set(*old_pos, None)
            except:
                pass


    def check_human(self):
        # front_cell = self.grid.get(self.front_pos[0],self.front_pos[1])
        # front_cell = self.front_pos
        # print("front_cell",front_cell)
        # not_clear = False
        # front_cell = self.grid.get(*self.front_pos)
        # if front_cell!=None:
        #     print('ball found is ',front_cell.type)
        #     not_clear = front_cell.type == 'ball' and front_cell.type != 'goal'
        # not_clear = front_cell.type != 'goal'  and  front_cell.type ==  'ball'
        # 
        front_cell = self.grid.get(*self.front_pos)
        print("front cell is ",front_cell)
        not_clear = front_cell and front_cell.type != 'goal'
        if not_clear:
            return 1

        return 2



    # def check_gateway():
    #     # Write code for gateway point region detection


    #     return self.gateway_detect


    def check_end(self):
          # a=self.grid.get(*self.front_pos)
        a=self.agent_pos
        if(a[0]==59  or a[1]==59):
            return 1
        if(a[0]==1  or a[1]==1):
            return 1
        front_cell = self.grid.get(*self.front_pos)

        if front_cell!=None:
            if(front_cell.type == 'wall'):
                  return 1

        return 2



    def step(self, action):
        # Invalid action
        # if action not in self.action_space:
            # action = 0

        self.human_detect = False
        self.gateway_detect = False

        # Check if there is a human in front of the agent
        # front_cell = self.grid.get(*self.front_pos)
        # not_clear = front_cell and front_cell.type != 'goal'

        # # Update obstacle positions
        # for i_obst in range(len(self.obstacles)):
        #     old_pos = self.obstacles[i_obst].cur_pos
        #     top = tuple(map(add, old_pos, (-1, -1)))

        #     try:
        #         self.place_obj(self.obstacles[i_obst], top=top, size=(3,3), max_tries=100)
        #         self.grid.set(*old_pos, None)
        #     except:
        #         pass        

        # Update the agent's position/direction
        # obs, reward, done, info = MiniGridEnv.step(self, action)

        obs = generate_obs()

        # If the agent tried to walk over an obstacle or wall
        if action == self.actions.forward and not_clear:
            reward = -1
            done = True
            return obs, reward, done, info

        return obs, reward, done, info

class DynamicObstaclesEnv5x5(DynamicObstaclesEnv):
    def __init__(self):
        super().__init__(size=5, n_obstacles=2)

class DynamicObstaclesRandomEnv5x5(DynamicObstaclesEnv):
    def __init__(self):
        super().__init__(size=5, agent_start_pos=None, n_obstacles=2)

class DynamicObstaclesEnv6x6(DynamicObstaclesEnv):
    def __init__(self):
        super().__init__(size=6, n_obstacles=3)

class DynamicObstaclesRandomEnv6x6(DynamicObstaclesEnv):
    def __init__(self):
        super().__init__(size=6, agent_start_pos=None, n_obstacles=3)

class DynamicObstaclesEnv16x16(DynamicObstaclesEnv):
    def __init__(self):
        super().__init__(size=16, n_obstacles=8)



class DynamicObstaclesEnv30x30(DynamicObstaclesEnv):
    def __init__(self):
        super().__init__(size=61, n_obstacles=20, agent_start_pos=(1, 1), agent_start_dir=(random.randint(0,3)), goal_pos=(random.randint(1,62), random.randint(1,62)))




register(
    id='MiniGrid-Dynamic-Obstacles-5x5-v0',
    entry_point='gym_minigrid.envs:DynamicObstaclesEnv5x5'
)

register(
    id='MiniGrid-Dynamic-Obstacles-Random-5x5-v0',
    entry_point='gym_minigrid.envs:DynamicObstaclesRandomEnv5x5'
)

register(
    id='MiniGrid-Dynamic-Obstacles-6x6-v0',
    entry_point='gym_minigrid.envs:DynamicObstaclesEnv6x6'
)

register(
    id='MiniGrid-Dynamic-Obstacles-Random-6x6-v0',
    entry_point='gym_minigrid.envs:DynamicObstaclesRandomEnv6x6'
)

register(
    id='MiniGrid-Dynamic-Obstacles-8x8-v0',
    entry_point='gym_minigrid.envs:DynamicObstaclesEnv'
)

register(
    id='MiniGrid-Dynamic-Obstacles-16x16-v0',
    entry_point='gym_minigrid.envs:DynamicObstaclesEnv16x16'
)

register(
    id='MiniGrid-Dynamic-Obstacles-30x30-v0',
    entry_point='gym_minigrid.envs:DynamicObstaclesEnv30x30'
)
