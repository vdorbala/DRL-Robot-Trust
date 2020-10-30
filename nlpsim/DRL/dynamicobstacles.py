from __future__ import division
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
        self.agent_start_pos = agent_start_pos
        self.agent_start_dir = agent_start_dir
        self.goal_pos = goal_pos
      
        # Number of cells (width and height) in the agent view
        # assert self.agent_view_size % 2 == 1
        # assert self.agent_view_size >= 3

        # self.agent_view_size = agent_view_size

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
        # Allow only 5 actions permitted: left (0), right (1), forward (2), backward(-1), and interact (7)
        self.action_space = spaces.Discrete(50) # L*10, R*10, U*10, D*10, I*10
        # self.action_space = [-1, 0, 1, 2, 7]

        print("Action space is {}".format(self.action_space))

        self.reward_range = (-200, 500)


        # self.observation_space = spaces.Tuple((spaces.MultiDiscrete([(0, 3), (0, 3), (0, 3), (0, 3)]), 
        #                                 spaces.Box(0,1,shape=(1,)),
        #                                 spaces.Discrete(10),
        #                                 spaces.Discrete(10),
        #                                 spaces.Discrete(2),
        #                                 spaces.Discrete(2)))


        # self.observation_space = spaces.Dict({
        #     'commands': spaces.MultiDiscrete([3, 3, 3, 3]),
        #     'confidence': spaces.Box(0,1,shape=(1,)),
        #     'icount': spaces.Discrete(10),
        #     'gcount': spaces.Discrete(10),
        #     'human_detect': spaces.Discrete(2),
        #     'gateway_detect': spaces.Discrete(2)
        # })

        self.h_l1_min = -4
        self.h_l1_max = 4

        self.h_l2_min = -4
        self.h_l2_max = 4

        self.h_l_min = 0
        self.h_l_max = 3

        self.h_c_min = 0.0
        self.h_c_max = 1.0

        self.NI_min = 0
        self.NI_max = 10

        self.NG_min = 0
        self.NG_max = 10

        self.DH_min = 0
        self.DH_max = 1

        self.DG_min = 0
        self.DG_max = 1

        self.n_observations = 9
        self.n_stacked_frames = 1


        low = np.array([self.h_l_min, self.h_l_min, self.h_l_min, self.h_l_min, self.h_c_min, self.NI_min, self.NG_min, self.DH_min, self.DG_min])
        high = np.array([self.h_l_max, self.h_l_max, self.h_l_max, self.h_l_max, self.h_c_max, self.NI_max, self.NG_max, self.DH_max, self.DG_max])

        self.observation_space = spaces.Box(low, high)


        self.commands = get_dir()

        self.confidence = get_conf()

        self.icount = 0

        self.gcount = 0

        self.human_detect = 0

        self.gateway_detect = 0

        self.IMAX = 4
        self.IMIN = 2

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

            elif (front_cell.type != 'goal'):
                  return 0
        return 2


    def generate_obs(self):
        """
        Generate the agent's view (partially observable, low-resolution encoding)
        """
        # print("Observations are {}, {}, {}, {}, {}, {}".format(self.commands, self.confidence, self.icount, self.gcount, self.human_detect, self.gateway_detect))
        # Observations are dictionaries containing:
        # - Human confidence levels
        # - Human language commands
        # - A count of number of gateways
        # - A count of the number of interactions
        # - If humans have been detected or not
        # - If intersections have been detected or not

        # obs = np.zeros(np.shape(self.observation_space))

        obs = np.hstack((self.commands, self.confidence, self.icount, self.gcount, self.human_detect, self.gateway_detect))

        # print("Shape of obs is {}. Obs is {}".format(np.shape(obs), obs))


        return obs

    def step(self, action):

        while((self.check_human()!=1) or (self.check_gateway() != 1)):
            pass

        print("Triggering new state now!")

        if action in np.arange(0,10):
            # Right Action
            tau_r = (action+1)/10
            cur_act = 0

        elif action in np.arange(10,20):
            # Down Action
            tau_r = (action-9)/10
            cur_act = 1

        elif action in np.arange(20,30):
            # Left Action
            tau_r = (action-19)/10
            cur_act = 2

        elif action in np.arange(30,40):
            # Up Action
            tau_r = (action-29)/10
            cur_act = 3

        else:
            # Interactive Action
            cur_act = 4
            tau_r = (action-39)/10



        # Reward Definitions:

        # Interactive Reward

        if (self.check_human() == 1) and (self.icount < self.IMIN) and (cur_act!=4):
            r_i = -100

        elif (self.check_human() != 1) and (cur_act == 4):
            r_i = -5

        else:
            r_i = 0


        # Target Reward

        front_cell = self.grid.get(*self.front_pos)

        if front_cell is not None:
            if 'goal' in front_cell:
                r_t = 500

            else:
                r_t = -200


        # Delay Penalty

        if front_cell is not None:
            if front_cell == 'goal':
                if (self.gcount == self.GMIN):
                    r_g = 200
                    done = True

                elif (self.gcount > self.GMIN)
                    r_g = -10*(self.gcount - self.GMIN)

                else:
                    r_g = 0

        # Social Reward

        if cur_act == self.commands[0]:
            temp = self.commands
            temp.pop(0)
            self.commands = np.array(temp)

            r_soc = 10

        elif self.confidence >= tau_r:
            r_soc = 50

        elif self.confidence < tau_r:
            r_soc = -100

        else:
            r_soc = 0


        reward = r_i + r_t + r_g + r_soc

        # Check if there is a human in front of the agent
        # front_cell = self.grid.get(*self.front_pos)
        # not_clear = front_cell and front_cell.type != 'goal'

        obs = self.generate_obs()
        reward = 0
        # If the agent tried to walk over an obstacle or wall
        if action == self.actions.forward:
            reward = -1000
            done = True

        print("Reward is {}".format(reward))

        info = {}

        return obs, reward, done, info

    # def reset(self):


    #     return np.zeros(np.shape(self.observation_space))


    def redraw(img):
        if not args.agent_view:
            img = self.render('rgb_array', tile_size=args.tile_size)

        self.window.show_img(img)

    def reset():
        if args.seed != -1:
            self.seed(args.seed)

        obs = self.reset()

        if hasattr(self, 'mission'):
            print('Mission: %s' % env.mission)
            self.window.set_caption(env.mission)

        redraw(obs)

class DynamicObstaclesEnv30x30(DynamicObstaclesEnv):
    def __init__(self):
        super().__init__(size=61, n_obstacles=20, agent_start_pos=(1, 1), agent_start_dir=(random.randint(0,3)), goal_pos=(random.randint(1,62), random.randint(1,62)))

register(
    id='MiniGrid-Dynamic-Obstacles-30x30-v0',
    entry_point='gym_minigrid.envs:DynamicObstaclesEnv30x30'
)