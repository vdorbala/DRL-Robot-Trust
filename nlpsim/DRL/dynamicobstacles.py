from __future__ import division
from gym_minigrid.minigrid import *
from gym_minigrid.register import register
from operator import add
import random
import numpy as np
from gym_minigrid.envs.gen_cmd import get_command

# from gym_minigrid.envs.gen_cmd import get_dir,get_conf


from gym_minigrid.envs.maze import maze,m2g, gazebo ,inter_x,mid_x
from gym_minigrid.envs.pathplan import astar
# from gym_minigrid.envs.pos import check_room,check_mid

from collections import defaultdict 


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


        self.commands = [10, 10, 10, 10]

        self.confidence = 0.0

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

    def init_cmd(self):
        self.commands = [10, 10, 10, 10]

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

        elif (self.check_human() == 1) and (cur_act == 4):
            self.commands = get_command(self.agent_pos, self.agent_dir,self.goal_pos)[0]
            self.confidence = get_command(self.agent_pos, self.agent_dir,self.goal_pos)[1]
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
        print("Observation is {}".format(obs))

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