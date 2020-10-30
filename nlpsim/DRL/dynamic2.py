from __future__ import division
from gym_minigrid.minigrid import *
from gym_minigrid.register import register
from operator import add
import random
import numpy as np
from gym_minigrid.envs.gen_cmd import get_command
import time
import csv


# from gym_minigrid.envs.gen_cmd import get_dir,get_conf
from stable_baselines.common.callbacks import BaseCallback

from gym_minigrid.envs.maze import maze, m2g, gazebo, inter_x, mid_x, mx
from gym_minigrid.envs.pathplan import astar
# from gym_minigrid.envs.pos import check_room,check_mid

from collections import defaultdict 


class DynamicObstaclesEnv2(MiniGridEnv):
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
        self.mincommand = 1
      
        # Number of cells (width and height) in the agent view
        # assert self.agent_view_size % 2 == 1
        # assert self.agent_view_size >= 3

        # self.agent_view_size = agent_view_size

        # Reduce obstacles if there are too many
        if n_obstacles <= size/2 + 1:
            self.n_obstacles = int(n_obstacles)
        else:
            self.n_obstacles = int(size/2)

        # Allow only 5 actions permitted: left (0), right (1), forward (2), backward(-1), and interact (7)

        # self.action_space = [-1, 0, 1, 2, 7]

        print("Action space is {}".format(self.action_space))

        self.reward_range = (-200, 500)

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
        self.NG_max = 15

        self.DH_min = 0
        self.DH_max = 1

        self.DG_min = 0
        self.DG_max = 1

        self.n_observations = 9
        self.n_stacked_frames = 1


        self.distance_min = 0
        self.distance_max = 100

        low = np.array([self.h_l_min, self.h_l_min, self.h_l_min, self.h_l_min, self.h_c_min, self.NI_min, self.NG_min, self.DH_min, self.DG_min, self.distance_min])
        high = np.array([self.h_l_max, self.h_l_max, self.h_l_max, self.h_l_max, self.h_c_max, self.NI_max, self.NG_max, self.DH_max, self.DG_max, self.distance_max])


        self.time_elapse = 0

        self.commands = [10, 10, 10, 10]

        self.confidence = 0.0

        self.icount = 0

        self.gcount = 0

        self.TIMENUM = 3

        self.human_detect = 0

        self.gateway_detect = 0

        self.IMAX = 4
        self.IMIN = 2

        self.GMAX= 7
        # self.GMAX = 15
        self.GMIN = 3

        self.r_t = 0

        self.r_i = 0

        self.r_g = 0

        self.r_soc = 0

        self.r_wall = 0

        self.r_dist = 0

        self.r_conf = 0

        super().__init__(
            grid_size=size,
            max_steps=4 * size * size,
            # Set this to True for maximum speed
            see_through_walls=True,
        )

        self.observation_space = spaces.Box(low, high)

        self.carrying = None

        self.reward_range = (-2000, 2000)

        self.action_space = spaces.Discrete(50) # L*10, R*10, U*10, D*10, I*10


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

    @property
    def left_pos(self):
        """
        Get the position of the cell that is right in front of the agent
        """

        return self.agent_pos - self.right_vec


    @property
    def right_pos(self):
        """
        Get the position of the cell that is right in front of the agent
        """

        return self.agent_pos + self.right_vec


    def init_cmd(self):
        self.commands = [10, 10, 10, 10]


    # def check_mid(self, gp):
    #     if(gp[0] in inter_x) or (gp[1] in inter_x) :
    #             return True
    #     return False

    def check_room(self, gp):
        x = np.arange(4, 40, 8)
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
        # if((gp[0] in mx) and (gp[1] in mx)):
                # return True
        # return False
        return True


    def check_spawn(self):
        gp=self.goal_pos


        while not self.check_mid(gp):
            gp=(random.randint(1,43), random.randint(1,43))
        # while(self.check_room(gp)):
        #     gp=(random.randint(1,59), random.randint(1,59))
        self.goal_pos=gp

        gp=self.agent_start_pos
        while not self.check_mid(gp):
            gp=(random.randint(1,43), random.randint(1,43))
        # while(self.check_room(gp)):
        #     gp=(random.randint(1,59), random.randint(1,59))
        self.agent_start_pos=gp
        return True


    def _gen_grid(self, width, height):
        # Create an empty grid
        self.grid = Grid(width, height)

        # Generate the surrounding walls
        self.grid.wall_rect(0, 0, width, height)

        x = np.arange(4, 40, 8)
        # y = np.arange(4, 44, 10)
        y = np.arange(4, 40, 8)
        # xx, yy = np.meshgrid(x, y)
        # Generate rooms
        for i in range(0, len(x)):
            for j in range(0, len(y)):
                self.grid.wall_rect(x[i], y[j], 5, 5)


        # CHECK if GOAL lies inside room or on walls.
        while True:
            self.check_spawn()
            if self.agent_start_pos != self.goal_pos:
                break
            else:
                self.agent_start_pos=(random.randint(1,43), random.randint(1,43))
                self.goal_pos=(random.randint(1,43), random.randint(1,43))

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
            r=[]
            try:
              f = self.grid.get(*self.front_pos)
              r.append(f)
            except:
                pass
            try:
              re = self.grid.get(*self.right_pos)
              r.append(re)
            except:
                pass
            try:
              l = self.grid.get(*self.left_pos)
              r.append(l)
            except:
                pass

            try:
              fr = self.grid.get(*self.fr_pos)
              r.append(fr)
            except:
                pass
            try:
              fl = self.grid.get(*self.fl_pos)
              r.append(fl)
            except:
                pass

            try:
              sf = self.grid.get(*self.sf_pos)
              r.append(sf)
            except:
                pass
            try:
              sr = self.grid.get(*self.sr_pos)
              r.append(sr)
            except:
                pass
            try:
              sl = self.grid.get(*self.sl_pos)
              r.append(sl)
            except:
                pass
            try:
              tf = self.grid.get(*self.tf_pos)
              r.append(tf)
            except:
                pass
            try:
              tr = self.grid.get(*self.tr_pos)
              r.append(tr)
            except:
                pass
            try:
              tl = self.grid.get(*self.tl_pos)
              r.append(tl)
            except:
                pass
            if len(r)<1:
                return 2
            for i in r:
                if(i is not None):
                    if(i.type == 'ball'):
                        self.human_detect = 1
                        return 1
            self.human_detect = 0
            return 2
    # def check_human(self):

    #     front_cell = self.grid.get(*self.front_pos)
    #     not_clear = front_cell and front_cell.type != 'goal'
    #     if front_cell is not None:
    #         if(front_cell.type == 'ball'):
    #            self.human_detect = 1
    #            return 1
    #     self.human_detect = 0
    #     return 2

    def check_inter(self,a,b):
        if ((a[0] in b) and (a[1] in b)):
                 return 1
        return 2

    def check_gateway(self):
        a=self.agent_pos
        value = self.check_inter(a, mid_x)
        return value

        x = inter_x[0:-1:2]
        for i in x:
          if(i-1<=a[0]<i+2):
                  for j in x:
                      if (j-1<=a[1]<j+2):
                        if self.gateway_detect == 0:
                            self.gateway_detect = 1
                            self.gcount = self.gcount + 1
                            return 1

        self.gateway_detect = 0
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

            elif (front_cell.type == 'goal'):
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

        self.distance = np.linalg.norm(np.array(self.goal_pos) - np.array(self.agent_pos))

        obs = np.hstack((self.commands, self.confidence, self.icount, self.gcount, self.human_detect, self.gateway_detect, self.distance))

        # print("Shape of obs is {}. Obs is {}".format(np.shape(obs), obs))

        return obs



    def move(self, act):
        reward = 0
        done = False

        # Get the position in front of the agent
        fwd_pos = super().front_pos

        # Get the postion behind the agent
        back_pos = super().back_pos

        # Get the contents of the cell in front of the agent
        fwd_cell = self.grid.get(*fwd_pos)

        # Get the contents of the cell behind the agent
        back_cell = self.grid.get(*back_pos)

        # Rotate left
        if act == 0:
            self.agent_dir -= 1
            if self.agent_dir < 0:
                self.agent_dir += 4

        # Rotate right
        elif act == 1:
            self.agent_dir = (self.agent_dir + 1) % 4

        # Move forward
        elif act == 2:
            if fwd_cell == None or fwd_cell.can_overlap():
                self.agent_pos = fwd_pos
            if fwd_cell != None and fwd_cell.type == 'goal':
                done = True

        # Move backward
        elif act == 3:
            if back_cell == None or back_cell.can_overlap():
                self.agent_pos = back_pos
            if back_cell != None and back_cell.type == 'goal':
                done = True

        return done


    def goal_check(self):

        self.distance = np.linalg.norm(np.array(self.goal_pos) - np.array(self.agent_pos))

        if self.distance <= 1.5:
            # WHAT WORKS: Test 1 - 300
            self.r_t += 0.3
            # print("Reached Goal. Carrying out step and ending episode.")
            return True
        else:
            # WHAT WORKS: Test 1 - 0
            # front_cell = self.grid.get(*self.front_pos)
            # if front_cell is not None:
            #     if front_cell.type == 'wall':
            #         self.r_wall += -1
            return False


    def event_control(self):
        flag = False
        done = False
        checkval = False
        self.update_humans()

        done = self.move(2)
        checkval = self.goal_check()
        done = self.move(2)
        checkval = self.goal_check()

        return checkval


    def emulate_check_human(self):
        # Function for emulating human found at intersection.
        # self.gcount -= 1
        self.human_detect = 1
        # self.icount += 1

        return True

    # def get_min_cmd(self):
        # self.GMAX=get_command(self.agent_start_pos, self.agent_start_dir,self.goal_pos)[2]

    def step(self, action):

        done = False
        checkval = False
        time_init = time.time()
        while True:
            # print("Time elapse now is {}".format(self.time_elapse))
            self.render('human')
            self.update_humans()

            if self.check_human() == 1:
                print("Found Human!")
                # if self.time_elapse > self.TIMENUM:
                break
            if self.check_gateway() == 1:
                # if (self.gcount % 4)==0 and (self.icount==0):
                    # retval = self.emulate_check_human()
                # if self.time_elapse > self.TIMENUM:
                break
            
            if (time.time()- time_init)>10:
                print("Ending Episode due to time!")
                obs = self.generate_obs()
                info = {"confval": 0.0, "distance": self.distance}
                # WHAT WORKS: Test 1 - -100
                return obs, -0.1, True, info

            checkval = self.event_control()
            if checkval == True:
                action = 100
                break
            # elif checkval==False:
            #     action = 200
            #     break
            # print("Waiting for state change")

        self.r_g = 0

        if action in np.arange(0,10):
            # Right Action
            tau_r = (action+1)/10
            cur_act = 0
            self.move(1)
            self.move(2)
            self.move(2)

        elif action in np.arange(10,20):
            # Down Action
            tau_r = (action-9)/10
            cur_act = 1
            self.move(1)
            self.move(1)
            self.move(2)

        elif action in np.arange(20,30):
            # Left Action
            tau_r = (action-19)/10
            cur_act = 2
            self.move(0)
            self.move(2)
            self.move(2)

        elif action in np.arange(30,40):
            # Up Action
            tau_r = (action-29)/10
            cur_act = 3
            self.move(2)
            self.move(2)


        elif action in np.arange(40,50):
            # Interactive Action
            cur_act = 4
            tau_r = (action-39)/10


        else:
            cur_act = 5
            print("Reached Goal!")
            done = True
            tau_r = 0.0
        # if front_cell is not None:
        #     if front_cell == 'goal' or cur_cell=='goal':
            if (self.gcount <= self.GMIN):
                # WHAT WORKS: Test 1 - 300
                self.r_g += 0.3

            elif (self.gcount > self.GMIN):
                # WHAT WORKS: Test 1 - -5
                self.r_g += -0.005*(self.gcount - self.GMIN)

            else:
                # WHAT WORKS: Test 1 - 0
                self.r_g += 0

            print("Optimal goal reward is {}".format(self.r_g))

        # else:
        #     cur_act = 6
        #     tau_r = 0.0
        #     # print("WALL!")
        #     self.r_wall += -1
        #     done = True


        act_dic = {0:"RIGHT", 1: "DOWN", 2:"LEFT", 3:"UP", 4: "INTERACTION!", 5: "Reached Goal", 6: "Wall"}

        print("Executed action {}".format(str(act_dic[cur_act])))

        # Reward Definitions:

        # Interactive Reward

        if (self.human_detect == 1) and (self.icount < self.IMIN) and (cur_act!=4):
            print("COULDVE INTERACTED BUT DID NOT! BOOHOO!")
            # WHAT WORKS: Test 1 - -200
            self.r_i += -0.2

        elif (self.human_detect != 1) and (cur_act == 4):
            print("WRONG ACTION AT INTERSECTION!")
            # WHAT WORKS: Test 1 - -200
            self.r_i += -0.2

        elif (self.human_detect == 1) and (cur_act == 4):
            self.commands = get_command(self.agent_pos, self.agent_dir,self.goal_pos)[0]
            self.confidence = 0.3 #get_command(self.agent_pos, self.agent_dir,self.goal_pos)[1]
            self.icount = self.icount + 1
            # Length check
            if len(self.commands)<4:
                while (4 - len(self.commands))>0:
                    self.commands = np.append(self.commands, 10)
            if len(self.commands)>4:
                self.commands = self.commands[:4]

            # WHAT WORKS: Test 1 - 1000
            self.r_i += 1
            self.gcount = 0
        else:
            # WHAT WORKS: Test 1 - 0
            self.r_i += 0

        # Target Reward defined in goal reach function

        # Delay Penalty

        # if self.gcount>(self.GMAX)/4:
            # self.r_g += -25
        # elif self.gcount>(self.GMAX)/2:
            # self.r_g += -50

        if (self.icount + self.gcount)>(self.GMAX -3):
            print("Ending Episode due to too many intersections!")
            # obs = self.generate_obs()
            # WHAT WORKS: Test 1 - -500
            self.r_g += -0.5
            done = True
            # return obs, -50, True, {}

        # Social Reward
        # WHAT WORKS: Test 1 - 0
        self.r_conf = 0

        if cur_act == self.commands[0]:
            temp = list(self.commands)
            temp.pop(0)
            temp.append(10)
            self.commands = np.array(temp)

            # WHAT WORKS: Test 1 - 1000
            self.r_soc += 1

            if self.confidence >= tau_r:
                # WHAT WORKS: Test 1 - 1000
                self.r_conf += 0.8

            if self.confidence < tau_r:
                # WHAT WORKS: Test 1 - -500
                self.r_conf += -0.3

        else:
            # WHAT WORKS: Test 1 - 0
            self.r_soc += 0

        # Check if there is a human in front of the agent
        # front_cell = self.grid.get(*self.front_pos)
        # not_clear = front_cell and front_cell.type != 'goal'

        # If the agent tried to walk over an obstacle or wall
        # front_cell = self.grid.get(*self.front_pos)
        # if front_cell is not None:
        #     if cur_act == 3 and front_cell.type == 'wall':
        #         # WHAT WORKS: Test 1 - -1000


        # Distance Reward
        self.distance = np.linalg.norm(np.array(self.goal_pos) - np.array(self.agent_pos))

        # WHAT WORKS: Test 1 - None
        # self.r_dist += -self.distance*0.5


        reward = self.r_i + self.r_t + self.r_g + self.r_soc + self.r_conf + self.r_wall #+ self.r_dist

        obs = self.generate_obs()

        self.time_elapse = 0
        # print("Time elapse now is {}".format(self.time_elapse))
        print("Rewards are r_i: {}, r_t: {}, r_g: {}, r_soc: {}, r_conf: {} total = {}".format(self.r_i, self.r_t, self.r_g, self.r_soc, self.r_conf, reward))
        # print("Robot Trust metric is {}".format(tau_r))
        print("Observation is {}".format(obs))

        # WHAT WORKS: Test 1 - -2000
        if reward<-2:
            print("Wow. Very bad bot.")
            done = True

        info = {"confval": tau_r, "distance": self.distance}

        return obs, reward, done, info


    def redraw(self, img):

        self.window.show_img(img)

    def reset(self):
        self.seed(np.random.randint(0,10000))

        # self.init_cmd()

        self.agent_start_pos = (2,42) #(random.randint(1,59), random.randint(1,59))

        self.goal_pos = (42,2) #(random.randint(1,59), random.randint(1,59))

        self._gen_grid(self.width, self.height)

        self.commands = [10, 10, 10, 10]

        self.confidence = 0.0

        self.icount = 0

        self.gcount = 0

        self.TIMENUM = 3

        self.human_detect = 0

        self.gateway_detect = 0

        self.IMAX = 4
        self.IMIN = 2

        self.GMAX=get_command(self.agent_start_pos, self.agent_start_dir,self.goal_pos)[2] + 5
        # self.GMAX = 15
        self.GMIN = get_command(self.agent_start_pos, self.agent_start_dir,self.goal_pos)[2] + 2

        self.r_t = 0

        self.r_i = 0

        self.r_g = 0

        self.r_soc = 0

        self.r_wall = 0

        self.r_dist = 0

        self.r_conf = 0


        # # These fields should be defined by _gen_grid
        # assert self.agent_pos is not None
        # assert self.agent_dir is not None

        # # Check that the agent doesn't overlap with an object
        # start_cell = self.grid.get(*self.agent_pos)
        # assert start_cell is None or start_cell.can_overlap()


        obs = self.generate_obs()
        # self.redraw()

        return obs