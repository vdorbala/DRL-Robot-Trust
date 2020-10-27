from gym_minigrid.minigrid import *
from gym_minigrid.register import register
from operator import add
import random
import numpy as np

from gym_minigrid.envs.gen_cmd import get_dir,get_conf

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


    def check_room(self, gp):
        x = np.arange(4, 60, 8)
        # y = np.arange(4, 60, 8)

        for i in x:
            # for j in y:
              if(i<=gp[0]<i+5) and (i<=gp[1]<i+5):
                   return True
        return False

    def check_mid(self, gp):
        if(gp[0] in inter_x) or (gp[1] in inter_x) :
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
        while(self.check_room(gp) and self.check_mid(gp)):
            gp=(random.randint(1,59), random.randint(1,59))
        self.goal_pos=gp

        gp=self.agent_start_pos
        while(self.check_room(gp) and self.check_mid(gp)):
            gp=(random.randint(1,59), random.randint(1,59))
        self.agent_start_pos=gp
        

        # Place a goal square at a random location
        self.grid.set(self.goal_pos[0], self.goal_pos[1], Goal())

        # CHECK if AGENT lies inside room or on walls.
        # self.agent_start_pos !=:
        

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
        front_cell = self.grid.get(*self.front_pos)
        not_clear = front_cell and front_cell.type != 'goal'
        if not_clear:
            self.human_detect = True

        return self.human_detect



    def check_gateway():
        # Write code for gateway point region detection

        return self.gateway_detect



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
        super().__init__(size=61, n_obstacles=20, agent_start_pos=(random.randint(1,62), random.randint(1,62)), agent_start_dir=(random.randint(0,3)), goal_pos=(random.randint(1,62), random.randint(1,62)))




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
