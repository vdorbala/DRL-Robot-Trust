from gym_minigrid.minigrid import *
from gym_minigrid.register import register
from operator import add
import random
import numpy as np

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
        # Allow only 3 actions permitted: left, right, forward
        self.action_space = spaces.Discrete(self.actions.forward + 1)
        self.reward_range = (-1, 1)


    def _gen_grid(self, width, height):
        # Create an empty grid
        self.grid = Grid(width, height)

        # Generate the surrounding walls
        self.grid.wall_rect(0, 0, width, height)

        x = np.arange(4, 23, 8)
        y = np.arange(4, 23, 8)
        # xx, yy = np.meshgrid(x, y)
        # Generate rooms
        for i in range(0, len(x)):
            for j in range(0, len(y)):
                self.grid.wall_rect(x[i], y[j], 5, 5)


        # CHECK if GOAL lies inside room or on walls.
        self.goal_pos

        # Place a goal square at a random location
        self.grid.set(self.goal_pos[0], self.goal_pos[1], Goal())

        # CHECK if AGENT lies inside room or on walls.
        self.agent_start_pos !=:
        

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

        self.mission = "get to the green goal square"



    def step(self, action):
        # Invalid action
        if action >= self.action_space.n:
            action = 0

        # Check if there is an obstacle in front of the agent
        front_cell = self.grid.get(*self.front_pos)
        not_clear = front_cell and front_cell.type != 'goal'

        # Update obstacle positions
        for i_obst in range(len(self.obstacles)):
            old_pos = self.obstacles[i_obst].cur_pos
            top = tuple(map(add, old_pos, (-1, -1)))

            try:
                self.place_obj(self.obstacles[i_obst], top=top, size=(3,3), max_tries=100)
                self.grid.set(*old_pos, None)
            except:
                pass

        # Update the agent's position/direction
        obs, reward, done, info = MiniGridEnv.step(self, action)

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
        super().__init__(size=30, n_obstacles=10, agent_start_pos=(random.randint(1,28), random.randint(1,28)), agent_start_dir=(random.randint(0,3)), goal_pos=(random.randint(1,28), random.randint(1,28)))




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
