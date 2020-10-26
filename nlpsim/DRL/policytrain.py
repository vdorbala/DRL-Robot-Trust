#!/usr/bin/env python3

import time
import argparse
import numpy as np
import gym
import gym_minigrid
from gym_minigrid.wrappers import *
from gym_minigrid.window import Window

import rospy


def redraw(img):
    if not args.agent_view:
        img = env.render('rgb_array', tile_size=args.tile_size)

    window.show_img(img)

def reset():
    if args.seed != -1:
        env.seed(args.seed)

    obs = env.reset()

    if hasattr(env, 'mission'):
        print('Mission: %s' % env.mission)
        window.set_caption(env.mission)

    redraw(obs)

def step(action):
    obs, reward, done, info = env.step(action)
    print('step=%s, reward=%.2f' % (env.step_count, reward))

    if done:
        print('done!')
        reset()
    else:
        redraw(obs)

def key_handler(event):
    print('pressed', event.key)

    if event.key == 'escape':
        window.close()
        return

    if event.key == 'backspace':
        reset()
        return

    if event.key == 'left':
        step(env.actions.left)
        return
    if event.key == 'right':
        step(env.actions.right)
        return
    if event.key == 'up':
        step(env.actions.forward)
        return

    # Spacebar
    if event.key == ' ':
        step(env.actions.toggle)
        return
    if event.key == 'pageup':
        step(env.actions.pickup)
        return
    if event.key == 'pagedown':
        step(env.actions.drop)
        return

    if event.key == 'enter':
        step(env.actions.done)
        return

parser = argparse.ArgumentParser()
parser.add_argument(
    "--env",
    help="gym environment to load",
    default='MiniGrid-Dynamic-Obstacles-30x30-v0'
)
parser.add_argument(
    "--seed",
    type=int,
    help="random seed to generate the environment with",
    default=20
)
parser.add_argument(
    "--tile_size",
    type=int,
    help="size at which to render tiles",
    default=32
)
parser.add_argument(
    '--agent_view',
    default=False,
    help="draw the agent sees (partially observable view)",
    action='store_true'
)


def move(act):
    reward = 0
    done = False

    # Get the position in front of the agent
    fwd_pos = env.front_pos

    # Get the postion behind the agent
    back_pos = env.back_pos

    # Get the contents of the cell in front of the agent
    fwd_cell = env.grid.get(*fwd_pos)

    # Get the contents of the cell behind the agent
    back_cell = env.grid.get(*back_pos)

    # Rotate left
    if act == 0:
        env.agent_dir -= 1
        if env.agent_dir < 0:
            env.agent_dir += 4

    # Rotate right
    elif act == 1:
        env.agent_dir = (env.agent_dir + 1) % 4

    # Move forward
    elif act == 2:
        if fwd_cell == None or fwd_cell.can_overlap():
            env.agent_pos = fwd_pos
        if fwd_cell != None and fwd_cell.type == 'goal':
            done = True
            reward = env._reward()


    # Move backward
    elif act == 3:
        if back_cell == None or back_cell.can_overlap():
            env.agent_pos = back_pos
        if back_cell != None and back_cell.type == 'goal':
            done = True
            reward = self._reward()

    return done

def event_control(env):

    env.render('human')
    
    while True:
        env.render('human')
        env.update_humans()
        if (env.human_detect == True) or (env.gateway_detect == True):
            step(env.actions.forward)
        else:
            print("Exploring till I find gateway or human!")
            done = move(np.random.randint(0,3))

            if env.window.closed:
                break
        
        print(done)

    return 0
            # env.agent_pos = env.front_pos
        # if rospy.get_param("/")
        # pass

if __name__ == '__main__':

    human_detect = False

    gateway_detect = False

    args = parser.parse_args()

    env = gym.make(args.env)

    # event_ros = rospy.init_node('drl', anonymous=True)

    # if args.agent_view:
    #     env = RGBImgPartialObsWrapper(env)
    #     env = ImgObsWrapper(env)

    # window = Window('gym_minigrid - ' + args.env)
    # # window.reg_key_handler(key_handler)

    # reset()

    event_control(env)