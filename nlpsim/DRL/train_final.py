#!/usr/bin/env python3
import argparse
import time
import datetime

from stable_baselines.common.policies import FeedForwardPolicy, register_policy
from stable_baselines.common.env_checker import check_env

# from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
import tensorboardX
import sys
import torch

import numpy as np
import gym
import gym_minigrid
from gym_minigrid.wrappers import *
from gym_minigrid.window import Window

import logging
import os

# Parse arguments

class CustomPolicy(FeedForwardPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomPolicy, self).__init__(*args, **kwargs,
                                           net_arch=[dict(pi=[144, 144, 144],
                                                          vf=[144, 144, 144])],
                                           feature_extraction="mlp")


#Utility Functions

def create_folders_if_necessary(path):
    dirname = os.path.dirname(path)
    if not os.path.isdir(dirname):
        os.makedirs(dirname)


def get_status_path(model_dir):
    return os.path.join(model_dir, "status.pt")


def get_status(model_dir):
    path = get_status_path(model_dir)
    return torch.load(path)

def make_env(env_key, seed=None):
    env = gym.make(env_key)
    env.seed(seed)
    return env

def get_storage_dir():
    if "RL_STORAGE" in os.environ:
        return os.environ["RL_STORAGE"]
    return "storage"


def get_txt_logger(model_dir):
    path = os.path.join(model_dir, "log.txt")
    create_folders_if_necessary(path)

    logging.basicConfig(
        level=logging.INFO,
        format="%(message)s",
        handlers=[
            logging.FileHandler(filename=path),
            logging.StreamHandler(sys.stdout)
        ]
    )

    return logging.getLogger()

def create_folders_if_necessary(path):
    dirname = os.path.dirname(path)
    if not os.path.isdir(dirname):
        os.makedirs(dirname)

def get_model_dir(model_name):
    return os.path.join(get_storage_dir(), model_name)


def train():

    # Set run dir
    date = datetime.datetime.now().strftime("%y-%m-%d-%H-%M-%S")
    default_model_name = "hricra_{}".format(date)

    model_name = default_model_name
    model_dir = get_model_dir(model_name)

    # Load Tensorboard writer
    tb_writer = tensorboardX.SummaryWriter(model_dir)
    txt_logger = get_txt_logger(model_dir)

    # Set device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    # Load environments

    env = 'MiniGrid-Dynamic-Obstacles-40x40-v0'

    envs = []
    for i in range(1):
        envs.append(make_env(env, 1 + 10000 * i))
    txt_logger.info("Environments loaded\n")

    # Load training status
    try:
        status = get_status(model_dir)
    except OSError:
        status = {"num_frames": 0, "update": 0}
    txt_logger.info("Training status loaded\n")

    # Load observations preprocessor

    obs_space = envs[0].observation_space
    txt_logger.info("Observations preprocessor loaded")

    # print("value is {}".format(envs[0].observation_space['human_detect']))

    print("Observation space is ! {}".format(obs_space))

    # Load algo
    algo = PPO2(CustomPolicy, envs[0], n_steps=10, ent_coef=0.01, learning_rate=0.01, nminibatches=5, tensorboard_log="./logs/", verbose=1)
    algo.learn(total_timesteps=10)
    algo.save("hricra_{}".format(time.time()))


if __name__ == '__main__':
    train()
