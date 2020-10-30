#!/usr/bin/env python3
import argparse
import time
import datetime

from stable_baselines.common.policies import FeedForwardPolicy, register_policy
from stable_baselines.common.env_checker import check_env

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines.common import set_global_seeds

from stable_baselines.common.evaluation import evaluate_policy

# from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
import tensorboardX
import sys
import torch

from stable_baselines.common.callbacks import BaseCallback
import tensorflow as tf


import time
import matplotlib.pyplot as plt
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


def make_env(env_id, rank, seed=0):
    """
    Utility function for multiprocessed env.
    
    :param env_id: (str) the environment ID
    :param seed: (int) the inital seed for RNG
    :param rank: (int) index of the subprocess
    """
    def _init():
        env = gym.make(env_id)
        # Important: use a different seed for each environment
        env.seed(seed + rank)
        return env
    set_global_seeds(seed)
    return _init

class TensorboardCallback(BaseCallback):
    """
    Custom callback for plotting additional values in tensorboard.
    """
    def __init__(self, verbose=0):
        self.is_tb_set = False
        super(TensorboardCallback, self).__init__(verbose)

    def _on_step(self) -> bool:
        # Log additional tensor
        confval = 0
        for i in range(0,16):
            confval += self.locals['infos'][i]['confval']

        confval = (confval/16)

        distance = 0
        for i in range(0,16):
            distance += self.locals['infos'][i]['distance']

        distance = (distance/16)
        # confval = self.locals['infos'][0]['confval']
        # distance = self.locals['infos'][0]['distance']
        # print("Conf values was {}".format(confval))
        # if not self.is_tb_set:
        #     with self.model.graph.as_default():
        #         tf.summary.scalar('value_target', tf.reduce_mean(self.model.value_target))
        #         self.model.summary = tf.summary.merge_all()
        #     self.is_tb_set = True
        # Log scalar value (here a random variable)
        value = confval
        summary = tf.Summary(value=[tf.Summary.Value(tag='confidence_score', simple_value=value)])
        self.locals['writer'].add_summary(summary, self.num_timesteps)
        
        value = distance
        summary = tf.Summary(value=[tf.Summary.Value(tag='distance', simple_value=value)])
        self.locals['writer'].add_summary(summary, self.num_timesteps)

        return True



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

    env_id = 'MiniGrid-Dynamic-Obstacles-40x40-v0'

    # envs = []
    # for i in range(1):
    #     envs.append(make_env(env, 1 + 10000 * i))
    # txt_logger.info("Environments loaded\n")

    # Load training status
    try:
        status = get_status(model_dir)
    except OSError:
        status = {"num_frames": 0, "update": 0}
    txt_logger.info("Training status loaded\n")

    # Load observations preprocessor
    # env = make_env(env_id, 1 + 10000)

    # obs_space = env.observation_space
    # txt_logger.info("Observations preprocessor loaded")

    # print("Observation space is ! {}".format(obs_space))


    # The different number of processes that will be used
    PROCESSES_TO_TEST = [16] 
    NUM_EXPERIMENTS = 1 # RL algorithms can often be unstable, so we run several experiments (see https://arxiv.org/abs/1709.06560)
    TRAIN_STEPS = 10000
    # Number of episodes for evaluation
    EVAL_EPS = 100
    ALGO = PPO2

    eval_env = gym.make(env_id)

    reward_averages = []
    reward_std = []
    training_times = []
    total_procs = 0
    for n_procs in PROCESSES_TO_TEST:
        total_procs += n_procs

        train_env = SubprocVecEnv([make_env(env_id, i+total_procs) for i in range(n_procs)], start_method='spawn')

    # print("value is {}".format(envs[0].observation_space['human_detect']))

    rewards = []
    times = []

    for experiment in range(NUM_EXPERIMENTS):
        # it is recommended to run several experiments due to variability in results
        train_env.reset()
        model = ALGO('MlpPolicy', train_env, n_steps=15, nminibatches=5, learning_rate=0.1, tensorboard_log="./vector_logs_new1/", verbose=1)
        start = time.time()
        model.learn(total_timesteps=TRAIN_STEPS, callback=TensorboardCallback())
        times.append(time.time() - start)
        mean_reward, _  = evaluate_policy(model, eval_env, n_eval_episodes=EVAL_EPS)
        rewards.append(mean_reward)
    # Important: when using subprocess, don't forget to close them
    # otherwise, you may have memory issues when running a lot of experiments
    train_env.close()
    reward_averages.append(np.mean(rewards))
    reward_std.append(np.std(rewards))
    training_times.append(np.mean(times))
    model.save("hricra_train2_{}".format(time.time()))

    training_steps_per_second = [TRAIN_STEPS / t for t in training_times]

    plt.figure(figsize=(9, 4))
    plt.subplots_adjust(wspace=0.5)
    plt.subplot(1, 2, 1)
    plt.errorbar(PROCESSES_TO_TEST, reward_averages, yerr=reward_std, capsize=2)
    plt.xlabel('Processes')
    plt.ylabel('Average return')
    plt.subplot(1, 2, 2)
    plt.bar(range(len(PROCESSES_TO_TEST)), training_steps_per_second)
    plt.xticks(range(len(PROCESSES_TO_TEST)), PROCESSES_TO_TEST)
    plt.xlabel('Processes')
    _ = plt.ylabel('Training steps per second')


    # # Load algo
    # algo = PPO2(CustomPolicy, env, n_steps=15, ent_coef=0.01, learning_rate=0.01, nminibatches=5, tensorboard_log="./logs/", verbose=1)
    # algo.learn(total_timesteps=10000)
    # algo.save("hricra_{}".format(time.time()))


if __name__ == '__main__':
    train()