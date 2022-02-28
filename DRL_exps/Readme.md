
# Deep Reinforcement Learning for Robot Trust

Contains code for learning Robot Trust using Deep RL.

# Steps to execute
## Prerequisites:

  1. Install Pytorch (Used 1.10.2) with Python 3.7.0

  2. Install Minigrid, and customize it as follows:
  
    1. ``` git clone https://github.com/maximecb/gym-minigrid.git ```

    2. ```cd /home/user/gym-minigrid/gym_minigrid/envs   ``` 
       replace the dynamicobstacles.py and add dynamic2.py , maze.py , gen_cmd.py , pathplan.py

    3. ```cd /home/user/gym-minigrid/gym_minigrid/ ``` 
       and replace minigrid.py
  
  3. Install minigrid using "python3 setup.py install".
##
  1. Run 'python3 train_final.py' to run train on a **Non Vecotrized** (single process) environment.
  2. Modify and run ``python3 vec_train.sh`` to run the vectorized environment.
