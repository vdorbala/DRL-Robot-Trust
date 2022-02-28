# Can a Robot Trust You?: A DRL-Based Approach to Trust-Driven Human-Guided Navigation

This repo contains the code for our ICRA 2021 paper - 
#### [**Can a Robot Trust You?: A DRL-Based Approach to Trust-Driven Human-Guided Navigation**](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9561983).<br>
##### Vishnu Sashank Dorbala, Arjun Srinivasan, Aniket Bera

Please do cite our work if you found this useful:

```
@inproceedings{dorbala2021can,
  title={Can a Robot Trust You?: A DRL-Based Approach to Trust-Driven Human-Guided Navigation},
  author={Dorbala, Vishnu Sashank and Srinivasan, Arjun and Bera, Aniket},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={3538--3545},
  year={2021},
  organization={IEEE}
}
```

## Overview

<img src="https://obj.umiacs.umd.edu/gamma-umd-website-imgs/researchdirections/socrob/robotrust_front_image.jpg">

## Abstract
*Humans are known to construct cognitive maps
of their everyday surroundings using a variety of perceptual
inputs. As such, when a human is asked for directions to a
particular location, their wayfinding capability in converting
this cognitive map into directional instructions is challenged. <br>
Owing to spatial anxiety, the language used in the spoken
instructions can be vague and often unclear. To account for
this unreliability in navigational guidance, we propose a novel
Deep Reinforcement Learning (DRL) based trust-driven robot
navigation algorithm that learns humans’ trustworthiness to
perform a language guided navigation task. <br>
Our approach seeks to answer the question as to whether
a robot can trust a human’s navigational guidance or not. To
this end, we look at training a policy that learns to navigate
towards a goal location using only trustworthy human guidance,
driven by its own robot trust metric.*

## Installation Steps

For the DRL pipeline, refer to instructions in the DRL_exps folder. <br>
For the Gazebo environment, refer to instructions in the gazebo_exps folder. <br>
For the Lang2Symb dataset, refer to the Qualtrics survey conducted in the Lang2Symb folder.