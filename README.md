# Safe Occlusion Aware Planning
This repository contains the experiment code for our RSS 2021 paper, *Safe Occlusion-aware Autonomous Driving via Game-Theoretic Active Perception*.

## Usage and Limitation

In order to reproduce our results, you need to install modified *[octomap-python](https://github.com/wkentaro/octomap-python)* and *[opendrive2lanelet](https://opendrive2lanelet.readthedocs.io/en/latest/)* packages provided in the ThridParty folder.

In addition, you will need [Carla](https://carla.org/) to run the simulation. We tested our code using Carla 0.9.11 with Python3.6 on Ubuntu 18.04.

You can run ```test _xxx.py``` file to see different examples.

Since we use this code as proof-of-concept purpose for our framework there are several parts needs significant improvement
1. The occluded regions are deteced useing brutal-force method.
2. The safe set are calculated in close-form with simplified dynamics. A more general safe set check step can be implemented by looking up pre-calculated reachable sets through HJI analysis.
3. The A* search in the trajectroy space is slow, and does not provide real-time performance. 

We are currently working on an extension of this work.

## Citation

```
@INPROCEEDINGS{Zhan-RSS-21, 
    AUTHOR    = {Zixu Zhang AND Jaime F Fisac}, 
    TITLE     = {{Safe Occlusion-Aware Autonomous Driving via Game-Theoretic Active Perception}}, 
    BOOKTITLE = {Proceedings of Robotics: Science and Systems}, 
    YEAR      = {2021}, 
    ADDRESS   = {Virtual}, 
    MONTH     = {July}, 
    DOI       = {10.15607/RSS.2021.XVII.066} 
} 
```