# Safe Occlusion Aware Planning
This repository contains the experiment code for our RSS 2021 paper, *Safe Occlusion-aware Autonomous Driving via Game-Theoretic Active Perception*.

## Usage and Limitations

In order to reproduce our results, you need to install a modified *[octomap-python](https://github.com/wkentaro/octomap-python)* and *[opendrive2lanelet](https://opendrive2lanelet.readthedocs.io/en/latest/)* packages provided in the ThridParty folder.

In addition, you will need [Carla](https://carla.org/) to run the simulation. We tested our code using Carla 0.9.11 with Python3.6 on Ubuntu 18.04.

You can run ```test _xxx.py``` file to see different examples.

This code is intended as a proof-of-concept demo of our proposed framework, and as such it leaves significant room for improvement on several fronts:
1. The occluded regions are currently deteced using a brute-force method.
2. The safe sets are calculated in closed-form with simplified dynamics. A more general safe set validation step can be implemented by looking up pre-calculated reachable sets obtained through Hamilton-Jacobi-Isaacs analysis.
3. The current A* search in space-time is slow, and does not provide real-time performance. This can be sped up by using alternative search algorithms.

We are currently working on an extension of this work. Stay tuned!

## Citation

If you find our code useful in your own work, please cite our associated paper:

> Z. Zhang and J. F. Fisac, “Safe Occlusion-Aware Autonomous Driving via Game-Theoretic Active Perception,” Robotics: Science and Systems (RSS), 2021.

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
