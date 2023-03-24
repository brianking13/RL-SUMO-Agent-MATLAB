# -*- coding: utf-8 -*-
"""
Created on Tue Feb  7 16:28:42 2023

@author: bking
"""

from python_env import VehEnv
import traci

def run_veh(action):
    visualize = True
    env=VehEnv(visualize)
    
    env.reset()
    
    env.step(action)
    print('Run Complete')
    traci.close()
    
