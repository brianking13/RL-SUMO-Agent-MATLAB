# -*- coding: utf-8 -*-
"""
Created on Mon Nov 14 21:35:39 2022

@author: bking

This code is a Python script for a simulation environment of a traffic
intersection. It uses the SUMO traffic simulation software, as well as a 
reinforcement learning (RL) library called Gym. The environment is defined as 
a class called VehEnv, which has a constructor that initializes the parameters 
of the environment and an __init__ method that sets the action and observation 
spaces. The step method is used to take a single step in the simulation and 
update the state of the environment based on the action taken by the agent. 
The InterStep method is a helper function used to gather data from the 
simulation about the state of the intersection.

"""


# imports for SUMO and general script
import traci
from sumolib import checkBinary
import time
import random
import xml.etree.ElementTree as ET
import os
import numpy as np

# imports for RL
from gym import Env
from gym.spaces import Discrete, Box


class VehEnv(Env):
    
    def __init__(self,visualizer):
            
            #actions
            self.action_space = Discrete(48)
            
            #observations
            self.observation_space = Box(low=np.array([0,0,0,0,0,0,0]),high=np.array([1,1,1,1,1,1,1]))
            
            # intialize parameters
            self.fuel_consumption = []
            # print("Visualize set to", visualizer)
            if visualizer == True:
                
                self.visualizer= True
                self.speeds=[]
                self.accels=[]
                self.total_reward = []
                self.fuel_consumption = [] 
                self.distance=[]
            else:
                self.visualizer = False
                
                
     
    def step(self,action):
        # decrease sim length by one for each step
        self.sim_length -= 1
        # get a list of all vehicles
        vehicle_list = traci.vehicle.getIDList()
        
        # if vehicle exists, set vehicle speed based on action and retrieve data
        if 'v_0' in vehicle_list:
           traci.vehicle.setSpeed('v_0',action+1)
           
           # for each intersection, reset the reward to 0
           reward = 0
           
           # for 5000 seconds, run sim. Break if an intersection is passed
           time = 0
           for i in range(1,5000):
               traci.simulationStep()
               
               # Gather fuel consumption data for all vehicles
               self.simFuelConsumptions()
               
               self.state, sec_reward, done, info,intersection = self.InterStep(action)
               time +=1
               if done == True:
                   break
              
               # accumulate the reward over time
               reward = reward + sec_reward
               if intersection == True:
                   break
        
        # find current and upcoming lane id's
        current_lane = traci.vehicle.getLaneID('v_0')
        
        # find number of vehicles moving along upcoming stretch of corridor
        number_vehicles_upcoming = traci.lane.getLastStepVehicleNumber(current_lane)-1
        # find mean speed of vehicles moving along upcoming stretch of corridor
        mean_lane_speed = traci.lane.getLastStepMeanSpeed(current_lane)
        # print('Upcoming Lane Car Number',number_vehicles_upcoming)
        # print('Lane Speed:', mean_lane_speed)
        
        self.state[len(self.state)-2] = number_vehicles_upcoming/20  
        self.state[len(self.state)-1] = mean_lane_speed/20
        
        
        return self.state, reward/time, done, info
             
    
    def InterStep(self,action):
        
        
        # get list of all vehicles
        vehicle_list = traci.vehicle.getIDList()
        
        # initialize variables
        reward = 0
        intersection_distances = []
        phase_durations =[]
        phases= []

        
        if 'v_0' in vehicle_list:
            
            v = traci.vehicle.getSpeed('v_0')
            
            # get data from each stop light
            pos = traci.vehicle.getPosition('v_0')
            light_names = traci.trafficlight.getIDList()
            for i in light_names: 
                # print(str(i))
                
                # get distance to each intersection
                intersect = traci.junction.getPosition(i)
                distance_from_light = intersect[0]-pos[0]
                intersection_distances.append(distance_from_light)
                
                # print('Distance to Intersection: ', str(distance_from_light))
                # # get current light state
                # light_states = traci.trafficlight.getPhase(i)
                # print('Light states: ', str(light_states))
                
                # # get times until next switch
                # time_to_switch = traci.trafficlight.getNextSwitch(i) - traci.simulation.getTime()
                # print('Time to switch: ', str(time_to_switch))
                
                # # get phases durations
                # phase_duration = traci.trafficlight.getPhaseDuration(i)
                # print('Phase Duration: ',str(phase_duration))
                # phase_durations.append(phase_duration)
                
                # # get current phases
                # phase = traci.trafficlight.getRedYellowGreenState(i)
                # current_phases = phase[10]
                # phases.append(phase[10])
                # print('Current Phase: ',str(current_phases),"\n")
        else:
            v = 0 
        
        # reward function
        fc=traci.vehicle.getFuelConsumption('v_0') 
        reward = self.reward_function(v,fc)
        accel = traci.vehicle.getAcceleration('v_0')
        
        if self.visualizer== True:
            time.sleep(.01)
            self.data_collection(v, accel, fc, reward)
            # print('fc: ',str(fc))
            # print('speed',str(v))
            # print('accel', str(accel))
            # print('reward', str(reward))
            # print('')

            
            
        # create state observations
        
        # find all positive gaps
        pos_intersection_distances = [i for i in intersection_distances if i > 0]
        
  
        # find distance to closest intersection
        gap = min(pos_intersection_distances)
        
        # NEXT PHASE TIME and NEXT PHASE
        index = intersection_distances.index(gap) 
        current_light = light_names[index]

        # get time until next phase
        phase_time = traci.trafficlight.getNextSwitch(current_light) - traci.simulation.getTime()
        
        # get next phase color
        current_phase = traci.trafficlight.getRedYellowGreenState(current_light)
        if current_phase == 'RR' or current_phase == 'rr':
            phase = 0
        else:
            phase = 1
        
        previous_gap = self.state[1]*1000
        
        intersection = False
        if gap > previous_gap:
            # print('reached intersection')
            intersection = True
        else:
            intersection = False
        
        # return the length of a green light
        phase_duration = traci.trafficlight.getPhaseDuration(current_light)
        if phase ==1:
            green_light_time = phase_duration/60
        else:
            green_light_time = 1-phase_duration/60

        self.state=[v/self.max_speed,gap/1000,phase_time/100,phase,green_light_time,0,0]
      
        # if self.sim_length <= 0 or vehicle_list == [] or traci.simulation.getTime() > 400:
        # when testing, set the sim to terminate at 8000 meters instead of at a red light
        # if gap < 15 and pos[0] > 8000:
        #     print('MADE IT FAR!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        #     # self.reset()
        # if v < 5 and accel>0 :
        if  traci.simulation.getTime()>500:
            done = False
            # print("BOOTSTRAP HERE!!!!")
        else:
            done = False
        
            
            
        info = {}
        
        return self.state, reward, done, info, intersection

    
    
    
    def reward_function(self,v,fc):
        # reward =  (5000/(fc/(v+.01)-.01)-1)/50-0.01
        # reward =  1/(2.7**(0.05*(fc/500)**2))
        # reward = 1/(2.7**(0.005*(20-v)**2))-0.5 # put this in graphing calculator to see speed vs reward
        # reward = 0
        # if v < 30 and v > 10:
            # reward += 1
        # else:
            # reward -= .5
        # reward = 1/(.01+fc/(v+.01))
        reward = -fc/5000+1
        if v < 10 or v > 30:
            reward -= 1
        if v<.1:
            reward -= 1
        # print('Reward: ',reward)
        
        return reward
        
    
    def get_fuelConsumption(self):
        return self.fuel_consumption
    
    def get_speeds(self):
        return self.speeds
    
    # Returns fuel consumption for all vehicles in the simulation
    def get_simFuelConsumption(self):
        if self.sim_total_distance>0:
            return self.sim_fuel_consumption/self.sim_total_distance
        else:
            return self.sim_fuel_consumption

    # Adds fuel consumption at time step to total fuel consumption for all vehicles behind the ego vehicle
    def simFuelConsumptions(self):
        x = traci.vehicle.getIDList()
        z = [y for y in x if y[:4]=='f_19' or y[:3]=='v_0']

        # add up the fuel consumption per meter for all vehicles and divide it by the number of vehicles in the sim
        fc = 0
        distance= 0
        for vehicle in z:
            # distance = speed since time step is 1 second ([m/s]*[s]=[m])
            distance += traci.vehicle.getSpeed(vehicle)
            fc  += traci.vehicle.getFuelConsumption(vehicle)
        
        # Variables for fuel consumption of ego car
        self.test_fc+=traci.vehicle.getFuelConsumption('v_0')
        self.test_distance +=traci.vehicle.getSpeed('v_0')
        
        self.sim_fuel_consumption += fc/len(z)
        self.sim_total_distance += distance/len(z)
        
    # Returns fuel consumption of ego car
    def testFC(self):
        return self.test_fc/self.test_distance      
    
    def get_totalReward(self):
        return self.total_reward
    
    def get_accelerations(self):
        return self.accels
    
    def get_Distance(self):
        return self.distance
    
    def data_collection(self,v,accel,fc,reward):
        self.speeds.append(v)
        self.accels.append(accel)
        self.fuel_consumption.append(fc)
        self.total_reward.append(reward)
        self.distance.append(traci.vehicle.getPosition('v_0'))
    
    def step_constantSpeed(self,set_speed):
        
        vehicle_list = traci.vehicle.getIDList()
        if 'v_0' in vehicle_list:
            traci.vehicle.setSpeed('v_0',set_speed)
            
        traci.simulationStep()
        
        # Gather fuel consumption data for all vehicles
        self.simFuelConsumptions()
        
        fc=traci.vehicle.getFuelConsumption('v_0')
        accel = traci.vehicle.getAcceleration('v_0')
        v = traci.vehicle.getSpeed('v_0')
        reward = self.reward_function(v, fc)
        
        if self.visualizer== True:
            time.sleep(.01)
            self.data_collection(v, accel, fc, reward)
            # print('fc: ',str(fc))
            # print('speed',str(v))
            # print('accel', str(accel))
            # print('reward', str(reward))
            # print('')
    
    def close(self):
        traci.close()
        
    def render(self):
        pass
    
    def reset(self):

        # close running session
        # running simution in SUMO
        try:
            traci.close()
        except:
            pass
        
        if self.visualizer == False:
            sumoBinary = checkBinary('sumo') 
        else:
            sumoBinary = checkBinary('sumo-gui') 
            
        # edit intersection positions after each episode
        tree = ET.parse('connectedCorridor.nod.xml')
        root = tree.getroot()
        for nodes in root.iter('nodes'):
            for intersection_num in range(1,17):
                for node in nodes.iter('node'):
                    if node.attrib['id'] == 'J' + str(intersection_num):
                        # print(intersection_num)
                        # print(node.attrib['x']) 
                        intersect = random.randint(800+1000*(intersection_num-1),1200+1000*(intersection_num-1))
                        # print(intersect)
                        node.attrib['x'] = str(intersect)
                        # print(node.attrib['x'])
        tree.write('connectedCorridor.nod.xml')  

        # edit network timing after each episode
        tree = ET.parse('connectedCorridor.tll.xml')
        root = tree.getroot()
        for nodes in root.iter('tlLogics'):
            for logic in root.iter('tlLogic'):
                logic.attrib['offset'] = str(random.randint(1,60))
                # make all lights have 60 seconds cycles
                rand_time =random.randint(10,50)
                # rand_time =30
                green = rand_time
                red = 60 - rand_time
                for phase in logic.iter('phase'):   
                    if phase.attrib['state'] == "GG":
                        phase.attrib['duration'] = str(green)
                    elif phase.attrib['state'] == "rr":
                        phase.attrib['duration'] = str(red)
        tree.write('connectedCorridor.tll.xml')   
                   
        # edit traffic demand each run
        tree = ET.parse('connectedCorridor.rou.xml')
        root = tree.getroot()
        for nodes in root.iter('routes'):
            for flow in root.iter('flow'):
                # only applying random 0,1 to vehicles in front of ego, not behind (f_19)
                if flow.attrib['id']!='f_19':
                    number_cars =str(random.randint(0,1))
                    # number_cars = "1"
                    flow.attrib['number'] = number_cars
        tree.write('connectedCorridor.rou.xml')
        
        
        os.system('netconvert --node-files=connectedCorridor.nod.xml --edge-files=connectedCorridor.edg.xml \
                  --connection-files=connectedCorridor.con.xml  \
                      --tllogic-files=connectedCorridor.tll.xml  \
          --output-file=connectedCorridor.net.xml')
          
        traci.start([sumoBinary, "-n", "connectedCorridor.net.xml", "-r", "connectedCorridor.rou.xml", "--start", "--quit-on-end"])
        
        
        traci.simulationStep()
        traci.simulationStep()

        
        if self.visualizer:
            # make GUI objects visible
            traci.gui.setSchema('View #0', 'big_vehicle')
            self.speeds=[]
            self.accels=[]
            self.total_reward = []
            self.fuel_consumption = []   
            self.distance=[]
 
        self.sim_length = 12
        self.max_speed = 40
        traci.vehicle.setMaxSpeed('v_0',self.max_speed)
        v = traci.vehicle.getSpeed('v_0')
        self.sim_fuel_consumption = 0
        self.sim_total_distance = 0
        
        
        self.test_fc=0
        self.test_distance=0
        
        self.state=[v/self.max_speed,0,0,0,0,0,0]
        
        self.step(20)

        return self.state
    
