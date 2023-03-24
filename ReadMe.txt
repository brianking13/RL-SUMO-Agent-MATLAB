This python program is used to train an RL agent to drive through a set of connected signalized corridors using Eclipse SUMO framework.
The RL agent is trained using the RL Toolbox in MATLAB using a custom MATLAB wrapper for the python gym environment.


Dependencies:
numpy==1.19.5
keras==2.10.0
keras_rl2==1.0.5
tensorflow==2.5.0
gym==0.17.3

pip install SUMO
pip install Traci

MATLAB Version: R2021b
Python Version: 3.9.12

To run ENV:

1) Run "initiate_env.m". If this file is not run, changes to the python code will not be implemented within the MATLAB wrapper.
2) Open the "Reinforcement Learning Designer" app in MATLAB
    a) import the environment ("env")
    b) Click "New" (the 3rd icon from the right)
    c) Select a name for your agent, the type (PPO or DQN), and the number of nodes
    d) Train the agent
    e) Click "Accept"
    f) Click "Export" and select the trianed agent
3) Run "simulate_agent.m". Make sure you change the agent name and the number of runs at the beginning of the scipt.
4) Excel sheets will be created with fuel economy after each run (units in mL).