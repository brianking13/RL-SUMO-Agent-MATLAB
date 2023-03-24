%% Brian King 
% Call a short script to run SUMO environment for 1 time step
% User should see a success statement in the cmd window
% The SUMO env window should pop up as well

% If Python issue, use this command with the path to your python version:
% pyversion('C:/Users/bking/anaconda3/python')
clc

% Clears classes & reloads Env in the event that the py file is changeed
clear classes

% Import vehicle environment
VehEnv = py.importlib.import_module('python_env');
py.importlib.reload(VehEnv);

x=py.run_traci_from_matlab.VehEnv(true);
x.reset();
x.step(10);

% Create a new environment in MATLAB 
clear
env = OpenAIWrapper(false);
disp('Test run successful. Proceed with RL implementation in the RL Designer App.')




