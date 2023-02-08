% Call a short script to run my SUMO environment for 1 time step
% pyversion('C:/Users/bking/anaconda3/python')
clc
% Clears classes & reloads Env in the event that the py file is changeed
clear classes
env = py.importlib.import_module('run_traci_from_matlab');
py.importlib.reload(env);

% Run simple function for vehicle to drive to the first intersection
% py.run_traci_from_matlab.run_veh(10)

% Import vehicle environment
VehEnv = py.importlib.import_module('noTraffic_env');
py.importlib.reload(VehEnv);

x=py.run_traci_from_matlab.VehEnv(true);
x.reset();
x.step(10);
x.step(20);

