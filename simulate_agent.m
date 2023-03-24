% Input number of runs
num_runs = 3;

% Define agent name from the Worksspace
agent = agent1_Trained; 

% Initiate empty arrays to track fuel consumption
tfc_ego=[];             % total fuel consumption of ego vehicle
tfc_all = [];     % combined fuel consumption of all vehicles

for i=[1:num_runs]
    %% Create the environment for training
    env = OpenAIWrapper(true);
    %% Simulate the environment with the trained agent
    simOptions = rlSimulationOptions('MaxSteps',6);
    experience = sim(env,agent,simOptions);
    
    totalReward = sum(experience.Reward);
    
    % Get total distance
    x=env.getDistance();
    x=x{end};
    distance = x{1};
    
    % Get Fuel Consumption of ego vehicle
    fc = env.getFuelConsumption;
    fuel_consumption = 0;
    for i = [1:length(fc)]
        fuel_consumption=fuel_consumption+fc{i};
    end
    total_fuelConsumption = fuel_consumption/distance
    
    % Plot speeds
    y=env.getSpeeds();
    speeds=[];
    for i =[1:length(y)]
        speeds(end+1)=y{i};
    end
    hold off
    plot(speeds,'LineWidth',3)
    xlabel('Time (s)')
    ylabel('Speeds (m/s)')
    
    % Create a variable to store fuel consumption of each run
    tfc_ego(end+1)=total_fuelConsumption;
    tfc_all(end+1) = env.getSimFuelConsumption();
  
    % Save Fuel Consumption per run to excel file
    writematrix(tfc_ego','fuel_consumption_ego.csv')

     % Save Fuel Consumption for entire corridor per run to excel file
    writematrix(tfc_all','fuel_consumption_all.csv')
end