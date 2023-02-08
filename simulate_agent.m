
for i=[1:29]
    %% Create the environment for training
    env = OpenAIWrapper();
    %% Simulate the environment with the trained agent
    simOptions = rlSimulationOptions('MaxSteps',6);
    experience = sim(env,agent,simOptions);
    
    totalReward = sum(experience.Reward);
    
    
    % Get total distance
    x=env.getDistance();
    x=x{end};
    distance = x{1};
    
    % Get Fuel Consumption
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
    tfc(end+1)=total_fuelConsumption;
    
    % Save Fuel Consumption per run to excel file
    writematrix(tfc','tfc.csv')
end