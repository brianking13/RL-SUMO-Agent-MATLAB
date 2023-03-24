% https://medium.com/analytics-vidhya/solving-openai-gym-environments-with-matlab-rl-toolbox-fb9d9e06b593


classdef OpenAIWrapper < rl.env.MATLABEnvironment

  properties
    open_env
    IsDone=false
  end

  methods

    function this = OpenAIWrapper(run_traci)
        ObservationInfo = rlNumericSpec([7 1]);
        ObservationInfo.Name = 'VehicleObservation';
%       ObservationInfo.Description = 'Position, Velocity, Angle, VelocityAtTip';
        ActionInfo = rlFiniteSetSpec([0:48]);
        ActionInfo.Name = 'VelocityCommand';
        this = this@rl.env.MATLABEnvironment(ObservationInfo, ActionInfo);
        disp('OpenAIWrapper')
        if nargin < 1
            run_traci = true;
        end
        this.open_env =  py.run_traci_from_matlab.VehEnv(run_traci);
    end
    
    function [Observation,Reward,IsDone,LoggedSignals] = step(this,Action)
        disp('Step')
      result = this.open_env.step(int16(Action)); 
      disp(Action)
      obs = result{1};
      Observation =[obs{1},obs{2},obs{3},double(obs{4}),obs{5},obs{6},obs{7}];
      Reward = result{2};
      IsDone = result{3};
      LoggedSignals = [];
    end

    function InitialObservation = reset(this)
        disp('Reset')
      r = this.open_env.reset();
      result = [r{1},r{2},r{3},double(r{4}),r{5},r{6},r{7}];
      InitialObservation =result;
      disp('end reset')
    end

    function [fc] = getFuelConsumption(this)
        fc = this.open_env.get_fuelConsumption();
    end

    function [speeds]=getSpeeds(this)
        speeds=this.open_env.get_speeds();
    end
    
    function [distance]=getDistance(this)
        distance=this.open_env.get_Distance();
    end

    function [sim_fc]=getSimFuelConsumption(this)
        sim_fc=this.open_env.get_simFuelConsumption();
    end

  end
end
