clear all
close all
clc

rng(7)
%% Setup
network_index = 0;                     % network Index
numOfPlatoons = 1;                     % num of platoons
numOfVehicles = [12]; %[4,5,6,7,8,10];    % num of vehicles in each platoon, for N=8, 10, the lower bound of the con0 is set as 0.001
totalNumOfVehicles = sum(numOfVehicles);

%% Initial setup
vehicleSeperationMean = 10;
vehicleSeperationVar = 7; 
platoonSeperationMean = 10;
platoonSeperationVar = 2;

%% Define the initial vehicular position/velocity/acceleration 
vehiclePosition = 0;                % Zeroth vehicle position
vehicleVelocity = 0;                % Zeroth vehicle velocity
vehicleAcceleration = 0;            % Zeroth vehicle acceleration

state_ki = [vehiclePosition; vehicleVelocity; vehicleAcceleration];
desiredSeparation_ki = 0;              % Desired separation from the leader

%% Define the initial states and desired states for all the vehicles
for k = 1:1:numOfPlatoons           % for every platoon k
    
    parameters_k = [];
    noiseMean_k = [];
    noiseStd_k = [];
    states_k =  [];                       % initial states collection
    desiredSeperation_k = [];

    for i = 1:1:numOfVehicles(k)    % for every vehicle in the k-th platoon

        % Vehicle parameters (assumed homogenuous)
        mass_ki = 1500;
        length_ki = 2.5;
        Af_ki = 2.2;
        rho_ki = 0.78;
        Cd_ki = 0.35;
        Cr_ki = 0.067;
        tau_ki = 0.25;

        % Collection of the initial/initially desired states of each vehicle
        states_k = [states_k, state_ki];
        
        % Define the vehicle position/velocity/acceleration 
        vehiclePosition = vehiclePosition - (vehicleSeperationMean+length_ki) - vehicleSeperationVar*(rand(1)-0.5);
        vehicleVelocity = 0;
        vehicleAcceleration = 0;
        state_ki = [vehiclePosition; vehicleVelocity; vehicleAcceleration];
        

        % Desired vehicular positions 
        desiredSeperation_k = [desiredSeperation_k, desiredSeparation_ki];
        desiredSeparation_ki = desiredSeparation_ki + (vehicleSeperationMean+length_ki);

        % Collection of vehicle parameters
        vehicleParameters_i = [mass_ki; length_ki; Af_ki; rho_ki; Cd_ki; Cr_ki; tau_ki];
        parameters_k = [parameters_k, vehicleParameters_i];

        % Noise parameters
        % noisemean_ki = rand(3,1);
        % noisestd_ki = 0.1*rand(3,1);

        noisemean_ki = zeros(3,1);
        % rand(3,1)-0.5*ones(3,1);
        noisestd_ki = 0.1*rand(3,1);

        noiseMean_k = [noiseMean_k, noisemean_ki];
        noiseStd_k = [noiseStd_k, noisestd_ki];
            
    end
    
    vehiclePosition = states_k(1,end) - platoonSeperationMean - platoonSeperationVar*(rand(1)-0.5);
    
    state_ki = [vehiclePosition; vehicleVelocity; vehicleAcceleration];
    desiredSeparation_ki = 0;

    parameters{k} = parameters_k;
    states{k} = states_k;
    desiredSeparation{k} = desiredSeperation_k;
    noiseMean{k} = noiseMean_k;
    noiseStd{k} = noiseStd_k;
      
end

net0 = Network(network_index,numOfPlatoons,numOfVehicles,parameters,states,desiredSeparation,noiseMean,noiseStd);

disp(['Initial Platoon:'])
fig1 = figure;
ax1 = gca;                   % gca = get current axis handle
ax1.YAxis.Visible = 'off';   % hide out the y-axis from the plot
net0.drawNetwork(fig1)


%% Collecting networks of different size
N = net0.platoons(1).numOfVehicles-1;
for followerCount = 2:N
    disp(['Laoding Platoon with only ',num2str(followerCount+1),' vehicles:'])
    net = net0.copy();
    net.removeVehicles(1,[(followerCount+2):N],0); % errordynamics type = 0
    netArray{followerCount} = net;
end

%% Basic Settings
errorDynamicsType = 2;  % Error dynamics formulation 1 or 2
isDSS = 0;               % Involve DSS constraints (1) or not (0)

gammaSqBar = 10;        % For the constraint: gamma^2 <= gammaSqBar % Desired max noise attenuation
%%% Local passivating controller synthesis specifications 
% LowerBound on \nu and upperBound on \rho 
nuBar = -40;       % For the constraint: nuBar <= nu       
rhoBar = 15;       % For the constraint: rhoHat <= rho <= rhoBar

%%% Parameterized approach to co-design
%%% Optimizing gammaSq with respect to the co-design parameters
% pVals = net1.optimizeCodesignParameters(isCentralized,isDSS)

%%% An alternative: Trying some fixed set of parameters.
pVals = 0.15*ones(numOfPlatoons,max(numOfVehicles));

showPlots = 0;

%% Evaluating time for centralized synthesis for each size network
isOnlyStabilizing = 0;  % Stabilizing controller or Disturbance robust Controller)

numOfIterations = 1000;
platoonSizes = [3:N+1];
cenTimesMat = [];
decenTimesMat = [];
for k = 1:1:numOfIterations
    k
    cenTimes = [];
    decenTimes = [];
    for followerCount = 2:N
        net = netArray{followerCount}.copy();
        
        isCentralized = 1;      % Centralized or decentralized controller synthesis
        [status, gammaSqVal, cenTimeVal] = net.loadPlatoonControllers(errorDynamicsType,isCentralized,isDSS,isOnlyStabilizing,gammaSqBar,nuBar,rhoBar,pVals);
        cenTimes = [cenTimes, cenTimeVal];
    end
    cenTimesMat = [cenTimesMat; cenTimes]

    net = netArray{N}.copy();
    isCentralized = 0;      % Centralized or decentralized controller synthesis
    [status, gammaSqVal, decenTimeVals] = net.loadPlatoonControllers(errorDynamicsType,isCentralized,isDSS,isOnlyStabilizing,gammaSqBar,nuBar,rhoBar,pVals);
    decenTimesMat = [decenTimesMat; decenTimeVals(2:end)]
    
end

save('timingData3')

figure
plot(platoonSizes, mean(cenTimesMat),'.-r')
hold on
plot(platoonSizes, mean(decenTimesMat),'.-b')
xlabel('Platoon Size')
ylabel('Execution Time')
legend('Centralized','Decentralized')
grid on