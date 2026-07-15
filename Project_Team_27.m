clc;
clear;
close all;

Pick_time = 2 ;
Place_time = 2;
Pick_target_Pos  = [0.8 0 0.6];
Place_target_Pos = [1.2 0 0.6];

%% IRB_1600 Rigid Body Tree Generation

robot = IRB1600_RBT_Generation();

%% Optimal Configuration Generation 
tol = 0.01; 
N = 100; 
 
% IK and IS solver for Picking
% [Q_final, EE_pose, errors, tau_tot]= IK_IS_Solver(robot, Pick_target_Pos, tol, N); 
Pick_target_Config = ([0.0000 0.1812 0.4499 0.0000 0.5859 0.0000]);

robot.show(Pick_target_Config);

% IK and IS solver for Placing
% [Q_final, EE_pose, errors, tau_tot]= IK_IS_Solver(robot, Place_target_Pos, tol, N);

Place_target_Config = ([0.0000 0.6693 -0.2911 0.0000 0.7475 0.0000]);
% robot.show(Place_target_Config);

%% TD3 Agent Generation

[agent, obsInfo, actInfo] = rlTD3Agent_Generation();
env = rlSimulinkEnv('Project_Team_27_Simulink','Project_Team_27_Simulink/RL Agent', obsInfo, actInfo);

% Backup Reset
load('Training_Backup.mat'); 

%% Training Launch
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 200, ...
    'MaxStepsPerEpisode', 200, ...
    'ScoreAveragingWindowLength', 20, ... 
    'Plots', 'training-progress', ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 750, ...
    'SaveAgentCriteria', 'EpisodeReward', ...
    'SaveAgentValue', 500,...         
     'SaveAgentDirectory', pwd + "/Training_Backup/checkpoints");

trainingStats = train(agent, env, trainOpts);

% Agent and statistics backup
nome_file = sprintf('Backup_%d.mat',1);
save(fullfile('Training_Backup', nome_file), 'agent', 'trainingStats');