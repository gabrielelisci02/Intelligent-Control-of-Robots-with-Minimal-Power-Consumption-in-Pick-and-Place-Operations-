function [agent, obsInfo, actInfo] = rlTD3Agent_Generation()
%% Observation and Action space
numObs = 32;
numAct = 6;

% Observation
obsInfo = rlNumericSpec([numObs 1]);
obsInfo.Name = 'observations';

% Action
actInfo = rlNumericSpec([numAct 1], 'LowerLimit', -100, 'UpperLimit', 100);
actInfo.Name = 'torque';

%% Critic Architecture
obsPath = [
    featureInputLayer(numObs, 'Name', 'observations')
    fullyConnectedLayer(256, 'Name', 'obsFC')
    reluLayer('Name', 'obsRelu')];

actPath = [
    featureInputLayer(numAct, 'Name', 'torque')
    fullyConnectedLayer(256, 'Name', 'actFC')];

%% Common Path
commonPath = [
    concatenationLayer(1, 2, 'Name', 'concat')
    fullyConnectedLayer(512, 'Name', 'commonFC1')
    reluLayer('Name', 'commonRelu1')
    fullyConnectedLayer(256, 'Name', 'commonFC2')
    reluLayer('Name', 'commonRelu2')
    fullyConnectedLayer(1, 'Name', 'Value')];


%% Network Graph Assembly
criticNetwork = layerGraph(obsPath);
criticNetwork = addLayers(criticNetwork, actPath);
criticNetwork = addLayers(criticNetwork, commonPath);

% Branches Connection
criticNetwork = connectLayers(criticNetwork, 'obsRelu', 'concat/in1');
criticNetwork = connectLayers(criticNetwork, 'actFC', 'concat/in2');

% Critics Creation
critic1 = rlQValueFunction(criticNetwork, obsInfo, actInfo, ...
    'ObservationInputNames', 'observations', 'ActionInputNames',...
    'torque');

critic2 = rlQValueFunction(criticNetwork, obsInfo, actInfo, ...
    'ObservationInputNames', 'observations', 'ActionInputNames',...
    'torque');

%% Actor Architecture
actorNetwork = [
    featureInputLayer(numObs, 'Name', 'state')
    fullyConnectedLayer(512)
    reluLayer
    fullyConnectedLayer(512)
    reluLayer
    fullyConnectedLayer(numAct)
    tanhLayer
    scalingLayer('Scale', 100)];

actorNetwork = dlnetwork(actorNetwork);

actor = rlContinuousDeterministicActor(actorNetwork, obsInfo, actInfo);

%% TD3 Agent Creation
agentOpts = rlTD3AgentOptions;

agentOpts.SampleTime = 0.01;
agentOpts.DiscountFactor = 0.99;
agentOpts.LearningFrequency = -1;

agentOpts.TargetSmoothFactor = 3e-3;

agentOpts.TargetPolicySmoothModel.StandardDeviation = 0.2;
agentOpts.TargetPolicySmoothModel.StandardDeviationDecayRate = 1e-4;
agentOpts.TargetPolicySmoothModel.LowerLimit = -0.5;
agentOpts.TargetPolicySmoothModel.UpperLimit = 0.5;


% Critics Options
criticOpts = rlOptimizerOptions( ...
    'LearnRate', 1e-4, ...
    'GradientThreshold', 1);

% Actor Options
actorOpts  = rlOptimizerOptions(...
    'LearnRate', 5e-5, ...
    'GradientThreshold', 1);

agentOpts.ActorOptimizerOptions = actorOpts;
agentOpts.CriticOptimizerOptions = criticOpts;

% Exploration Model
agentOpts.ExplorationModel.StandardDeviation = 0.8;
agentOpts.ExplorationModel.StandardDeviationDecayRate = 2e-4;
agentOpts.ExplorationModel.StandardDeviationMin = 0.005;  

agentOpts.ExperienceBufferLength = 1e6;
agentOpts.NumStepsToLookAhead = 1;
agentOpts.MiniBatchSize = 256;
 
agentOpts.InfoToSave.Optimizer = true;
agentOpts.InfoToSave.ExperienceBuffer = true;
agentOpts.InfoToSave.Target = true;

% Agent Creation
agent = rlTD3Agent(actor, [critic1, critic2], agentOpts);
end