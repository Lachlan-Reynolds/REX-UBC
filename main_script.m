%% Setup workspace
clear;
clc;
close all;

%% Define system properties
% Simulation
Sim.NumTimeSteps = 30;
Sim.SavesPerTimeStep = 100;

Options = optimoptions('fmincon','UseParallel',true);

% Global
Global.GravAccel = 9.81;
Global.AirDensity = 1.2;

% System
Sys.NumDrones = 1;
Sys.NumStates = 2*Sys.NumDrones + 2;
Sys.NumInputs = 2*Sys.NumDrones;

% Drone
for DroneNum = Sys.NumDrones:-1:1

 Drone(DroneNum).Mass = 1;
 
 Drone(DroneNum).Area = 1;
 
 Drone(DroneNum).DragCoeff = 20;
 
 Drone(DroneNum).Cable.Length = 1;
 
 Drone(DroneNum).Cable.StiffCoeff = 1000;
 
 Drone(DroneNum).Cable.DampCoeff = 0.1*2*sqrt(Drone(DroneNum).Cable.StiffCoeff*Drone(DroneNum).Mass);
end

% Load

Load.Mass = 3;

Load.Area = 2;

Load.DragCoeff = 20;

% MPC
MPC.HorLength = 5;

MPC.SamplingPeriod = 1;

MPC.TargetPosVec = [10 0]';

MPC.MaxDroneSpeed = 1;

%% Initialize arrays
for DroneNum = Sys.NumDrones:-1:1
 Drone(DroneNum).PosVec = zeros(2,1);
 Drone(DroneNum).VelVec = zeros(2,1);
 Drone(DroneNum).PropForce = zeros(2,1);
 Drone(DroneNum).GravForce = zeros(2,1);
 Drone(DroneNum).DragForce = zeros(2,1);
 Drone(DroneNum).Cable.Tension = zeros(2,1);
end

for DroneNum = Sys.NumDrones:-1:1
 Drone(DroneNum).TotalInputVecTraj = zeros(Sim.NumTimeSteps,2);
 Drone(DroneNum).TotalPosVecTraj = zeros(Sim.SavesPerTimeStep + (Sim.SavesPerTimeStep - 1)*(Sim.NumTimeSteps - 1),2);
 Drone(DroneNum).TotalVelVecTraj = zeros(Sim.SavesPerTimeStep + (Sim.SavesPerTimeStep - 1)*(Sim.NumTimeSteps - 1),2);
end
Load.TotalPosVecTraj = zeros(Sim.SavesPerTimeStep + (Sim.SavesPerTimeStep - 1)*(Sim.NumTimeSteps - 1),2);
Load.TotalVelVecTraj = zeros(Sim.SavesPerTimeStep + (Sim.SavesPerTimeStep - 1)*(Sim.NumTimeSteps - 1),2);
TotalTime = linspace(0,Sim.NumTimeSteps*MPC.SamplingPeriod,Sim.SavesPerTimeStep + (Sim.SavesPerTimeStep - 1)*(Sim.NumTimeSteps - 1))';

%% Define initial conditions
InitStateVec = [0 0 0 (Drone(1).Cable.Length + Load.Mass*Global.GravAccel/Drone(1).Cable.StiffCoeff) 0 0 0 0]';
InitInputTraj = [
 1;
 (Drone(1).Mass + Load.Mass/Sys.NumDrones)*Global.GravAccel];
InitInputTraj = InitInputTraj.*ones(1,MPC.HorLength);
InitInputTraj = reshape(InitInputTraj,[],1);

%%
TotalTimeStepNum = 1;
for TimeStep = 1:Sim.NumTimeSteps
 % Solve MPC problem to obtain optimal input trajectory
 %CostFunc
 Func1 = @(InputTraj) CostFunc(InputTraj,InitStateVec,Global,Sys,Drone,Load,MPC);
 %ConstFunc
 Func2 = @(InputTraj) ConstFunc(InputTraj,InitStateVec,Global,Sys,Drone,Load,MPC);
 
  
 %fmincon
 OptimInputTraj = fmincon(Func1,InitInputTraj,[],[],[],[],[],[],Func2,Options);
 OptimInputTraj = reshape(OptimInputTraj,Sys.NumInputs,MPC.HorLength);
 
 % Implement first prediction time-step input vector
 InputVec = OptimInputTraj(:,1);
 Func = @(Time,StateVec) SystemModel(StateVec,InputVec,Global,Sys,Drone,Load);
 [Time,StateVecTraj] = ode45(Func,linspace(0,MPC.SamplingPeriod,Sim.SavesPerTimeStep),InitStateVec);
 
 % Update initial state vector for next time-step
 InitStateVec = StateVecTraj(end,:)';
 InitInputTraj = [OptimInputTraj(:,2:end), OptimInputTraj(:,end)];
 InitInputTraj = reshape(InitInputTraj,[],1);
 
 % Extract state trajectories
 for DroneNum = Sys.NumDrones:-1:1
  Drone(DroneNum).PosVecTraj = StateVecTraj(:,2*(DroneNum - 1) + 1:2*(DroneNum - 1) + 2);
  Drone(DroneNum).VelVecTraj = StateVecTraj(:,Sys.NumStates + 2*(DroneNum - 1) + 1:Sys.NumStates + 2*(DroneNum - 1) + 2);
 end
 Load.PosVecTraj = StateVecTraj(:,2*Sys.NumDrones + 1:2*Sys.NumDrones + 2);
 Load.VelVecTraj = StateVecTraj(:,Sys.NumStates + 2*Sys.NumDrones + 1:Sys.NumStates + 2*Sys.NumDrones + 2);
 
 % Save data to total trajectory arrays
 for DroneNum = Sys.NumDrones:-1:1
  Drone(DroneNum).TotalInputVecTraj(TimeStep,:) = InputVec';
  Drone(DroneNum).TotalPosVecTraj(TotalTimeStepNum:TotalTimeStepNum + Sim.SavesPerTimeStep - 1,:) = Drone(DroneNum).PosVecTraj;
  Drone(DroneNum).TotalVelVecTraj(TotalTimeStepNum:TotalTimeStepNum + Sim.SavesPerTimeStep - 1,:) = Drone(DroneNum).VelVecTraj;
 end
 Load.TotalPosVecTraj(TotalTimeStepNum:TotalTimeStepNum + Sim.SavesPerTimeStep - 1,:) = Load.PosVecTraj;
 Load.TotalVelVecTraj(TotalTimeStepNum:TotalTimeStepNum + Sim.SavesPerTimeStep - 1,:) = Load.VelVecTraj;
 TotalTimeStepNum = TotalTimeStepNum + Sim.SavesPerTimeStep - 1;
 
 % Plot animation over current time-step
 TmpX = StateVecTraj(:,1:2:2*Sys.NumDrones + 2);
 TmpY = StateVecTraj(:,2:2:2*Sys.NumDrones + 2);
 for InTimeStep = 1:length(Time)
  for DroneNum = 1:Sys.NumDrones
   plot(...
    Drone(DroneNum).PosVecTraj(InTimeStep,1),Drone(DroneNum).PosVecTraj(InTimeStep,2),...
    'ko','LineWidth',3.0);
   hold on;
   plot(...
    [Drone(DroneNum).PosVecTraj(InTimeStep,1) Load.PosVecTraj(InTimeStep,1)]',...
    [Drone(DroneNum).PosVecTraj(InTimeStep,2) Load.PosVecTraj(InTimeStep,2)]',...
    'k','LineWidth',2.0);
  end
  plot(...
   Load.PosVecTraj(InTimeStep,1),Load.PosVecTraj(InTimeStep,2),...
   'ko','MarkerSize',20.0,'LineWidth',3.0);
  hold off;
  axis([...
   min(min(TmpX))...
   max(max(TmpX))...
   min(min(TmpY))...
   max(max(TmpY))]);
  pause(0.05);
 end
end

%% Save data
save('Data.Mat');

%% Plot animation
% Create a temporary vector to determine axes limits
TmpX = zeros(length(TotalTime),Sys.NumDrones + 1);
TmpY = zeros(length(TotalTime),Sys.NumDrones + 1);
for DroneNum = 1:Sys.NumDrones
 TmpX(:,DroneNum) = Drone(DroneNum).TotalPosVecTraj(:,1);
 TmpY(:,DroneNum) = Drone(DroneNum).TotalPosVecTraj(:,2);
end
TmpX(:,Sys.NumDrones + 1) = Load.TotalPosVecTraj(:,1);
TmpY(:,Sys.NumDrones + 1) = Load.TotalPosVecTraj(:,2);

% Create plots
Video = VideoWriter('Single drone simulation.avi');
Video.Quality = 100;
Video.FrameRate = 24;
open(Video);
for TimeStep = 1:10:length(TotalTime)
 for DroneNum = 1:Sys.NumDrones
  plot(...
  Drone(DroneNum).TotalPosVecTraj(TimeStep,1),Drone(DroneNum).TotalPosVecTraj(TimeStep,2),...
  'ko','LineWidth',3.0);
 hold on;
 plot(...
  [Drone(DroneNum).TotalPosVecTraj(TimeStep,1) Load.TotalPosVecTraj(TimeStep,1)]',...
  [Drone(DroneNum).TotalPosVecTraj(TimeStep,2) Load.TotalPosVecTraj(TimeStep,2)]',...
  'k','LineWidth',2.0);
 end
 plot(...
  Load.TotalPosVecTraj(TimeStep,1),Load.TotalPosVecTraj(TimeStep,2),...
  'ko','MarkerSize',20.0,'LineWidth',3.0);
 plot(MPC.TargetPosVec(1),MPC.TargetPosVec(2),'r+','MarkerSize',10.0,'LineWidth',3.0);
 hold off;
 axis([...
  (min(min(TmpX)) - 2)...
  (max(max(TmpX)) + 2)...
  (min(min(TmpY)) - 2)...
  (max(max(TmpY)) + 2)]);
 writeVideo(Video,getframe(gcf));
 pause(0.05)
end
close(Video);
