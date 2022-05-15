function [IneqConst,EqConst] = ConstFunc(InputTraj,InitStateVec,Global,Sys,Drone,Load,MPC)
InputTraj = reshape(InputTraj,Sys.NumInputs,MPC.HorLength);
IneqConst = zeros(Sys.NumDrones,MPC.HorLength);
for TimeStep = 1:MPC.HorLength
 % Solve simulation for current sampling period
 InputVec = InputTraj(:,TimeStep);
 Func = @(Time,StateVec) SystemModel(StateVec,InputVec,Global,Sys,Drone,Load);
 [~,StateVecTraj] = ode45(Func,linspace(0,MPC.SamplingPeriod,10),InitStateVec);
 
 % Define initial state vec for next time-step
 InitStateVec = StateVecTraj(end,:)';
 
 % Extract state values
 for DroneNum = 1:Sys.NumDrones
  Drone(DroneNum).PosVec = InitStateVec(2*(DroneNum - 1) + 1:2*(DroneNum - 1) + 2);
  Drone(DroneNum).VelVec = InitStateVec(Sys.NumStates + 2*(DroneNum - 1) + 1:Sys.NumStates + 2*(DroneNum - 1) + 2);
 end
 Load.PosVec = InitStateVec(2*Sys.NumDrones + 1:2*Sys.NumDrones + 2);
 Load.VelVec = InitStateVec(Sys.NumStates + 2*Sys.NumDrones + 1:Sys.NumStates + 2*Sys.NumDrones + 2);
 
 % Define constraint values
 for DroneNum = 1:Sys.NumDrones
  IneqConst(DroneNum,TimeStep) = norm(Drone(DroneNum).VelVec) - MPC.MaxDroneSpeed;
 end
end
IneqConst = reshape(IneqConst,[],1);
EqConst = Load.VelVec;
