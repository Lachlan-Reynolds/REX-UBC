function StateDerivVec = SystemModel(StateVec,InputVec,Global,Sys,Drone,Load)

%% Extract states and inputs from vectors
% States
for DroneNum = 1:Sys.NumDrones
 Drone(DroneNum).PosVec = StateVec(2*(DroneNum - 1) + 1:2*(DroneNum - 1) + 2);
 Drone(DroneNum).VelVec = StateVec(2*Sys.NumDrones + 2 + 2*(DroneNum - 1) + 1:2*Sys.NumDrones + 2 + 2*(DroneNum - 1) + 2);
end
Load.PosVec = StateVec(2*Sys.NumDrones + 1:2*Sys.NumDrones + 2);
Load.VelVec = StateVec(2*Sys.NumDrones + 2 + 2*Sys.NumDrones + 1:2*Sys.NumDrones + 2 + 2*Sys.NumDrones + 2);

% Inputs
for DroneNum = 1:Sys.NumDrones
 Drone(DroneNum).PropForce = InputVec(2*(DroneNum - 1) + 1:2*(DroneNum - 1) + 2);
end

%% Define drone forces
for DroneNum = 1:Sys.NumDrones
 % Gravitational force
 Drone(DroneNum).GravForce = [0 -Drone(DroneNum).Mass*Global.GravAccel]';
 
 % Drag force
 Drone(DroneNum).DragForce =...
  -0.5*Drone(DroneNum).DragCoeff*Global.AirDensity*Drone(DroneNum).Area*...
  norm(Drone(DroneNum).VelVec)*Drone(DroneNum).VelVec;
 
 % Cable tension
 Drone(DroneNum).Cable.Tension =...
  (Drone(DroneNum).Cable.StiffCoeff*(norm(Load.PosVec - Drone(DroneNum).PosVec) - Drone(DroneNum).Cable.Length)...
  + Drone(DroneNum).Cable.DampCoeff*norm(Load.VelVec - Drone(DroneNum).VelVec))*...
  (Load.PosVec - Drone(DroneNum).PosVec)/norm(Load.PosVec - Drone(DroneNum).PosVec);
end

%% Define load forces
% Gravitational force
Load.GravForce = [0 -Load.Mass*Global.GravAccel]';

% Drag force
Load.DragForce =...
 -0.5*Load.DragCoeff*Global.AirDensity*Load.Area*...
 norm(Load.VelVec)*Load.VelVec;

% Total cable tension
Load.CableForce = zeros(2,1);
for DroneNum = 1:Sys.NumDrones
 Load.CableForce = Load.CableForce - Drone(DroneNum).Cable.Tension;
end

%% Build state derivative vector
StateDerivVec = zeros(2*(2*Sys.NumDrones + 1),1);
for DroneNum = 1:Sys.NumDrones
 StateDerivVec(2*(DroneNum - 1) + 1:2*(DroneNum - 1) + 2) = Drone(DroneNum).VelVec;
 StateDerivVec(2*Sys.NumDrones + 2 + 2*(DroneNum - 1) + 1:2*Sys.NumDrones + 2 + 2*(DroneNum - 1) + 2) = (1/Drone(DroneNum).Mass)*(Drone(DroneNum).PropForce + Drone(DroneNum).GravForce + Drone(DroneNum).DragForce + Drone(DroneNum).Cable.Tension);
end
StateDerivVec(2*Sys.NumDrones + 1:2*Sys.NumDrones + 2) =  Load.VelVec;
StateDerivVec(2*Sys.NumDrones + 2 + 2*Sys.NumDrones + 1:2*Sys.NumDrones + 2 + 2*Sys.NumDrones + 2) =  (1/Load.Mass)*(Load.GravForce + Load.DragForce + Load.CableForce);
