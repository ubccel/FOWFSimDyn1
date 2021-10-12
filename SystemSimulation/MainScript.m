%% Setup workspace
clear;
clc;
close all;
format short g;
set(0,'defaulttextinterpreter','latex');
set(0,'DefaultTextFontname','Times New Roman');
set(0,'DefaultAxesFontName','Times New Roman');
set(0,'DefaultTextFontSize',12);
set(0,'DefaultAxesFontSize',12);


%% Simulation settings
% Simulation case
Global.TurbDia = 126;

Global.MeanWindSpeed = 12;  %m/s
Farm.SpacingX = 7*Global.TurbDia;    %downstream spacing, m
Farm.SpacingY = 4*Global.TurbDia;


% Mesh settings
Solver.DnsElmLengthA = 1*Global.TurbDia;
Solver.NumMeshRedElm = 4;
Solver.CentralDiff = 1;

% Model dynamics?
Solver.TurbMotion = 1;

% Tuned parameters
Global.WakeExpConst = 0.08; 
Global.RefWindSpeed = Global.MeanWindSpeed;
Global.WakeExpRate = Global.WakeExpConst*Global.RefWindSpeed;
Global.GaussStd_Slope = 0.0250176061678923;
Global.GaussStd_Inter = 0.396350723166689;

% Actuator disk model
Solver.ActDiscModel = 'Vortex';
Solver.RotorMomModel = 'Jimenez';

% Fundamental sample time and impulse time setting
SampleTime = 1;        %1
timeEnd = 1500;         %2000 sec


%%model selction
modelselect = 1;
%if modelselect equal to 1, the code would run as a 3 input model (yaw angle, blade pitch angle and generator torque)
% if the modelsect is not equal to 1, the code would run as a 3 input model (wind speed, yaw angle and AI factor)


%% Wind farm layout
% Wind farm layout with wind direction of 0 deg
Farm.NumTurbX = 3;
Farm.NumTurbY = 1;

% Total number of wind turbines
Farm.NumTurb = Farm.NumTurbX*Farm.NumTurbY;

%% Simulation inputs to vary
% Free stream wind conditions
InitFreeStreamWindVel = [Global.MeanWindSpeed 0]';
InitFreeStreamWindAccel = [0 0]';

%freeStreamImpulse = 0.45;
freeStreamImpulse = 1;
StepFreeStreamWindVel = [(Global.MeanWindSpeed + freeStreamImpulse) 0]';
StepFreeStreamWindAccel = [0 0]';

% Turbine inputs
InitAIFactorVec = 1/3*ones(Farm.NumTurb,1);
InitYawAngleVec = [-15;15;0]*pi/180;   % original 0, data in vector is in degrees,and the *pi/180 is used to convert to radians
InitRotorSpeedVec=1*ones(Farm.NumTurb,1);   % Initial rotor speed rad/s
InitGeneTorqueVec= 30000.*ones(Farm.NumTurb,1) * 97; %Initial generator torque Nm (*97 is to convert to the rotor side)
InitBladePitch=0*ones(Farm.NumTurb,1);      % Initial blade pitch angle degree

%warning
for i = 1:length(InitYawAngleVec)
    if (InitYawAngleVec(i)> 25*pi/180 || InitYawAngleVec(i)<-25*pi/180)
        disp('warning: values in initial yaw angle vector may need be converted to radians');
    end
end



%% External properties
% Gravitational acceleration
Global.GravAccel = 9.81;

% Fluid properties
Global.AirDensity = 1.225;
Global.WaterDensity = 1028;

%% Floating wind turbine properties
for TurbNum = Farm.NumTurb:-1:1
 % Mechanical properties
 Turb(TurbNum).Mass = 1.4e7;
 Turb(TurbNum).MomentInertia=4.3703e7;  
 
 % Turbine dimensions
 Turb(TurbNum).RotorDia = Global.TurbDia;
 Turb(TurbNum).RotorArea = pi/4*Turb(TurbNum).RotorDia^2;
 
 % Platform dimensions
 Turb(TurbNum).FLDistRelG  = 40.87;
 Turb(TurbNum).MidCylDia = 6.5;  %center cylinder
 Turb(TurbNum).TopCylDia = 12;
 Turb(TurbNum).BottCylDia = 24;
 Turb(TurbNum).MidCylLength = 20;
 Turb(TurbNum).TopCylLength = 14;
 Turb(TurbNum).BottCylLength = 6;
 
 % Platform hydrodynamic properties
 Turb(TurbNum).MidCylDragCoeff = 0.56;
 Turb(TurbNum).TopCylDragCoeff = 0.61;
 Turb(TurbNum).BottCylDragCoeff = 0.68;
 Turb(TurbNum).AddedMassCoeff = 0.63;
 Turb(TurbNum).EffDragFactor = 0.5*Global.WaterDensity*(Turb(TurbNum).MidCylDragCoeff*Turb(TurbNum).MidCylDia*Turb(TurbNum).MidCylLength + 3*Turb(TurbNum).TopCylDragCoeff*Turb(TurbNum).TopCylDia*Turb(TurbNum).TopCylLength + 3*Turb(TurbNum).BottCylDragCoeff*Turb(TurbNum).BottCylDia*Turb(TurbNum).BottCylLength);
 Turb(TurbNum).AddedMass = pi/4*Turb(TurbNum).AddedMassCoeff*Global.WaterDensity*(Turb(TurbNum).MidCylLength*Turb(TurbNum).MidCylDia^2 + 3*Turb(TurbNum).TopCylLength*Turb(TurbNum).TopCylDia^2  + 3*Turb(TurbNum).BottCylLength*Turb(TurbNum).BottCylDia^2);
end

%% Mooring system properties
for TurbNum = Farm.NumTurb:-1:1
 % Cable mechanical properties
 Turb(TurbNum).CableNetDensity = 108.63;
 Turb(TurbNum).CableSpWeight = Global.GravAccel*Turb(TurbNum).CableNetDensity;
 Turb(TurbNum).CableStiff = 753.6e6;  
 Turb(TurbNum).SeabedFricCoeff = 1;
 
 % Cable dimensions
 Turb(TurbNum).CableLength = 910; % org: 980 960
 Turb(TurbNum).CableDia = 0.0766;
 Turb(TurbNum).CableArea = pi*(Turb(TurbNum).CableDia^2)/4;
 
 % Mooring system dimensions
 Turb(TurbNum).VerDist = 186;
 Turb(TurbNum).AnchDistRelG = 837.6;
 Turb(TurbNum).HorDistInit = Turb(TurbNum).AnchDistRelG - Turb(TurbNum).FLDistRelG;
 Turb(TurbNum).AnchAngleRelX = 0;
end

%% Wind farm layout
% Turbine spacing with wind direction = 0 deg

% Installed turbine positions with wind direction = 0 deg
TurbNum = Farm.NumTurb;
for TurbNumY = Farm.NumTurbY:-1:1
 for TurbNumX = Farm.NumTurbX:-1:1
  Turb(TurbNum).NeutralPosVec = zeros(2,1);
  Turb(TurbNum).NeutralPosVec(1) = 0 + (TurbNumX - 1)*Farm.SpacingX;
  Turb(TurbNum).NeutralPosVec(2) = 0 + (TurbNumY - 1)*Farm.SpacingY;
  TurbNum = TurbNum - 1;
 end
end 

% Transfom installation locations based on wind direction
Farm.AngleRelX = 0; % deg
Farm.CoordTransMat = [
 cosd(Farm.AngleRelX) -sind(Farm.AngleRelX);
 sind(Farm.AngleRelX) cosd(Farm.AngleRelX)];
TmpMat = zeros(2,Farm.NumTurb);
for TurbNum = 1:Farm.NumTurb
 Turb(TurbNum).NeutralPosVec = Farm.CoordTransMat*Turb(TurbNum).NeutralPosVec;
 TmpMat(:,TurbNum) = Turb(TurbNum).NeutralPosVec;
end
TmpMat = sortrows(TmpMat',1)';
for TurbNum = Farm.NumTurb:-1:1
 Turb(TurbNum).NeutralPosVec = TmpMat(:,TurbNum);
end

%% Calculate mooring system dimensions across wind farm
for TurbNum = Farm.NumTurb:-1:1
 % Transformation matrix for anchor locations
 Turb(TurbNum).AnchTransMat = [
  cosd(Turb(TurbNum).AnchAngleRelX) -sind(Turb(TurbNum).AnchAngleRelX);
  sind(Turb(TurbNum).AnchAngleRelX) cosd(Turb(TurbNum).AnchAngleRelX)];
 
 % Fairlead locations relative to turbine COG
 Turb(TurbNum).FL1PosRelG = Turb(TurbNum).FLDistRelG*Farm.CoordTransMat*Turb(TurbNum).AnchTransMat*[cosd(60);sind(60)];
 Turb(TurbNum).FL2PosRelG = Turb(TurbNum).FLDistRelG*Farm.CoordTransMat*Turb(TurbNum).AnchTransMat*[cosd(180);sind(180)];
 Turb(TurbNum).FL3PosRelG = Turb(TurbNum).FLDistRelG*Farm.CoordTransMat*Turb(TurbNum).AnchTransMat*[cosd(300);sind(300)];
 
 % Anchor locations relative to installed positions
 Turb(TurbNum).Anch1PosVec = Turb(TurbNum).AnchDistRelG*Farm.CoordTransMat*Turb(TurbNum).AnchTransMat*[cosd(60);sind(60)] + Turb(TurbNum).NeutralPosVec;
 Turb(TurbNum).Anch2PosVec = Turb(TurbNum).AnchDistRelG*Farm.CoordTransMat*Turb(TurbNum).AnchTransMat*[cosd(180);sind(180)] + Turb(TurbNum).NeutralPosVec;
 Turb(TurbNum).Anch3PosVec = Turb(TurbNum).AnchDistRelG*Farm.CoordTransMat*Turb(TurbNum).AnchTransMat*[cosd(300);sind(300)] + Turb(TurbNum).NeutralPosVec;
end

%% Generate lookup table for mooring line forces
 % Horizontal incremental spacing
 Moor.HorDistSpacing = 1;
 
 % Range of horizontal fairlead distances to simulate
 Moor.HorDistMin = 0;
 Moor.HorDistCat = Turb(1).CableLength - Turb(1).VerDist;
 Moor.HorForceTrans = (1 - (Turb(1).VerDist/Turb(1).CableLength - Turb(1).CableSpWeight*Turb(1).CableLength/(2*Turb(1).CableStiff))^2)*Turb(1).CableSpWeight*Turb(1).CableLength/(2*(Turb(1).VerDist/Turb(1).CableLength - Turb(1).CableSpWeight*Turb(1).CableLength/(2*Turb(1).CableStiff)));
 Moor.VerForceTrans = Turb(1).CableSpWeight*Turb(1).CableLength;
 Moor.HorDistTrans = (Moor.HorForceTrans/Turb(1).CableSpWeight)*(Turb(1).CableSpWeight*Turb(1).CableLength/Turb(1).CableStiff + asinh(Turb(1).CableSpWeight*Turb(1).CableLength/Moor.HorForceTrans));
 Moor.MaxFac = 1.5;
 Moor.HorDistMax = Moor.MaxFac*sqrt(Turb(1).CableLength^2 - Turb(1).VerDist^2);
 HorDistVec = (0:Moor.HorDistSpacing:Moor.HorDistMax)';
 
 % Solve catenary problem to generate lookup table
 HorForceVec = zeros(length(HorDistVec),1);
 VerForceVec = zeros(length(HorDistVec),1);
 Moor.HorForceGuess = Moor.HorForceTrans;
 Moor.VerForceGuess = Moor.VerForceTrans;
 Moor_fsolveOptions = optimoptions('fsolve',...
  'Display','off',...
  'FiniteDifferenceType','central',...
  'StepTolerance',1e-16,...
  'FunctionTolerance',1e-16,...
  'MaxFunctionEvaluations',10000,...
  'MaxIterations',10000,...
  'OptimalityTolerance',1e-16);
 for i = 1:length(HorDistVec)
  if HorDistVec(i) <= Moor.HorDistCat
   HorForceVec(i) = 0;
   VerForceVec(i) = Turb(1).CableSpWeight*Turb(1).VerDist;
  elseif HorDistVec(i) <= Moor.HorDistTrans
   Func = @(x) [
    (-HorDistVec(i) + (Turb(1).CableLength - x(2)/Turb(1).CableSpWeight +((1 + x(1)/Turb(1).CableStiff)^3 - ((1 + x(1)/Turb(1).CableStiff)^2 - 2*Turb(1).CableSpWeight*Turb(1).SeabedFricCoeff/Turb(1).CableStiff*min(Turb(1).CableLength - x(2)/Turb(1).CableSpWeight,x(1)*(1 + x(1)/(2*Turb(1).CableStiff))/(Turb(1).CableSpWeight*Turb(1).SeabedFricCoeff)))^(3/2))/(3*Turb(1).SeabedFricCoeff*Turb(1).CableSpWeight/Turb(1).CableStiff) - min(Turb(1).CableLength - x(2)/Turb(1).CableSpWeight,x(1)*(1 + x(1)/(2*Turb(1).CableStiff))/(Turb(1).CableSpWeight*Turb(1).SeabedFricCoeff))) + x(1)/Turb(1).CableSpWeight*(x(2)/Turb(1).CableStiff + asinh(x(2)/x(1))));
    (-Turb(1).VerDist + (1/Turb(1).CableSpWeight)*((x(2)^2)/(2*Turb(1).CableStiff) - x(1)*(1 - sqrt(1 + (x(2)/x(1))^2))))];
   Vars = fsolve(Func,[Moor.HorForceGuess Moor.VerForceGuess]',Moor_fsolveOptions);
   HorForceVec(i) = Vars(1);
   VerForceVec(i) = Vars(2);
  elseif HorDistVec(i) > Moor.HorDistTrans
   Func = @(x) [
    (-HorDistVec(i) + x(1)/Turb(1).CableSpWeight*(Turb(1).CableSpWeight*Turb(1).CableLength/Turb(1).CableStiff + asinh(x(2)/x(1)) - asinh((x(2) - Turb(1).CableSpWeight*Turb(1).CableLength)/x(1))));
    (-Turb(1).VerDist + (Turb(1).CableLength/Turb(1).CableStiff*(x(2) - Turb(1).CableSpWeight*...
    Turb(1).CableLength/2) + x(1)/Turb(1).CableSpWeight*(sqrt(1 + (x(2)/x(1))^2) - sqrt(1 + ((x(2) - Turb(1).CableSpWeight*Turb(1).CableLength)/x(1))^2))))];
   Vars = fsolve(Func,[Moor.HorForceGuess Moor.VerForceGuess]',Moor_fsolveOptions);
   HorForceVec(i) = Vars(1);
   VerForceVec(i) = Vars(2);
  end
 end
 
 % Save lookup table data
 HorDistData = HorDistVec;
 HorForceData = HorForceVec;
 VerForceData = VerForceVec;
 
%% Build finite difference mesh
Farm.ExtraSpacingX = 7*Global.TurbDia;
Farm.ExtraSpacingY = 3*Global.TurbDia;
Farm.MaxNumWakePoints = 1000;
Farm.LengthX = max(TmpMat(1,:)) - min(TmpMat(1,:));
Farm.LengthY = max(TmpMat(2,:)) - min(TmpMat(2,:));
Farm.TotalNumWakePoints = 0;
for TurbNum = Farm.NumTurb:-1:1
 Turb(TurbNum).DnsLength = Farm.LengthX*2 + Farm.ExtraSpacingX - (Turb(TurbNum).NeutralPosVec(1) - Turb(1).NeutralPosVec(1)); %check
 Turb(TurbNum).NumElm_A = ceil(Turb(TurbNum).DnsLength/Solver.DnsElmLengthA);
 Turb(TurbNum).NumElm = Turb(TurbNum).NumElm_A + Solver.NumMeshRedElm;
 Turb(TurbNum).ElmLength_A = Turb(TurbNum).DnsLength/Turb(TurbNum).NumElm_A;
 Turb(TurbNum).NumWakePoints = Turb(TurbNum).NumElm;
 Farm.TotalNumWakePoints = Farm.TotalNumWakePoints + Turb(TurbNum).NumWakePoints;
 Turb(TurbNum).WakePointPos_X = zeros(1,Farm.MaxNumWakePoints);
 for PointNum = 1:Turb(TurbNum).NumElm_A
  Turb(TurbNum).WakePointPos_X(PointNum) = PointNum*Turb(TurbNum).ElmLength_A;
 end
 ElmLengthTmp = Turb(TurbNum).ElmLength_A/2;
 for PointNum = Turb(TurbNum).NumElm_A + 1:Turb(TurbNum).NumWakePoints
  Turb(TurbNum).WakePointPos_X(PointNum) = Turb(TurbNum).WakePointPos_X(PointNum - 1) + ElmLengthTmp;
  ElmLengthTmp = ElmLengthTmp/2;
 end
end

%% Initialize dynamic arrays
Global.FreeStreamWindVel = InitFreeStreamWindVel;
Global.FreeStreamWindAccel = InitFreeStreamWindAccel;
Global.FreeStreamUnitVec = zeros(2,1);
for TurbNum = Farm.NumTurb:-1:1
 Turb(TurbNum).PosVec = zeros(2,1);
 Turb(TurbNum).VelVec = zeros(2,1);
 Turb(TurbNum).WakePointPos_Y = zeros(1,Farm.MaxNumWakePoints);
 Turb(TurbNum).WakePointVelVec = zeros(2,Farm.MaxNumWakePoints);
 Turb(TurbNum).WakePointDia = zeros(1,Farm.MaxNumWakePoints);
 Turb(TurbNum).AIFactor = 0;
 Turb(TurbNum).YawAngle = 0;
 Turb(TurbNum).Power = 0;
 Turb(TurbNum).ThrustCoeff = 0;
 Turb(TurbNum).PowerCoeff = 0;
 Turb(TurbNum).RotorNormVec = zeros(2,1);
 Turb(TurbNum).ThrustForceVec = zeros(2,1);
 Turb(TurbNum).UpsWindVelVec = zeros(2,1);
 Turb(TurbNum).RelUpsWindVel = zeros(2,1);
 Turb(TurbNum).DragForceVec = zeros(2,1);
 Turb(TurbNum).MoorForceVec = zeros(2,1);
 Turb(TurbNum).ForceVec = zeros(2,1);
 Turb(TurbNum).AccelVec = zeros(2,1);
 Turb(TurbNum).RelWindAngle = 0;
 Turb(TurbNum).RelYawAngle = 0;
 Turb(TurbNum).InitWakeVelVec = zeros(2,1);
 Turb(TurbNum).RelUpsWindVelVec = zeros(2,1);
 Turb(TurbNum).BladePitch=0;
 Turb(TurbNum).GeneTorque=0;
 Turb(TurbNum).RotorSpeed=0;
 Turb(TurbNum).NormalWindVel=0;
 Turb(TurbNum).Lambda=0;
 Turb(TurbNum).TurbTorque=0;
end
Arrays.TurbPosStateDerivMat = zeros(2,Farm.NumTurb);
Arrays.TurbVelStateDerivMat = zeros(2,Farm.NumTurb);
Arrays.WakeStateDerivMat = zeros(4,Farm.TotalNumWakePoints);
Arrays.TurbPowerVec = zeros(1,Farm.NumTurb);
Arrays.WakeLocalVel = zeros(1,Farm.MaxNumWakePoints);
Arrays.WakeLocalAccel = zeros(2,Farm.MaxNumWakePoints);
Arrays.WakeLocalDiaROT = zeros(1,Farm.MaxNumWakePoints);
Arrays.Output_PosX = zeros(Farm.NumTurb,1);
Arrays.Output_PosY = zeros(Farm.NumTurb,1);
Arrays.Output_Power = zeros(Farm.NumTurb,1);



%% Initial conditions
% Initialize arrays
InitTurbPosMat = zeros(2,Farm.NumTurb);
InitTurbVelMat = zeros(2,Farm.NumTurb);
InitWakeStateMat = zeros(4,Farm.TotalNumWakePoints);

% Fill initialized arrays
BaseNum = 0;
for TurbNum = 1:Farm.NumTurb
 % Turbine states
 InitTurbPosMat(:,TurbNum) = Turb(TurbNum).NeutralPosVec; %clarify
 InitTurbVelMat(:,TurbNum) = [0 0]';
 
 % Wake position states
 InitWakeStateMat(:,BaseNum + 1:BaseNum + Turb(TurbNum).NumWakePoints) = [
  [0 1]*InitFreeStreamWindVel/([1 0]*InitFreeStreamWindVel)*Turb(TurbNum).WakePointPos_X(1:Turb(TurbNum).NumWakePoints);
  (InitFreeStreamWindVel - InitTurbVelMat(:,TurbNum))*ones(1,Turb(TurbNum).NumWakePoints);
  Turb(TurbNum).RotorDia + Global.WakeExpRate/norm(InitFreeStreamWindVel)*Turb(TurbNum).WakePointPos_X(1:Turb(TurbNum).NumWakePoints)];
 
 % Update wake base number
 BaseNum = BaseNum + Turb(TurbNum).NumWakePoints;
end

% Build init state vector
InitStateVec = [
 reshape(InitTurbPosMat,[],1);
 reshape(InitTurbVelMat,[],1);
 reshape(InitWakeStateMat,[],1)];
StateDerivs = zeros(size(InitStateVec));

%% Run simulation
% Save parameters
save('Parameters_Simu.mat','Solver','Global','Farm','Turb','Moor','Arrays');

% Run Simulink
InitFreeStreamWindVel = Global.MeanWindSpeed;
sim('Simulink_StateDerivVec.slx');


%% Plot
time = ans.tout;

PosX = ans.PosX.signals.values;
PosY = ans.PosY.signals.values;
Power = ans.Power.signals.values;

figure(1)

subplot(3,1,1)
for i = 1:Farm.NumTurb
    stairs(time, PosX(:,i)); hold on;
end
title('Position X');
xlabel('Time [s]');
ylabel('Position [m]');
legend('Turbine1');
grid on


subplot(3,1,2)
for i = 1:Farm.NumTurb
    stairs(time, PosY(:,i)); hold on;
end
title('Position Y');
xlabel('Time [s]');
ylabel('Position [m]');
Legend = cell(Farm.NumTurb,1);
 for iter=1:Farm.NumTurb
   Legend{iter}=strcat('Turbine', num2str(iter));
 end
legend(Legend);
grid on

subplot(3,1,3)
for i = 1:Farm.NumTurb
    stairs(time, Power(:,i)); hold on;
end
title('Power');
xlabel('Time [s]');
ylabel('Power [W]');
Legend = cell(Farm.NumTurb,1);
 for iter=1:Farm.NumTurb
   Legend{iter}=strcat('Turbine', num2str(iter));
 end
legend(Legend);
grid on
