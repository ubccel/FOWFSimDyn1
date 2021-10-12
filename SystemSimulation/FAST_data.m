windV=[3 4 5 6 7 8 9 10 11];
GenPr=[40.5 177.7 403.9 737.6 1187.2 1771.1 2518.6 3448.4 4562.5];    %kW
GenSpeed=[676.3 696.7 728.1 770.3 821.4 888.1 998.7 1109.0 1153.3];   %rpm
RotSpeed=[6.972 7.183 7.506 7.942 8.469 9.156 10.296 11.431 11.890];
GenTor=[0.606 2.58 5.611 9.686 14.620 20.174 25.510 31.455 40.014];
RotThrust=[171.7 215.9 268.9 330.3 398.6 478.0 579.2 691.5 790.6];  %kN
FAST_Ct=1000*RotThrust./(1/2*1.225*pi*63^2*windV.^2);
ss_rotspeed=0*RotSpeed;
for i=1:9
    ss_rotspeed(i)=get_rotorspeed(GenTor(i)*1000*97,windV(i),0);
end
Error=(ss_rotspeed-RotSpeed/60*2*pi)./RotSpeed*60/2/pi;
max(Error);
plot(windV,RotSpeed/60*2*pi);
hold on;
plot(windV,ss_rotspeed);
legend('FAST','MyModel');
xlabel('Wind Speed (m/s)');
ylabel('Rotor Speed (rad/s)');

%% Load Experimental Cp-Ct Data Points
load('paraVal.mat');
Lambda.mesh=paraVal.aero.lambda_grid;
Beta.mesh=paraVal.aero.beta_grid;
Cp.mesh=paraVal.aero.cp_grid;
Ct.mesh=paraVal.aero.ct_grid;

Data_Vector=[Lambda.mesh(:) Beta.mesh(:) Cp.mesh(:) Ct.mesh(:)];
Cp_interpolant=scatteredInterpolant(Lambda.mesh(:),Beta.mesh(:),Cp.mesh(:));
Ct_interpolant=scatteredInterpolant(Lambda.mesh(:),Beta.mesh(:),Ct.mesh(:));

%% Axial Induction Factor Compensator
ss_TSR=ss_rotspeed*63./windV;

figure;
plot(windV,ss_TSR);
xlabel('wind speed');
ylabel('Lambda');
ss_TSR_min=min(ss_TSR);
ss_TSR_max=max(ss_TSR);
TSR_vec=linspace(ss_TSR_min,ss_TSR_max);
BETA_vec=zeros(1,100);
ss_Cp=Cp_interpolant(TSR_vec,BETA_vec);
ss_Ct=Ct_interpolant(TSR_vec,BETA_vec);
figure;
plot(TSR_vec,ss_Cp);
hold on;
plot(TSR_vec,ss_Ct);

% Vortex Model
aifactor=linspace(1/3,0.5);
DefAngle = 0;
RelYawAngle=0;
ThrustCoeff = 4*aifactor.*(cos(RelYawAngle) + tan(DefAngle/2)*sin(RelYawAngle) - aifactor.*((sec(DefAngle/2))^2));
PowerCoeff = 0.8397*4*aifactor.*(cos(RelYawAngle) - aifactor).*(cos(RelYawAngle) + tan(DefAngle/2)*sin(RelYawAngle) - aifactor.*((sec(DefAngle/2))^2));
plot(TSR_vec,PowerCoeff);
plot(TSR_vec,ThrustCoeff);
legend('ss_Cp','ss_Ct','V_Cp','V_Ct');