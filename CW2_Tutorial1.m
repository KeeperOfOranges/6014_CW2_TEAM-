% ENGR6014 - Week 7 Seminar Task Template
clear all; clc

% Vehicle parameters
m = 750;            % Vehicle mass (kg)
g = 9.81;           % Gravitational acc. constant (m/s^2)
mu = 1.2;           % Friction coefficient (-)
CDA = 0.6;          % Aero. drag force factor (kg/m)
CLA = 1.2;          % Aero. downforce factor (kg/m)
D_aero = 0.45;      % Aero. CoP distribution - Front (-)
D_weight = 0.45;    % Weight distribution - Front (-)
density = 1.2;
% Corner radius
R_C = [120 150 100];

Motor_torque_lookup = [0 2000 4000 6000 8000 10000 12000 14000 16000 18000; 440 440 440 440 440 440 440 380 335 295]; 

for i = 1:length(R_C)

     % ... Equations of motion to be implemented here ...
    V_corner(i) = sqrt(mu*m*g*R_C(i) /(m- (mu*CLA*R_C(i))));
    Df_front(i) = CLA* D_aero * V_corner(i)^2;
    Df_Rear(i) = CLA * (1-D_aero) * V_corner(i)^2;
    DF_Total(i) = Df_front(i)+Df_Rear(i); 
    Drag_force(i) = CDA * V_corner(i)^2;
    F_Z_Front(i) = Df_front(i) + (m*D_weight)*g;
    F_Z_Rear(i) = Df_Rear(i) + (m*(1-D_weight)*g);
    F_Z_Total(i) = F_Z_Front(i)+F_Z_Rear(i);
    FDrive(i) = mu*(F_Z_Rear(i));

    ax(i) = (FDrive(i)-Drag_force(i))/m;



end
