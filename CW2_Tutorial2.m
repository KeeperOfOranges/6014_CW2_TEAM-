% ENGR6014 - Week 8 Seminar Task Template
clear all; clc

% Vehicle parameters
m = 750;            % Vehicle mass (kg)
g = 9.81;           % Gravitational acc. constant (m/s^2)
mu = 1.2;           % Friction coefficient (-)
CDA = 0.6;          % Aero. drag force factor (kg/m)
CLA = 1.2;          % Aero. downforce factor (kg/m)
D_aero = 0.45;      % Aero. CoP distribution - Front (-)
D_weight = 0.45;    % Weight distribution - Front (-)
L = 2.7;            % Wheelbase (m)
h_cog = 0.35;       % CoG height (m)
F_tractive = 5000;  % Constant tractive effort at the wheel (N)

% Initial states
vel = 40;           % Initial velocity (m/s)
acc = 0;            % Initial acceleration (m/s^2)
s = 0;              % Initial distance (m)

% Simulation settings
ds = 1;             % Distance step interval (m)
    
while s < 10
    
    % ... Equations of motion to be implemented here ...
    
    Df_front = CLA* D_aero * vel^2;
    Df_Rear = CLA * (1-D_aero) * vel^2;
    DF_Total = Df_front+Df_Rear; 
    Drag_force = CDA * vel^2;
    F_Z_Front = Df_front + (m*D_weight)*g;
    F_Z_Rear = Df_Rear + (m*(1-D_weight)*g);
    F_Z_Total = F_Z_Front+F_Z_Rear;    
    FDrive = mu*(F_Z_Rear);

    acc = (FDrive-Drag_force)/m;

    LMT = (m*acc*h_cog)/(L);


    % Suvat equation for velocity calculation:
    vel = sqrt(vel.^2 + 2.*acc.*ds);
    
    % Update the distance:
    s = s + ds;
 end