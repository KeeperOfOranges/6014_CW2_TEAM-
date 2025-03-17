clear all all;
close all force;

%● Rear-wheel driven electric powertrain configuration
%● Steady-state, constant speed cornering around a circular arc (pure lateral acceleration)
%● Braking and acceleration in a straight line (pure longitudinal acceleration)
%● While power needs to be applied during cornering at constant speed to overcome drag forces,
%application of power to rear wheels does not reduce grip available for cornering (i.e. no traction circle)
%● Negligible yaw inertia effects (i.e. car transitions from straight-line motion to constant speed cornering
%motion instantaneously at the end of the braking zone)
% ● Negligible rolling resistance and wind speed
% ● Negligible rotational inertia of powertrain and wheels
% ● Tyre coefficient of friction is constant (vertical load dependency not considered)
% ● Track profile is assumed to be perfectly flat with no gradient/banking variations
% ● Single-track (bicycle) model with only two wheels
% ● For simplicity, assume that braking energy is not recovered (i.e. no regeneration capabilities)

%% ENGR6014: Motorsport Vehicle Performance 2024-2025
% Assignment 2 - Steady-State Laptime Simulator Development (Vehicle Parameters)
% Version: 1.0

ParametersVersion = '2024-2025: v1';

% Vehicle parameters:
m                   = 785;    % Vehicle Mass including driver [kg]
D_weight            = 0.44;   % Weight distribution - Front-to-Rear [-] 
wheelbase           = 3050;   % Wheelbase [mm]
COG_H               = 300;    % Centre of gravity height [mm]
Rad_tyre            = 330;    % Rear tyre rolling radius [mm]
mu_long             = 1.30;   % Tyre friction coefficient [-], Braking & Acceleration
mu_lat              = 1.35;   % Tyre friction coefficient [-], Cornering
D_aero              = 0.42;   % Aerodynamic balance - Front-to-Rear [-] 
CdA                 = 0.45;   % Aerodynamic drag factor [kg/m]
ClA                 = 0.65;   % Aerodynamic downforce factor [kg/m]
FDR                 = 8.50;   % Final drive ratio [-]
GR                  = 1.00;   % Gear ratio [-]
finalDrive_Eff      = 0.90;   % Final driveline efficiency [-]
Motor_torque_lookup = [0 2000 4000 6000 8000 10000 12000 14000 16000 18000; ...
    360 360 360 360 270 216 180 154 135 120]; % Motor torque lookup

% Circuit properties: 
Radius_corner       = [65 55 15 35];        % Radius of each corner [m]
Angle_corner        = [90 30 147 93];       % Angle of each corner [deg]
Length_straight     = [325 360 688 144];    % Length of each straight [m]

%Simulation parameters
Delta_S             = 0.1;   % Calculation step size interval [m]
g                   = 9.81;  % Graviational Constant


for i = 1:length(Radius_corner)

     % ... Equations of motion to be implemented here ...
    V_corner(i) = sqrt(mu_lat*m*g*Radius_corner(i) /(m- (mu_lat*ClA*Radius_corner(i))));
    Df_front(i) = ClA* D_aero * V_corner(i)^2;
    Df_Rear(i) = ClA * (1-D_aero) * V_corner(i)^2;
    DF_Total(i) = Df_front(i)+Df_Rear(i); 
    Drag_force(i) = CdA * V_corner(i)^2;
    F_Z_Front(i) = Df_front(i) + (m*D_weight)*g;
    F_Z_Rear(i) = Df_Rear(i) + (m*(1-D_weight)*g);
    F_Z_Total(i) = F_Z_Front(i)+F_Z_Rear(i);
    FwdDrive(i) = mu_lat*(F_Z_Rear(i));

    accel_Long(i) = (FDrive(i)-Drag_force(i))/m;



end

% Baseline Simulator Marking Criteria:
%   Your lap time simulators should consider the following sub-systems, and calculate the following quantities
%   and store them as vectors in MATLAB upon running of your codes (specific units that each quantity needs to
%   be calculated in indicated in brackets):

% A. Cornering Capabilities
%   Calculate the following cornering-related properties and resulting vehicle cornering capabilities to complete
%   the corner sections considered across the circuit:
%       ● Distance covered (travelled) in corner (m)
%       ● Cornering speed (m/s)
%       ● Elapsed time to complete corner (s)
%       ● Lateral (centripetal) acceleration in corner (m/s2)
% 12 marks
% B. Driving Capabilities
%   Calculate the acceleration capabilities under driving conditions upon the corner exit, assuming no braking
%   occurs in the straight-line section. For all the straight-line sections considered, calculate at each iteration:
%       ● Motor torque (Nm)
%       ● Total vertical load on rear wheels (N)
%       ● Traction-limited acceleration (m/s2)
%       ● Power-limited acceleration (m/s2)
%       ● Actual acceleration (m/s2)
%       ● Speed (m/s)
%           24 marks
% C. Braking Capabilities
%   Assuming brakes are released fully at the instant of corner entry, calculate the deceleration capabilities
%   prior to corner entry, assuming no driving occurs in the straight-line section considered.
%       ● Speed (m/s)
%       ● Drag force (N)
%       ● Downforce (N)
%       ● Deceleration (m/s2)
%           16 marks
% D. Complete Straight-line Behaviour
%   By considering the crossover points between the driving and braking calculation performed in above steps,
%   calculate the following for each one of the straight-line sections considered (from previous corner exit to
%   next corner entry):
%       ● Resulting acceleration profile (m/s2)
%       ● Resulting speed profile (m/s)
%       ● Time to complete each straight-line section (s)
%           4marks
% E. Complete Circuit Behaviour
%   In this step you should compile the relevant information from all previous calculations to build a complete
%   lap of the track.
%       ● Total time taken to complete a lap of the circuit displayed in the command window
%       ● MATLAB plots (automatically generated upon running the code) for the velocity trace (km/h),
%         acceleration trace (g) in the longitudinal and lateral directions as a function of distance travelled (m)
%           4 marks
% F. Simulator efficiency, flexibility, and clarity for future development
%   A good lap time simulator will perform the calculations as quickly as possible (while providing a good user
%   experience), in order that the simulator can run thousands of laps with different setup configurations in a
%   minimal amount of time. When assessing the performance of your codes, the following criteria will be
%   checked:
%       ● Efficiency: which will be measured in terms of how long it takes for your code to produce the required
%        results. Therefore, whilst developing your simulators you should consider how to structure it in a
%        computationally efficient manner.
%       ● Flexibility: which will be measured in terms of ease of changing parameters or circuit profiles by a 3rd
%        party.
%       ● Clarity: which will be measured in terms of inclusion of clearly annotated comments throughout the
%        code, such that it can be debugged or further developed by a 3rd party. Therefore, your annotations
%        should be helpful and informative yet concise.