%% 1. PHYSICAL PARAMETERS (TUNED FOR STABILITY)
M_total = 1.087;    % Exact mass from your table
L_com   = 0.08;     % 8cm (Estimate: Center of mass is higher due to top plate)
g       = 9.81;

% NEW: Increased Inertia.
% The top plate adds "swing weight" which actually makes it fall SLOWER.
J_robot = 0.006;    

% NEW: Stronger Motors. 
% Previous value (0.8) was too weak. 
% Real geared motors produce high torque. We bump this to 25.
K_motor = 25.0; 

Alpha_n = (M_total * g * L_com) / J_robot; 

% Numerator and Denominator
num_plant = [K_motor];
den_plant = [1  0  -Alpha_n]; 

%% 2. CONTROLLER PARAMETERS (From your Arduino Code)
Kp = 45.0;
Ki = 0.05;
Kd = 1.5;
dt = 0.005; % 5ms loop time

