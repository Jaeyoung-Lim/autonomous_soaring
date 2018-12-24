%% Autonomous dynamic soaring simulator
% Written by: Jaeyoung Lim jalim@student.ethz.ch

% initialize();
%% Constants
rho = 1.1; %
b = 4.32; % Wing span
S = 0.957; % Wing reference area
g = 9.8; % gravity
AR = 19.54; % Wing aspect ratio
e = 0.85; % Oswald's efficiency factor
CD_0 = 0.012; % Parasite drag coeffient
m = 5.44;
CL_max = 1.2; % Maximum Lift Coefficient
dphidt_max = 30; % Maximum roll rate


% coeffs = struct('cd', 0, ...
%                'A', 0, ...
%                'S', 0, ...
%                'm', 1.0);
% 
% state = struct('gamma_a', 0, ...
%                'psi_a', 0, ...
%                'phi', 0, ...
%                'pos', [0.0, 0.0, 0.0], ...
%                'v_a', 0, ...
%                'L', [0.0, 0.0, 0.0]); 
           
gamma_a = 0;
dgamma_a_dt = 0;
vel = [0.0, 0.0, 0.0];
phi = 0;
psi_a = 0;
pos = [0.0, 0.0, 0.0];
v_a = 10.0;
L = 0.0;
Jw = [0.0, 0.0, 0.0;
      0.0, 0.0, 0.0;
      0.0, 0.0, 0.0];
C_d0 = 0.012;
dt = 0.1;

while true
    %     state = step(coeffs, state, wind_gradient);
    L = m/cos(phi) * (v_a * dgamma_a_dt + g * cos(gamma_a) - [sin(gamma_a)*cos(psi_a), sin(gamma_a) * sin(psi_a), cos(gamma_a)]* Jw * vel');
    % dgamma_a_update = (1/state.v_a) * ( L / m * cos(phi) - g * cos(gamma_a) + [sin(gamma_a)*cos(psi_a), sin(gamma_a) * sin(psi_a), cos(gamma_a)]' * Jw * vel);
    D = 0.5 * rho * v_a^2 * S * C_d0 + L^2 / (0.5 * rho * v_a^2 * S * pi() * AR * e);
    
    dva_dt = (-D) / m - g * sin(gamma_a) - [cos(gamma_a)*cos(psi_a), cos(gamma_a) * sin(psi_a), -sin(gamma_a)] * Jw * vel';
    dpsi_a_dt = (1 / (v_a * cos(gamma_a))) * (L / m * sin(phi) + [sin(psi_a), -cos(psi_a), 0] * Jw * vel');
    
    psi_a = psi_a + dpsi_a_dt * dt;
    gamma_a = gamma_a + dgamma_a_dt * dt;
    pos = pos + vel * dt;
    
    
end
% function state = step(coeffs, state, wind_gradient)
% 
% 
% 
% end