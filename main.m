%% Autonomous dynamic soaring simulator
% Written by: Jaeyoung Lim jalim@student.ethz.ch

% initialize();
%% Constants
constants();

% coeffs = struct('cd', 0, ...
%                'A', 0, ...
%                'S', 0, ...
%                'm', 1.0);
% 
state = struct('gamma_a', 0, ...
               'psi_a', 0, ...
               'phi', 0, ...
               'pos', [0.0, 0.0, 0.0], ...
               'vel', [0.0, 0.0, 0.0], ...
               'v_a', 0, ...
               'L', [0.0, 0.0, 0.0]); 
dgamma_a_dt = 0;
dphi_dt = 0;

pos = [0.0, 0.0, 0.0];
vel = [0.0, 0.0, 0.0];
v_a = 10.0;
L = 0.0;

C_d0 = 0.012;
m = 5.44;
dt = 0.1;

roll_rate = 1.0;
pitch_rate = 1.0;
Jw = [0.0, 0.0, 0.0;
      0.0, 0.0, 0.0;
      0.0, 0.0, 0.0];
  
wind_gradient = Jw;


while true
    state = step(state, roll_rate, pitch_rate, wind_gradient);
    
    
end

function state = step(state, roll_rate, pitch_rate, wind_gradient)
    
    constants();

    [gamma_a, psi_a, phi, pos, vel, v_a] = getstate(state);
    
    dgamma_a_dt = pitch_rate;
    dphi_dt = roll_rate;
    Jw = wind_gradient;
    

    L = m/cos(phi) * (v_a * dgamma_a_dt + g * cos(gamma_a) - [sin(gamma_a)*cos(psi_a), sin(gamma_a) * sin(psi_a), cos(gamma_a)]* Jw * vel');
    % dgamma_a_update = (1/state.v_a) * ( L / m * cos(phi) - g * cos(gamma_a) + [sin(gamma_a)*cos(psi_a), sin(gamma_a) * sin(psi_a), cos(gamma_a)]' * Jw * vel);
    D = 0.5 * rho * v_a^2 * S * Cd_0 + L^2 / (0.5 * rho * v_a^2 * S * pi() * AR * e);
    
    dva_dt = (-D) / m - g * sin(gamma_a) - [cos(gamma_a)*cos(psi_a), cos(gamma_a) * sin(psi_a), -sin(gamma_a)] * Jw * vel';
    dpsi_a_dt = (1 / (v_a * cos(gamma_a))) * (L / m * sin(phi) + [sin(psi_a), -cos(psi_a), 0] * Jw * vel');
    
    psi_a = psi_a + dpsi_a_dt * dt;
    phi = phi + dphi_dt * dt;
    gamma_a = gamma_a + dgamma_a_dt * dt;
    pos = pos + vel * dt;
    
    state = setstate(gamma_a, psi_a, phi, pos, vel, v_a);


end

function [gamma_a, psi_a, phi, pos, vel, v_a] = getstate(state)
    gamma_a = state.gamma_a;
    psi_a = state.psi_a;
    phi = state.phi;
    pos = state.pos;
    vel = state.vel;
    v_a = state.v_a;
end

function state = setstate(gamma_a, psi_a, phi, pos, vel, v_a)
    state.gamma_a = gamma_a;
    state.psi_a = psi_a;
    state.phi = phi;
    state.pos = pos;
    state.vel = vel;
    state.v_a = v_a;
end