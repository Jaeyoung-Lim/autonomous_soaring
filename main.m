%% Autonomous dynamic soaring simulator
% Written by: Jaeyoung Lim jalim@student.ethz.ch

%% Initialize
clc; clear all;


state = struct('gamma_a', 0, ...
               'psi_a', 0, ...
               'phi', 0, ...
               'pos', [0.0, 0.0, -1.0], ...
               'vel', [20.0, 0.0, 0.0], ...
               'v_a', 20.0); 

roll_rate = 0.0;
pitch_rate = 0.0;

Jw = [0.0, 0.0, 0.0;
      0.0, 0.0, 0.0;
      0.0, 0.0, 0.0];
  
wind_gradient = Jw;
pos = [];

while true
    state = step(state, roll_rate, pitch_rate, wind_gradient);
    
    [~, ~, ~, pos1] = getstate(state);
    pos =[pos; pos1];
    figure(1);
    
    plot3(pos(:, 1), pos(:, 2), -pos(:, 3), 'r-');hold on; axis equal; zlim([0, 1]);
    if islanded(state)
       break; 
    end
end

function landed = islanded(state)
    if state.pos(3) >= 0
        landed = true;
    else
        landed = false;
    end
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
    
    % Calculate intertial forces
    Cia = rpy2rotmat(phi, gamma_a, psi_a);
    acc = (1 / m) * (Cia * [ -D, 0, -L]' + m * [0, 0, 9.8]')';
    
    vel = vel + acc * dt;
    pos = pos + vel * dt + 0.5 * acc * dt^2;
    v_a = v_a + dva_dt * dt ;
    psi_a = psi_a + dpsi_a_dt * dt;
    phi = phi + dphi_dt * dt;
    gamma_a = gamma_a + dgamma_a_dt * dt;
    
    state = setstate(gamma_a, psi_a, phi, pos, vel, v_a);
end

function rotmat = rpy2rotmat(phi, gamma, psi)
    
    Lz = [cos(psi), -sin(psi), 0;
          sin(psi), cos(psi), 0;
                 0,        0, 1];
    Ly = [cos(gamma), 0, sin(gamma);
          0         , 1,          0;
          -sin(gamma), 0, cos(gamma)];
    Lx = [1, 0, 0;
          0, cos(phi), -sin(phi)
          0, sin(phi),  cos(phi)];
    
    rotmat = Lz * Ly * Lx;
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