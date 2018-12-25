%% Autonomous dynamic soaring simulator
% Written by: Jaeyoung Lim jalim@student.ethz.ch

%% Initialize
clc; clear all;


state = struct('gamma_a', 0, ...
               'psi_a', 0, ...
               'phi', 0, ...
               'pos', [0.0, 0.0, -1.0], ...
               'vel', [0.1, 0.0, 0.0], ...
               'v_a', 0.1); 

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

