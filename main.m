%% Autonomous dynamic soaring simulator
% Written by: Jaeyoung Lim jalim@student.ethz.ch

%% Initialize
clc; clear all;


state = struct('gamma_a', 0, ...
               'psi_a', 0, ...
               'phi', 0, ...
               'pos', [0.0, 0.0, -1.0], ...
               'vel', [10.0, 0.0, 0.0], ...
               'v_a', 10.0); 

roll_rate = 0.5;
pitch_rate = 0.5;

Jw = [0.0, 0.0, 0.0;
      0.0, 0.0, 0.0;
      0.0, 0.0, 0.0];
  
wind_gradient = Jw;
pos = [];
loop = 0;
while true
    state = step(state, roll_rate, pitch_rate, wind_gradient);
    
    [~, ~, ~, pos1] = getstate(state);
    pos =[pos; pos1];
    figure(1);
    
    plot3(pos(:, 1), pos(:, 2), -pos(:, 3), 'r-');hold on; axis equal; grid on; zlim([-1, 3]);
    xlabel('x Position [m]'); ylabel('y Position [m]'); zlabel('z Position [m]');
%     if loop - 60 >= 0 || loop == 0
%         plotaircraft(state); hold on;
%         loop = 0;
%     end
     plotaircraft(state); hold off;

    if islanded(state)
       break; 
    end
    loop = loop + 1;
end

