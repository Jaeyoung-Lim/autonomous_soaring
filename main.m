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

while true
    state = step(state, roll_rate, pitch_rate, wind_gradient);
    
    [~, ~, ~, pos1] = getstate(state);
    pos =[pos; pos1];
    figure(1);
    
    plot3(pos(:, 1), pos(:, 2), -pos(:, 3), 'r-');hold on; axis equal; grid on; zlim([0, 3]);
    plotaircraft(state); hold off;
    if islanded(state)
       break; 
    end
end

function plotaircraft(state)
    [gamma_a, psi_a, phi, pos] = getstate(state);
    R = rpy2rotmat(phi, gamma_a, psi_a);
    
    wingspan = 0.5 * 2.4;
    wingchord = wingspan / 15;
    tailspan = 0.5 * 0.8;
    tailchord = wingchord;
    tailpos = 0.8;

    % Draw main wing
    X = [      0,  wingchord, wingchord, 0, 0,  wingchord, wingchord, 0];
    Y = [wingspan, wingspan,          0, 0, -wingspan, -wingspan, 0, 0];
    Z = [0 0 0 0 0 0 0 0];
    C = [0.5000 1.0000, 1.0000 1.0000 0.5000 1.0000, 1.0000 1.0000];
    vert = [X; Y; Z];
    vert = R * vert;
    X = vert(1, :);
    Y = vert(2, :);
    Z = -vert(3, :);
    
    X = X + pos(1);
    Y = Y + pos(2);
    Z = Z - pos(3);
    
    fill3(X,Y,Z,C);

    % Draw tail wing
    X = [      0,  tailchord, tailchord, 0, 0,  tailchord, tailchord, 0];
    X = X - tailpos;
    Y = [tailspan, tailspan,          0, 0, -tailspan, -tailspan, 0, 0];
    Z = [0 0 0 0 0 0 0 0];
    C = [0.5000 1.0000, 1.0000 1.0000 0.5000 1.0000, 1.0000 1.0000];
    Z = Z - 0.3;
    vert = [X; Y; Z];
    vert = R * vert;
    X = vert(1, :);
    Y = vert(2, :);
    Z = -vert(3, :);

    X = X + pos(1);
    Y = Y + pos(2);
    Z = Z - pos(3);
    fill3(X,Y,Z,C);
    
    % Draw Vertical Stabilizer
    
    % Draw fuselarge
    t = 0:pi/20:pi;
    
    [X,Y,Z] = cylinder(0.1 * sin(t));
    X = X(:)';
    Y = Y(:)';
    Z = Z(:)';
    Z = Z - 0.5;
    vert = [X; Y; Z];
    R1 = rpy2rotmat(0, -pi()/2, 0);
    vert = R * R1 * vert;
    X = vert(1, :);
    Y = vert(2, :);
    Z = -vert(3, :);
    
    X = X + pos(1);
    Y = Y + pos(2);
    Z = Z - pos(3);
    X = reshape(X, [size(t, 2), size(t, 2)]);
    Y = reshape(Y, [size(t, 2), size(t, 2)]);
    Z = reshape(Z, [size(t, 2), size(t, 2)]);
    
    surf(X,Y,Z)
    t = 0:pi/20:pi;
    t = ones(size(t));
    [X,Y,Z] = cylinder(0.05 * t);
    X = X(:)';
    Y = Y(:)';
    Z = Z(:)';
    Z = Z - 0.1;
    vert = [X; Y; Z];
    R1 = rpy2rotmat(0, -pi()/2, 0);
    vert = R * R1 * vert;
    X = vert(1, :);
    Y = vert(2, :);
    Z = -vert(3, :);
    
    X = X + pos(1);
    Y = Y + pos(2);
    Z = Z - pos(3);
    X = reshape(X, [size(t, 2), size(t, 2)]);
    Y = reshape(Y, [size(t, 2), size(t, 2)]);
    Z = reshape(Z, [size(t, 2), size(t, 2)]);
    
    surf(X,Y,Z)

    

end