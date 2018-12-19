

% initialize();

rho = 1.1;
g = 9.8;

coeffs = struct('cd', 0, ...
               'A', 0, ...
               'S', 0, ...
               'm', 1.0);

state = struct('gamma_a', 0, ...
               'psi_a', 0, ...
               'phi', 0, ...
               'pos', [0.0, 0.0, 0.0], ...
               'v_a', 0, ...
               'L', [0.0, 0.0, 0.0]); 

while true
    state = step(coeffs, state, wind_gradient);
    
end

function state = step(coeffs, state, wind_gradient)
m = coeffs.m;

L_update = m/cos(phi) * (v_a * dgamma_a + g * cos(gamma_a) - [sin(gamma_a)*cos(psi_a), sin(gamma_a) * sin(psi_a), cos(gamma_a)]'* wind_gradient * vel);

dgamma_a_update = (1/state.v_a) * ( L / m * cos(phi) - g * cos(gamma_a) + [sin(gamma_a)*cos(psi_a), sin(gamma_a) * sin(psi_a), cos(gamma_a)]' * wind_gradient * vel);

D_update = 0.5 * rho * v_a^2 * S * C_d0 + L^2 / (9.5 * rho * 


end