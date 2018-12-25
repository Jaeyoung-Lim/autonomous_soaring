function state = step(state, roll_rate, pitch_rate, wind_gradient)
    
    constants();

    [gamma_a, psi_a, phi, pos, vel, v_a] = getstate(state);
    
    dphi_dt = roll_rate;
    Jw = wind_gradient;

    dgamma_a_dt = pitch_rate;    
    L = m/cos(phi) * (v_a * dgamma_a_dt + g * cos(gamma_a) - [sin(gamma_a)*cos(psi_a), sin(gamma_a) * sin(psi_a), cos(gamma_a)]* Jw * vel');
    % Check if lift is feasible
    if ~liftisfeasible(state, L)
        dgamma_a_dt = (1/v_a) * ( L / m * cos(phi) - g * cos(gamma_a) + [sin(gamma_a)*cos(psi_a), sin(gamma_a) * sin(psi_a), cos(gamma_a)] * Jw * vel');
    end
    
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
