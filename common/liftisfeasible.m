function feasible = liftisfeasible(state, Lift)
    constants();
    [~, ~, ~, ~, ~, v_a] = getstate(state);
    
    Cl = 2 * Lift / (rho * v_a^2 * S );
    
    if Cl <= Cl_max 
        feasible = true;
    else
        feasible = false;
    end
end
