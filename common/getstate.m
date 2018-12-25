function [gamma_a, psi_a, phi, pos, vel, v_a] = getstate(state)
    gamma_a = state.gamma_a;
    psi_a = state.psi_a;
    phi = state.phi;
    pos = state.pos;
    vel = state.vel;
    v_a = state.v_a;
end