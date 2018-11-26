function [error_partials] = VehicleStateCentralDiff(state_index,delta, unperturb_states,unperturb_controls, constants, disturbance_vector)


    %reset to solution before perturbation
    states_k = unperturb_states;
    %/ perturb high
    states_k(state_index) =  states_k(state_index)*(1+delta);
    perturb_delta = states_k(state_index)*delta;
    %run the model
    [states, outputs] = VehicleDynamicsModel(states_k, unperturb_controls, disturbance_vector, constants);
    error_high = states(2:5);
    
    
    %reset to solution before perturbation
    states_k = unperturb_states;
    %/ perturb low
    states_k(state_index) = states_k(state_index)*(1-delta);
    %run the model
    [states, outputs] = VehicleDynamicsModel(states_k, unperturb_controls, disturbance_vector, constants);
    error_low = states(2:5);
    error_partials = (error_high-error_low)./(2*perturb_delta);
end