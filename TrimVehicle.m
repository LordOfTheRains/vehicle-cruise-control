function [Xtrim, Ytrim, Utrim,XdotTrim] = TrimVehicle(Vkmph, constants, disturbance_vector)
%{

Xtrim - trimmed States
Ytrim - trimmed output vectors
Utrim - trimmed controls
XdotTrim - trimmed state derivatives


%}
tol = 10^-6;
delta = 0.05;
velocity_m_s = Vkmph/3.6000;


%=====================CONSTANTS===================
m = constants(1);
I_wheel = constants(2);
L_front = constants(3);
L_rear = constants(4);
h_cg = constants(5);
h_drag = constants(6);
mu_static = constants(7);
B = constants(8);
C = constants(9);
R = constants(10);
mu_rolling = constants(11);
density = constants(12);
Cd_S = constants(13);
g = constants(14);
p_max = constants(15);
torque_max = constants(16);
tau_pt = constants(17);

theta = disturbance_vector(1);
V_wind = disturbance_vector(2);
Drag = (density*Cd_S*(velocity_m_s-V_wind)^2)/2;


initial_uacc = (Drag + m*g*sin(theta)+mu_rolling*m*g*cos(theta))*velocity_m_s/p_max;
initial_wf = velocity_m_s/R;
initial_wr = initial_wf;
initial_T = (Drag + m*g*sin(theta)+mu_rolling*m*g*cos(theta))*R;

states_k = [0;velocity_m_s;initial_wf;initial_wr;initial_T];
controls_k = [initial_uacc;0];


[state_ders, outputs] = VehicleDynamicsModel(states_k, controls_k, disturbance_vector, constants);
error_vector = state_ders(2:5);
solution_error = norm(error_vector);
unperturb_states = states_k;
unperturb_controls = controls_k;
while(solution_error>tol)
    
    u_acc = unperturb_controls(1);
    states_sol = unperturb_states(3:5);
    curr_solution = transpose([u_acc transpose(states_sol)]);
    
    
    %reset to solution before perturbation
    states_k = unperturb_states;
    controls_k = unperturb_controls;
    %/ perturb uacc high
    controls_k(1) =  controls_k(1)*(1+delta);
    perturb_delta = controls_k(1)*delta;
    %run the model
    [state_ders, outputs] = VehicleDynamicsModel(states_k, controls_k, disturbance_vector, constants);
    error_high = state_ders(2:5);
    
    
    %reset to solution before perturbation
    states_k = unperturb_states;
    controls_k = unperturb_controls;
    %/ perturb uacc low
    controls_k(1) = controls_k(1)*(1-delta);
    %run the model
    [state_ders, outputs] = VehicleDynamicsModel(states_k, controls_k, disturbance_vector, constants);
    error_low = state_ders(2:5);
    u_acc_error_partials = (error_high-error_low)./(2*perturb_delta);
    jacobian(:,1) = u_acc_error_partials;
    
    %perturb w_f
    jacobian(:,2) = VehicleStateCentralDiff(3,delta, unperturb_states,unperturb_controls, constants, disturbance_vector);
    %perturb w_r
    jacobian(:,3) = VehicleStateCentralDiff(4,delta, unperturb_states,unperturb_controls, constants, disturbance_vector);
    %perturb T
    jacobian(:,4) = VehicleStateCentralDiff(5,delta, unperturb_states,unperturb_controls, constants, disturbance_vector);
    
    solution_delta = inv(jacobian)*error_vector;
    next_solution = curr_solution - solution_delta;
   
    unperturb_controls(1) = next_solution(1);
    unperturb_states(3:5) = next_solution(2:4);
    
    
    [state_ders, outputs] = VehicleDynamicsModel(unperturb_states, unperturb_controls, disturbance_vector, constants);
    error_vector = state_ders(2:5);
    
    solution_error = norm(error_vector);
    Xtrim = unperturb_states;
    Ytrim = outputs;
    Utrim = unperturb_controls;
    XdotTrim = state_ders;
    
end


end