function [state_derivatives, outputs] = VehicleDynamicsModel(state_vector, control_vector, disturbance_vector, constants)

%{
============Constants=================

m = mass, kg
I_wheel = wheel inertia per wheel, kg.m^2
L_front = distant from cg to front axle, m
L_rear = distant from cg to rear axle, m
h_cg = distant from cg to ground, m
h_drag = distant from cg to drag axle, m
mu_static = static friction coefficient (nominal)
B = tire constant 1
C = tire constant 2
R = tire radius, m
mu_rolling = rolling friction coefficient
density = air density, kg/m^3
Cd_S = drag reference area, m^2
g = gravity, m/s^2
p_max = max power, kW
torque_max = max torque, Nm
tau_pt = drivetrain time constant, seconds

%}

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

%============input processing
x = state_vector(1);
V = state_vector(2);
omega_front = state_vector(3);
omega_rear = state_vector(4);
torque = state_vector(5);
u_acc = control_vector(1);
u_brk = control_vector(2);
theta = disturbance_vector(1);
V_wind = disturbance_vector(2);
d_mu_rolling = disturbance_vector(3);

%=============normalize inputs=========
F_gravity_x = m*g*sin(theta);
F_gravity_y = m*g*cos(theta);

Drag = 0.5*density*Cd_S*(V-V_wind)^2;
slip_ratio = @(omega) (V-omega*R)/V;



coef_friction = @(omega) mu_static*sin(C*atan(B*slip_ratio(omega)));

mu_front = coef_friction(omega_front);
mu_rear = coef_friction(omega_rear);

%normal forces on tires
Fn_rear = 0.5*(Drag*h_drag + F_gravity_y*(L_front+mu_front*h_cg))...
    /(L_rear+L_front-mu_rear*h_cg+mu_front*h_cg);
Fn_front = F_gravity_y/2-Fn_rear;



%Longitudinal forces;
Fx_front = coef_friction(omega_front)*Fn_front;
Fx_rear = coef_friction(omega_rear)*Fn_rear;

%sum of forces x axis
sum_Fx = 2*(Fx_front+Fx_rear)-Drag-F_gravity_x;
V_dot = sum_Fx/m;

%sum of torque tire
sum_tau_front = -Fn_front*mu_rolling*R-Fx_front*R;
sum_tau_rear = torque/2-Fn_rear*mu_rolling*R-(Fx_rear)*R;


omega_dot_front=sum_tau_front/I_wheel;
omega_dot_rear=sum_tau_rear/I_wheel;


%drive train
Tss_max = min(torque_max,p_max/omega_rear);
Tss = Tss_max*u_acc;
T_dot = 1.00/tau_pt*(Tss-torque);

p = 2.0*omega_dot_rear*torque/2.0;
x_km=x/1000.000;
V_kmph=V*3.600;
RPM_front=omega_front*30.00/pi;
RPM_rear=omega_rear*30.00/pi;
percent_sf=100.00*slip_ratio(omega_front);
percent_sr=100.00*slip_ratio(omega_rear);
percent_T=100.00*torque/torque_max;
percent_P=100.00*p/p_max;
%======Final outputs========

state_derivatives = [V;
    V_dot;
    omega_dot_front;
    omega_dot_rear;
    T_dot];

outputs = [
    x_km;
    V_kmph;
    RPM_front;
    RPM_rear;
    percent_sf;
    percent_sr;
    percent_T;
    percent_P

];

end
