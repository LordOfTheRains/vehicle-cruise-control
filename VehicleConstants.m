function [constants] = VehicleConstants(passengers)


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
p_max = constants(15)*1000;
torque_max = constants(16);
tau_pt = constants(17);
%}
%=====================CONSTANTS===================
constants = [
    1726+80*passengers;
    1.80;
    1.10;
    1.30;
    0.58;
    0.20;
    0.80;
    7;
    1.4;
    0.29;
    0.04;
    1.225;
    0.86;
    9.81;
    135000;
    4500;
    0.025
];
end