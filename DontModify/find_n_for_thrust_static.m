function n = find_n_for_thrust_static(static_prop, Dprop, rho, T_desired)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function n = find_n_for_thrust_static(static_prop, Dprop, rho, T_desired)
% This function finds the rotations per second speed of given propeller
% which is stationary given the diameter and desired torque. This
% should only be used after you've calculated the desired torque needed for
% the vehicle
% Inputs:
% static_prop: structure with propeller CT and CP for different RPMs
% Dprop: the propeller diameter. . 
% rho: the air density
% T_desired: the desired thrust for a single propeller
% Outputs: 
% n: the propeller speed in rotations per second needed
% ME271E, Fall 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
options = optimset('Display','off');
n = fsolve(@(nh) interp1(static_prop.data(:,1),static_prop.data(:,2),60*nh)*rho*nh^2*Dprop^4-T_desired,300,options);