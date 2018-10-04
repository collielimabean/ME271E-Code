function n = find_n_for_thrust(propellerdata,RPM_propellers,v_cruise,Dprop, rho, T_desired)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function n = find_n_for_thrust(propellerdata,RPM_propellers,RPM_prop,v_cruise,Dprop, rho, T_desired)
% This function finds the rotations per second speed of given propeller
% given a cruise speed, diameter, air speed, and desired torque. This
% should only be used after you've calculated the desired torque needed for
% that cruise speed. 
% Inputs:
% propellerdata: structure with propeller CT, CP, eta for different RPMs
% and advance ratios
% RPM_propellers: the RPMs the propeller was measured for in propellerdata
% v_cruise: the cruise speed here. This is used to calculate the advance
% ratio to get the propeller parameters (CT and CP) at each iterated n
% Dprop: the propeller diameter. This is used for the advance ratio as
% well. 
% rho: the air density
% T_desired: the desired thrust for a single propeller
% Outputs: 
% n: the propeller speed in rotations per second needed
% ME271E, Fall 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
options = optimset('Display','off');
n = fsolve(@(nl) interp1(propellerdata(find(abs(60*nl-RPM_propellers)== min(abs(60*nl-RPM_propellers)))).data(:,1),propellerdata(find(abs(60*nl-RPM_propellers)== min(abs(60*nl-RPM_propellers)))).data(:,2),v_cruise/(nl*Dprop))*rho*nl^2*Dprop^4-T_desired,300,options);