function [CP, CT] = get_propeller_parameters_static(static_prop, RPM)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [CP, CT] = get_propeller_parameters_static(static_prop, RPM)
% This function looks up the propeller parameters for a given static
% propeller at a given RPM. For a moving propeller, use
% get_propeller_parameters. 
% Inputs:
% static_prop: structure with propeller CT and CP for different RPMs
% RPM: the RPM at the point of interest
% Outputs: 
% CP: the power coefficient at the point of interest
% CT: the thrust coefficient at the point of interest
% ME271E, Fall 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CP = interp1(static_prop.data(:,1),static_prop.data(:,3),RPM);
CT = interp1(static_prop.data(:,1),static_prop.data(:,2),RPM);