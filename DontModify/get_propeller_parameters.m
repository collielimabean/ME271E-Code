function [eta, CP, CT] = get_propeller_parameters(propellerdata, RPM_propellers, RPM_prop, J)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [eta, CP, CT] = get_propeller_parameters(propellerdata, RPM_propellers, RPM_prop, J)
% This function looks up the propeller parameters for a given propeller at
% a particular RPM and advance ratio. For a static propeller, use
% get_propeller_parameters_static. 
% Inputs:
% propellerdata: structure with propeller CT, CP, eta for different RPMs
% and advance ratios
% RPM_propellers: the RPMs the propeller was measured for in propellerdata
% RPM_prop: the RPM at the point of interest
% J: the advance ratio at the point of interest
% Outputs: 
% eta: the propeller efficiency at the point of interest
% CP: the power coefficient at the point of interest
% CT: the thrust coefficient at the point of interest
% ME271E, Fall 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

propcurve_ID = find(abs(RPM_prop-RPM_propellers)== min(abs(RPM_prop-RPM_propellers)));
eta = interp1(propellerdata(propcurve_ID).data(:,1),propellerdata(propcurve_ID).data(:,4),J);
CP = interp1(propellerdata(propcurve_ID).data(:,1),propellerdata(propcurve_ID).data(:,3),J);
CT = interp1(propellerdata(propcurve_ID).data(:,1),propellerdata(propcurve_ID).data(:,2),J);
