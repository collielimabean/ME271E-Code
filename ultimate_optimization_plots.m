%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot_figures_from_ultimate_optimization.m
% This script shows some plots based off the results of
% ultimate_optimization. ultimate_optimization should be run first. 
% ME271E, Fall 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% make a pie chart with the distribution of mass
avionics_mass = 0.132;
pie_avionics = avionics_mass; 
pie_propulsion = UAV_propulsionmass; 
pie_wings = UAV_wingmass;
pie_battery = UAV_mass*UAV_Batt_Fraction;
pie_payload = UAV_mass*UAV_payload_fraction;
pie_body = UAV_bodymass;
pie_total = UAV_mass;

mass_spectrum = [pie_body, pie_payload, pie_battery, pie_wings, pie_propulsion, pie_avionics];
labels = {'Body mass', 'Payload mass', 'Battery mass', 'Wing mass', 'Propulsion mass', 'Avionics mass'};

figure; 
pie(mass_spectrum,labels);
title('Distribution of mass in the UAV');

%%  plot a payload mass (y) vs range in km (x). also plot effects of wing area/aspect ratio
F_AR = zeros(size(RANGES_U));
F_WS = zeros(size(RANGES_U));
F_V = zeros(size(RANGES_U));
F_PL = zeros(size(RANGES_U));
for i = 1:length(RANGES_U(:))
    [j_ar,~,~,~] = ind2sub(size(RANGES_U),i);
    [~,j_w,~,~] = ind2sub(size(RANGES_U),i);
    [~,~,j_v,~] = ind2sub(size(RANGES_U),i);
    [~,~,~,j_m] = ind2sub(size(RANGES_U),i);
    F_AR(i) = AR_U(j_ar);
    F_WS(i) = wing_U(j_w);
    F_V(i) = v_U(j_v);
    F_PL(i) = payload_U(j_m);
end
figure;
colormap('parula')
sz = 25;
scatter(RANGES_U(:), F_PL(:), sz*F_WS(:)/max(F_WS(:)), F_V(:),'filled'); % scatter(x,y,size,color)
cb = colorbar;
cb.Label.String = 'Cruise speed [m/s]';
xlabel('Range [km]');
ylabel('Payload mass [kg]');
title('Size of dot indicates wingspan');

