%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ultimate_optimization.m
% This script iterates over a several dimensional matrix defining the
% design space for a given vehicle
% ME271E, Fall 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
tic;

%% Ultimate Optimization
% define the minimum and maximum payload
min_payload = 1; % kg
max_payload = 1; % kg
nStepsP = 1;
% define the minimum and maximum cruise velocity
min_velocity = 10; % m/s
max_velocity = 22; % m/s
nStepsV = 4;
% define the minimum and maximum wingspan
%in_wingspan = 1.55; % m
%max_wingspan = 1.55; % m
nStepsWS = 4;
% define the minimum and maximum aspect ratio
min_AR = 5; % -
max_AR = 14; % -
nStepsAR = 4;

% use those to calculate the range for the optimization
payload_U = linspace(min_payload,max_payload,nStepsP);
v_U = linspace(min_velocity,max_velocity,nStepsV);
%wing_U = linspace(min_wingspan,max_wingspan,nStepsWS);
AR_U = linspace(min_AR,max_AR,nStepsAR);
wing_U = sqrt((AR_U * 1.55^2)/6);

%Define matrices for data collection
RANGES_U = zeros(nStepsAR, nStepsWS, nStepsV, nStepsP);
MASSES_U = zeros(nStepsAR, nStepsWS, nStepsV, nStepsP);
RPMS_U = zeros(nStepsAR, nStepsWS, nStepsV, nStepsP);
TIMES_U = zeros(nStepsAR, nStepsWS, nStepsV, nStepsP);
BATTS_U = zeros(nStepsAR, nStepsWS, nStepsV, nStepsP);
Payloads_U = zeros(nStepsAR, nStepsWS, nStepsV, nStepsP);
GlideRatio_U = zeros(nStepsAR, nStepsWS, nStepsV, nStepsP);
Eta_PRPLSN_U = zeros(nStepsAR, nStepsWS, nStepsV, nStepsP);
bodymass_U = zeros(nStepsAR, nStepsWS, nStepsV, nStepsP);
wingmass_U = zeros(nStepsAR, nStepsWS, nStepsV, nStepsP);
propulsionmass_U = zeros(nStepsAR, nStepsWS, nStepsV, nStepsP);
RANGEFORM_U = zeros(nStepsAR, nStepsWS, nStepsV, nStepsP);


% Loop
for j_ar = 1:nStepsAR
    %for j_w = 1:nStepsWS
    for j_w = j_ar
        for j_v = 1:nStepsV
            for j_m = 1:nStepsP
                mass_Uav = payload_U(j_m);
                velocity_Uav = v_U(j_v);
                wing_Uav = wing_U(j_w);
                ar_Uav = AR_U(j_ar);
                UAV = range_calculation_2(mass_Uav, velocity_Uav, wing_Uav, ar_Uav);
                RANGES_U(j_ar,j_w,j_v,j_m) = UAV.range;
                MASSES_U(j_ar,j_w,j_v,j_m) = UAV.mass;
                RPMS_U(j_ar,j_w,j_v,j_m) = UAV.RPM;
                TIMES_U(j_ar,j_w,j_v,j_m) = UAV.FlightTime;
                BATTS_U(j_ar,j_w,j_v,j_m) = UAV.BattMass;
                Payloads_U(j_ar,j_w,j_v,j_m) = UAV.PayloadMass;
                GlideRatio_U(j_ar,j_w,j_v,j_m) = UAV.MaxGlideRatio;
                Eta_PRPLSN_U(j_ar,j_w,j_v,j_m) = UAV.eta_propulsion;
                bodymass_U(j_ar,j_w,j_v,j_m) = UAV.BODYMASS;
                wingmass_U(j_ar,j_w,j_v,j_m) = UAV.WINGMASS;
                propulsionmass_U(j_ar,j_w,j_v,j_m) = UAV.PROPULSIONMASS;
                RANGEFORM_U(j_ar, j_w, j_v, j_m) = UAV.rangeform;
                fprintf('AR=%d/%d, WS=%d/%d, Vel=%d/%d, PL=%d/%d\n', j_ar,nStepsAR , j_w,nStepsWS , j_v,nStepsV , j_m,nStepsP);
            end
        end
    end
end

% Find the Maximum Range (Index)
[MAX_RANGE_ULTIMATE,Index] = max(RANGES_U(:));
[A,B,C,D] = ind2sub(size(RANGES_U),Index);

% Obtain parameters at maximum range.
AspectRatio_Max = AR_U(A);
WingSpan_Max = wing_U(B);
CruiseVelocity_Max = v_U(C);
Payload_Max = payload_U(D);

%% 
UAV_range = RANGES_U(A,B,C,D);
UAV_mass = MASSES_U(A,B,C,D);
UAV_RPMs = RPMS_U(A,B,C,D);
UAV_time = TIMES_U(A,B,C,D);
UAV_Batt_Fraction = BATTS_U(A,B,C,D);
UAV_payload_fraction = Payloads_U(A,B,C,D);
UAV_GlideRatio = GlideRatio_U(A,B,C,D);
UAV_eta_propulsion = Eta_PRPLSN_U(A,B,C,D);
UAV_bodymass = bodymass_U(A,B,C,D);
UAV_wingmass = wingmass_U(A,B,C,D);
UAV_propulsionmass = propulsionmass_U(A,B,C,D);

%% Part c, part i
% for i = 1:nStepsAR
%     plot(v_U, reshape(max(RANGES_U(:, i, :, :)), 1, nStepsV), 'DisplayName', strcat('AR=', num2str(AR_U(i))));
%     hold on
% end
% hold off
% legend
% xlabel('Cruise Velocity (m/s)')
% ylabel('Maximum Range (km)')
% title('Part (c) (i) Maximum Range vs Cruise Velocity')

%% Part c, part ii
for i = 1:nStepsAR
    plot(v_U, reshape(max(RANGES_U(:, i, :, :)), 1, nStepsV), 'DisplayName', strcat('AR=', num2str(AR_U(i))));
    hold on
end

% additional simulated
for i=1:nStepsAR
    plot(v_U, reshape(max(RANGEFORM_U(:, i, :, :)), 1, nStepsV), 'LineStyle', '--', 'DisplayName', strcat('Formula AR=', num2str(AR_U(i))));
    hold on
end

hold off
legend
xlabel('Cruise Velocity (m/s)')
ylabel('Maximum Range (km)')
title('Part (c) (ii) Maximum Range vs Cruise Velocity')

%% Part d, aspect ratio
% for i = 1:nStepsP
%     plot(AR_U, RANGES_U(:, :, :, i), 'DisplayName', strcat('Payload=', num2str(payload_U(i))));
%     hold on
% end
% hold off
% legend
% xlabel('Aspect Ratio (-)')
% ylabel('Maximum Range (km)')
% title('Part (d) Maximum Range vs Aspect Ratio')

%% Part d, l/d ratio
% for i = 1:nStepsP
%     plot(AR_U, GlideRatio_U(:, :, :, i), 'DisplayName', strcat('Payload=', num2str(payload_U(i))));
%     hold on
% end
% hold off
% legend
% xlabel('Aspect Ratio (-)')
% ylabel('Glide Ratio (-)')
% title('Part (d) L/D Ratio vs Aspect Ratio')


%% Part e
% for i = 1:nStepsP
%     plot(wing_U, RANGES_U(:, :, :, i), 'DisplayName', strcat('Payload=', num2str(payload_U(i))));
%     hold on
% end
% hold off
% legend
% xlabel('Aspect Ratio (-)')
% ylabel('Maximum Range (km)')
% title('Part (e) Maximum Range vs Wingspan')

%%
toc;
