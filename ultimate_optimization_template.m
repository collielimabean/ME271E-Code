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
min_payload = 0.5; % kg
max_payload = 4.5; % kg
nStepsP = 3;
% define the minimum and maximum cruise velocity
min_velocity = 12; % m/s
max_velocity = 30; % m/s
nStepsV = 3;
% define the minimum and maximum wingspan
min_wingspan = 1; % m
max_wingspan = 1.5; % m
nStepsWS = 3;
% define the minimum and maximum aspect ratio
min_AR = 6; % -
max_AR = 14; % -
nStepsAR = 3;
% define the minimum and maximum hover time 
min_t_hover = 45; % s
max_t_hover = 75; % s
nStepsTHover = 3;
% define the minimum and maximum lambda
min_lambda = 0.4; % - 
max_lambda = 1; % -
nStepsLambda = 3;

% use those to calculate the range for the optimization
payload_U = linspace(min_payload,max_payload,nStepsP);
v_U = linspace(min_velocity,max_velocity,nStepsV);
wing_U = linspace(min_wingspan,max_wingspan,nStepsWS);
AR_U = linspace(min_AR,max_AR,nStepsAR);
t_hover_U = linspace(min_t_hover, max_t_hover, nStepsTHover);
lambda_U = linspace(min_lambda, max_lambda, nStepsLambda);

%Define matrices for data collection
RANGES_U = zeros(nStepsLambda, nStepsTHover, nStepsAR, nStepsWS, nStepsV, nStepsP);
MASSES_U = zeros(nStepsLambda, nStepsTHover, nStepsAR, nStepsWS, nStepsV, nStepsP);
RPMS_U = zeros(nStepsLambda, nStepsTHover, nStepsAR, nStepsWS, nStepsV, nStepsP);
TIMES_U = zeros(nStepsLambda, nStepsTHover, nStepsAR, nStepsWS, nStepsV, nStepsP);
BATTS_U = zeros(nStepsLambda, nStepsTHover, nStepsAR, nStepsWS, nStepsV, nStepsP);
Payloads_U = zeros(nStepsLambda, nStepsTHover, nStepsAR, nStepsWS, nStepsV, nStepsP);
GlideRatio_U = zeros(nStepsLambda, nStepsTHover, nStepsAR, nStepsWS, nStepsV, nStepsP);
Eta_PRPLSN_U = zeros(nStepsLambda, nStepsTHover, nStepsAR, nStepsWS, nStepsV, nStepsP);
bodymass_U = zeros(nStepsLambda, nStepsTHover, nStepsAR, nStepsWS, nStepsV, nStepsP);
wingmass_U = zeros(nStepsLambda, nStepsTHover, nStepsAR, nStepsWS, nStepsV, nStepsP);
propulsionmass_U = zeros(nStepsLambda, nStepsTHover, nStepsAR, nStepsWS, nStepsV, nStepsP);

% Loop
for j_lambda = 1:nStepsLambda
    for j_t_hover = 1:nStepsTHover
        for j_ar = 1:nStepsAR
            for j_w = 1:nStepsWS
                for j_v = 1:nStepsV
                    for j_m = 1:nStepsP
                        mass_Uav = payload_U(j_m);
                        velocity_Uav = v_U(j_v);
                        wing_Uav = wing_U(j_w);
                        ar_Uav = AR_U(j_ar);
                        t_hover_UAV = t_hover_U(j_t_hover);
                        lambda_UAV = lambda_U(j_lambda);
                        
                        UAV = range_calculation_2(mass_Uav, velocity_Uav, wing_Uav, ar_Uav, t_hover_UAV, lambda_UAV);
                        RANGES_U(j_lambda, j_t_hover, j_ar,j_w,j_v,j_m) = UAV.range;
                        MASSES_U(j_lambda, j_t_hover,j_ar,j_w,j_v,j_m) = UAV.mass;
                        RPMS_U(j_lambda, j_t_hover,j_ar,j_w,j_v,j_m) = UAV.RPM;
                        TIMES_U(j_lambda, j_t_hover,j_ar,j_w,j_v,j_m) = UAV.FlightTime;
                        BATTS_U(j_lambda, j_t_hover,j_ar,j_w,j_v,j_m) = UAV.BattMass;
                        Payloads_U(j_lambda, j_t_hover,j_ar,j_w,j_v,j_m) = UAV.PayloadMass;
                        GlideRatio_U(j_lambda, j_t_hover,j_ar,j_w,j_v,j_m) = UAV.MaxGlideRatio;
                        Eta_PRPLSN_U(j_lambda, j_t_hover,j_ar,j_w,j_v,j_m) = UAV.eta_propulsion;
                        bodymass_U(j_lambda, j_t_hover,j_ar,j_w,j_v,j_m) = UAV.BODYMASS;
                        wingmass_U(j_lambda, j_t_hover,j_ar,j_w,j_v,j_m) = UAV.WINGMASS;
                        propulsionmass_U(j_lambda, j_t_hover,j_ar,j_w,j_v,j_m) = UAV.PROPULSIONMASS;
                        fprintf('L=%d/%d, t_h=%d/%d, AR=%d/%d, WS=%d/%d, Vel=%d/%d, PL=%d/%d\n', j_lambda, nStepsLambda, j_t_hover, nStepsTHover, j_ar,nStepsAR , j_w,nStepsWS , j_v,nStepsV , j_m,nStepsP);
                    end
                end
            end
        end
    end
end
% Find the Maximum Range (Index)
[MAXFLIGHTTIME,Index] = max(TIMES_U(:));
[MAXRANGE,~] = max(RANGES_U(:));
[A,B,C,D,E,F] = ind2sub(size(TIMES_U),Index);

% Obtain parameters at maximum range.
Lambda_Max = lambda_U(A);
THover_Max = t_hover_U(B);
AspectRatio_Max = AR_U(C);
WingSpan_Max = wing_U(D);
CruiseVelocity_Max = v_U(E);
Payload_Max = payload_U(F);

%% 
UAV_range = RANGES_U(A,B,C,D,E,F);
UAV_mass = MASSES_U(A,B,C,D,E,F);
UAV_RPMs = RPMS_U(A,B,C,D,E,F);
UAV_time = TIMES_U(A,B,C,D,E,F);
UAV_Batt_Fraction = BATTS_U(A,B,C,D,E,F);
UAV_payload_fraction = Payloads_U(A,B,C,D,E,F);
UAV_GlideRatio = GlideRatio_U(A,B,C,D,E,F);
UAV_eta_propulsion = Eta_PRPLSN_U(A,B,C,D,E,F);
UAV_bodymass = bodymass_U(A,B,C,D,E,F);
UAV_wingmass = wingmass_U(A,B,C,D,E,F);
UAV_propulsionmass = propulsionmass_U(A,B,C,D,E,F);

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
% for i = 1:nStepsAR
%     plot(v_U, reshape(max(RANGES_U(:, i, :, :)), 1, nStepsV), 'DisplayName', strcat('AR=', num2str(AR_U(i))));
%     hold on
% end
% 
% % additional simulated
% for i=1:nStepsAR
%     plot(v_U, reshape(max(RANGEFORM_U(:, i, :, :)), 1, nStepsV), 'LineStyle', '--', 'DisplayName', strcat('Formula AR=', num2str(AR_U(i))));
%     hold on
% end
% 
% hold off
% legend
% xlabel('Cruise Velocity (m/s)')
% ylabel('Maximum Range (km)')
% title('Part (c) (ii) Maximum Range vs Cruise Velocity')

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
