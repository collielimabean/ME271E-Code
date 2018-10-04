% function UAV = range_calculation_2(m_payload,v_cruise,b,AR)

addpath('DontModify'); % Make sure Matlab can see all the functions it needs to call from the folder: 'DontModify'

Modify = 10; % Placeholder! Replace all occurrences of 'Modify' with the proper equations or variables

% load in the airfoil and propeller data that you calculated earlier. Make
% sure to have run Airfoil_Propeller_Data first
load airfoil_and_propeller_info
m_payload = 1.0;                                            % kg, payload mass
v_cruise = 17;                                              % m/s, cruise speed
b = 1.55;                                                   % m, wingspan
AR = 6;                                                     % -, aspect ratio

% Start with 14000 RPM in problem 1
RPMs = 14000;
% Use this range of RPMs for the final part of problem 1
% RPMs = 4000:500:40000;

% store vectors for iterating over the RPMs
Routput = nan+zeros(size(RPMs));
CL_temp = nan+zeros(size(RPMs));
J_level_iter = nan+zeros(size(RPMs));
GlideRatio = zeros(size(RPMs));
m_total_output = zeros(size(RPMs));
FlightTime = zeros(size(RPMs));
Percent_BattMass = zeros(size(RPMs));
Percent_PayloadMass = zeros(size(RPMs));
eta_props = zeros(size(RPMs));
BODYMASS = zeros(size(RPMs));
WINGMASS = zeros(size(RPMs));
PROPULSIONMASS = zeros(size(RPMs));
% iterate over the RPMs to get the final value
for RPMk = 1:length(RPMs)
    RPM_prop = RPMs(RPMk);
    %% input specific for your UAV
    n = RPM_prop/60;                                        % propeller rotations per second
    S = b^2/AR;                                             % wing area calculation
    n_props = 4;                                            % number of propellers
    m_avionics = 0.132;                                     % kg, avionics mass
    Dprop = 0.23;                                           % m, propeller diameter
    zeta = 1.3;                                             % max thrust to weight ratio (safety factor)
    m_prop = 10/1000;                                       % propeller weight (each)
    tovercroot = 0.12;                                      % wing root thickness to chord ratio 
    lambda = 1;                                             % -, taper ratio    
    % t_hover = 60 for the final two parts of problem 1 and for problem of 2
    t_hover = 0;                                            % s, hover time
    
    %% general information on batteries/efficiencies (Li-ion) - given
    k_battery = 150;                                        % battery energy density, Wh/kg
    f_battery = 0.8;                                        % fraction of battery useable
    k_esc = 14000;                                          % energy density of ESC, W/kg
    eta_esc = 0.9;                                          % ESC efficiency
    k_motor = 3400;                                         % energy density of motor, W/kg
    eta_motor = 0.7;                                        % efficiency of motor
    rho = 1.225;                                            % air density, kg/m^3
    g = 9.81;                                               % gravity, m/s^2
    eta_int = 0.95;                                         % interference loss, taken from table 2.3 in flemish thesis
    e = 0.7;                                                % oswald efficiency, assumed
    Nz = 4;                                                 % ultimate loading
    
    %% propeller info - insert your calculations here for the propeller
    % get propeller efficiency and CT and CP. Use get_propeller_parameters_static
    [CP_prop, CT_prop] = get_propeller_parameters_static(Modify);
    % get the propeller power and thrust, Pprop = ?; Tprop = ?;
    Pprop = Modify;                                          % propeller power, Watts
    Tprop = Modify;                                          % propeller thrust, N
    %% masses calculations
    m_total = Modify;                                        % total mass, kg
    m_fixed = Modify;                                        % fixed mass, kg
    m_body = Modify;                                         % body mass, kg
    m_wing = Modify;                                         % wing mass, kg
    m_esc = Modify;                                          % esc weight (each)
    m_motor = Modify;                                        % motor weight (each)
    m_battery = Modify;                                      % battery weight
    % calculate cargo bay size - include batteries + fixed. This is scaled and given for the first two problems. You may change it for your own vehicle as long as you justify your changes.
    m_hold = m_fixed + m_battery;                            % cargo bay size
    BODYMASS(RPMk) = m_body;
    WINGMASS(RPMk) = m_wing;
    PROPULSIONMASS(RPMk) = n_props*(m_motor+m_esc+m_prop);
    dims = [.1 .2 .25]*(m_hold^(1/3));                      % calculate the cargo bay size based off this (empirical)
    hbody = dims(1); 
    lbody = dims(3);
    wbody = dims(2); 
    
    % if the battery mass is positive and the propeller fits then continue on with the analyis, otherwise stop - this is not a valid RPM    
    if(m_battery>0 && Dprop<wbody/1.25)   
        %% needed CL and CD
        Percent_BattMass(RPMk) = Modify;                     % calculate proportion battery mass compared to the weight of the entire vehicle
        Percent_PayloadMass(RPMk) = Modify;                  % calculate proportion payload mass compared to the weight of the entire vehicle
        % calculate the dynamic pressure, q
        q = Modify;
        % calculate the coefficient of lift
        CL = Modify;
        % check that CL is a reasonable value. If not, go on to the next iteration
        CLmax_possible = 1.27;
        if(CL>CLmax_possible*.9)
            CL_temp(RPMk) = nan;
            continue % Break out of the main "for" loop
        end
        CL_temp(RPMk) = CL;
        
        % get the profile drag for this CL. Use the profile_drag function you created earlier
        CD0 = Modify;
        % get the body drag using the relationship given in class
        ct_ratio = lbody/hbody;
        CDstar = getCDstar(ct_ratio);
        % calculate the scaled frontal drag coefficient
        Afront = Modify;        
        CDbodyfront = Modify;
        % calculate the total CD
        CD = Modify;
        % calculate the glide ratio here
        GlideRatio(RPMk) = Modify;
        % calculate the thrust needed for level flight
        D_level = Modify;
        
    %% calculate n for cruise flight and hovering flight
        Tprop_hover = Modify;
        Tprop_level = Modify;
        % calculate the needed thrust per propeller for both level and hovering flight       
        Tprop_hover_per_prop = Modify;
        Tprop_level_per_prop = Modify;
        % get the rotations per second needed for each of these using the provided functions        
        n_hover = find_n_for_thrust_static(Modify);
        n_level = find_n_for_thrust(Modify);
        % calculate the advance ratio for level flight
        J_level = Modify;
        J_level_iter(RPMk) = J_level;
        % get the propeller parameters for level flight
        [CP_hover, CT_hover] = get_propeller_parameters_static(Modify);
        [eta_level, CP_level, CT_level] = get_propeller_parameters(Modify); 
        % calculate the power needed for level and hovering flight
        Pprop_hover = Modify;
        Pprop_level = Modify;
        % store the propeller efficiency of each step, eta_props(RPMk) = ?;
        eta_props(RPMk) = Modify;           % propeller efficiency
        
    %% battery calculations
        % calculate the battery energy 
        E_battery = Modify;                                      % battery energy, Wh
        % calculate the battery power needed for hover and level flight
        Pbat_hover = Modify;
        Pbat_level = Modify;
        % calculate the time spent in level flight
        t_level = Modify;
        % calculate the time spent total in flight
        FlightTime(RPMk) = Modify;                               % time in hours
        % calculate the flight range
        R = Modify;                                              % Range, m
        R = R/1000;                                              % Range, km
        % store the range and mass
        Routput(RPMk) = R;
        m_total_output(RPMk) = m_total;
        
    %% Check range calculation against complete formula
        if t_hover == 0
            eta_propulsion = Modify;
            Rformula = Modify;      % range based on complete formula (m)
            Rformula = Rformula/1000; % convert to km
            fprintf('R from code: %6.2f, R from formula: %6.2f\n', R, Rformula);
        end
    else
        % if there is no battery mass or the propeller is too big, skip the iteration
        GlideRatio(RPMk) = NaN;
        Routput(RPMk) = NaN;
        m_total_output(RPMk) = NaN;
        eta_props(RPMk) = NaN;
        CL_temp(RPMk) = nan;
        J_level_iter(RPMk) = nan;
    end

end

% store all the outputs here
Index = find(Routput == nanmax(Routput),1); % If you want to maximize range
% Index = find(FlightTime == nanmax(FlightTime),1); % If you want to maximize flight time
if ~isempty(Index)
    UAV.range = Routput(Index);
    UAV.J = J_level_iter(Index);
    UAV.MaxGlideRatio = GlideRatio(Index);
    UAV.mass = m_total_output(Index);
    UAV.RPM = RPMs(Index);
    UAV.FlightTime = FlightTime(Index);
    UAV.BattMass = Percent_BattMass(Index);
    UAV.PayloadMass = Percent_PayloadMass(Index);
    UAV.eta_propulsion = eta_props(Index);
    UAV.BODYMASS = BODYMASS(Index);
    UAV.WINGMASS = WINGMASS(Index);
    UAV.PROPULSIONMASS = PROPULSIONMASS(Index);
else
    UAV.range = nan;
    UAV.J = nan;
    UAV.MaxGlideRatio = nan;
    UAV.mass = nan;
    UAV.RPM = nan;
    UAV.FlightTime = nan;
    UAV.BattMass = nan;
    UAV.PayloadMass = nan;
    UAV.eta_propulsion = nan;
    UAV.BODYMASS = nan;
    UAV.WINGMASS = nan;
    UAV.PROPULSIONMASS = nan;
end

%% this code plots the RPM outputs. You may use it to get the outputs for the first problem. 
% You should also use it in order to check that the RPM range input is correct for your vehicle
% 
% figure; 
% plot(RPMs,Routput, 'LineWidth', 2);
% set(gca, 'FontSize', 12);
% xlabel('max propeller RPM')
% ylabel('range (km)')
% fprintf('Max range = %6.2fkm\n', max(Routput));