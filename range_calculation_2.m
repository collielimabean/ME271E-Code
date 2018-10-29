function UAV = range_calculation_2(m_payload, v_cruise, b, AR, t_hover, lambda)
    addpath('DontModify'); % Make sure Matlab can see all the functions it needs to call from the folder: 'DontModify'

    % load in the airfoil and propeller data that you calculated earlier. Make
    % sure to have run Airfoil_Propeller_Data first
    load airfoil_and_propeller_info
%     m_payload = 1.0;                                            % kg, payload mass
%     v_cruise = 17;                                              % m/s, cruise speed
%     b = 1.55;                                                   % m, wingspan
%     AR = 6;                                                     % -, aspect ratio

    % Start with 14000 RPM in problem 1
    %RPMs = 14000;
    % Use this range of RPMs for the final part of problem 1
    RPMs = 4000:500:40000;

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
    FWDPROPULSIONMASS = zeros(size(RPMs));
    RangeForm = zeros(size(RPMs));
    
    % iterate over the RPMs to get the final value
    for RPMk = 1:length(RPMs)
        RPM_prop = RPMs(RPMk);
        %% input specific for your UAV
        n = RPM_prop/60;                                        % propeller rotations per second
        S = b^2/AR;                                             % wing area calculation
        n_props = 5;                                            % total number of propellers
        n_hover_props = 4;                                      % number of hover props
        n_fwd_props = n_props - n_hover_props;                  % number of forward props
        m_avionics = 0.132;                                     % kg, avionics mass
        Dprop = 0.23;                                           % m, propeller diameter
        zeta = 1.3;                                             % max thrust to weight ratio (safety factor)
        m_prop = 10/1000;                                       % propeller weight (each)
        tovercroot = 0.12;                                      % wing root thickness to chord ratio 
        
        %lambda = 1;                                             % -, taper ratio    
        % t_hover = 60 for the final two parts of problem 1 and for problem of 2
        %t_hover = 60;                                            % s, hover time

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
        [CP_prop, CT_prop] = get_propeller_parameters_static(static_prop, RPM_prop);
        % get the propeller power and thrust, Pprop = ?; Tprop = ?;
        Pprop = CP_prop * rho * n^3 * Dprop^5;                % propeller power, Watts
        Tprop =  CT_prop * rho * n^2 * Dprop^4;               % propeller thrust, N
        %% masses calculations
        m_total = (eta_int * n_props * Tprop) / (g * zeta);            % total mass, kg
        m_fixed = m_payload + m_avionics;                        % fixed mass, kg
        m_body = 0.15 * m_total + 0.05 * m_payload;              % body mass, kg
        m_wing = 0.0038 * (Nz * m_total)^1.06 * (AR^0.38) * (S^0.25) * (1 + lambda)^0.21 * (tovercroot^-0.14); % wing mass, kg
        m_esc = (1 / k_esc) * (Pprop / eta_motor);               % esc weight (each)
        m_motor = (1 / k_motor) * Pprop;                         % motor weight (each)
        m_battery = m_total - m_body - m_wing - m_fixed - n_props * (m_motor + m_esc + m_prop); % battery weight
        % calculate cargo bay size - include batteries + fixed. This is scaled and given for the first two problems. You may change it for your own vehicle as long as you justify your changes.
        m_hold = m_fixed + m_battery;                            % cargo bay size
        BODYMASS(RPMk) = m_body;
        WINGMASS(RPMk) = m_wing;
        FWDPROPULSIONMASS(RPMk) = n_fwd_props * (m_motor + m_esc + m_prop);
        PROPULSIONMASS(RPMk) = n_hover_props * (m_motor + m_esc + m_prop);
        dims = [.1 .2 .25]*(m_hold^(1/3));                      % calculate the cargo bay size based off this (empirical)
        hbody = dims(1); 
        lbody = dims(3);
        wbody = dims(2); 

        % if the battery mass is positive and the propeller fits then continue on with the analyis, otherwise stop - this is not a valid RPM    
        if(m_battery>0 && Dprop<wbody/1.25)   
            %% needed CL and CD
            Percent_BattMass(RPMk) = m_battery / m_total;       % calculate proportion battery mass compared to the weight of the entire vehicle
            Percent_PayloadMass(RPMk) = m_payload / m_total;    % calculate proportion payload mass compared to the weight of the entire vehicle
            % calculate the dynamic pressure, q
            q = 0.5 * rho * v_cruise^2;
            % calculate the coefficient of lift
            CL = (m_total * g) / (q * S);
            % check that CL is a reasonable value. If not, go on to the next iteration
            CLmax_possible = 1.27;
            if(CL>CLmax_possible*.9)
                CL_temp(RPMk) = nan;
                continue % Break out of the main "for" loop
            end
            CL_temp(RPMk) = CL;

            % get the profile drag for this CL. Use the profile_drag function you created earlier
            CD0 = profile_drag(CL);
            % get the body drag using the relationship given in class
            ct_ratio = lbody/hbody;
            CDstar = getCDstar(ct_ratio);
            % calculate the scaled frontal drag coefficient
            Afront = wbody * hbody;        
            CDbodyfront = CDstar * (Afront / S);
            % calculate the total CD
            CD = CDbodyfront + CD0 + (CL^2) / (pi * AR * e);
            % calculate the glide ratio here
            GlideRatio(RPMk) = CL/CD;
            % calculate the thrust needed for level flight
            D_level = CD * q * S;

        %% calculate n for cruise flight and hovering flight
            Tprop_hover = m_total * g;
            Tprop_level = D_level;
            % calculate the needed thrust per propeller for both level and hovering flight       
            Tprop_hover_per_prop = Tprop_hover / n_hover_props;
            Tprop_level_per_prop = Tprop_level / n_fwd_props;
            % get the rotations per second needed for each of these using the provided functions        
            n_hover = find_n_for_thrust_static(static_prop, Dprop, rho, Tprop_hover_per_prop);
            n_level = find_n_for_thrust(propellerdata, RPM_propellers, v_cruise, Dprop, rho, Tprop_level_per_prop);
            % calculate the advance ratio for level flight
            J_level = v_cruise / (n_level * Dprop);
            J_level_iter(RPMk) = J_level;
            % get the propeller parameters for level flight
            [CP_hover, ~] = get_propeller_parameters_static(static_prop, RPM_prop);
            [~, CP_level, CT_level] = get_propeller_parameters(propellerdata, RPM_propellers, RPM_prop, J_level); 
            % calculate the power needed for level and hovering flight
            Pprop_hover = n_hover_props * CP_hover * rho * n_hover^3 * Dprop^5;
            Pprop_level = n_fwd_props * CP_level * rho * n_level^3 * Dprop^5;
            % store the propeller efficiency of each step, eta_props(RPMk) = ?;
            eta_props(RPMk) = J_level * (CT_level / CP_level);           % propeller efficiency

        %% battery calculations
            % calculate the battery energy 
            E_battery = m_battery * k_battery;                           % battery energy, Wh
            % calculate the battery power needed for hover and level flight
            Pbat_hover = Pprop_hover / (eta_esc * eta_motor);
            Pbat_level = Pprop_level / (eta_esc * eta_motor);
            % calculate the time spent in level flight
            t_level = (f_battery * (E_battery * 3600) - t_hover * Pbat_hover) / Pbat_level;
            % calculate the time spent total in flight
            FlightTime(RPMk) = t_level / 3600;                               % time in hours
            % calculate the flight range
            R = t_level * v_cruise;                                  % Range, m
            R = R/1000;                                              % Range, km
            % store the range and mass
            Routput(RPMk) = R;
            m_total_output(RPMk) = m_total;

        %% Check range calculation against complete formula
            if t_hover == 0
                eta_propulsion = eta_esc * eta_motor * eta_props(RPMk);
                Rformula = (k_battery * 3600 / g) * f_battery * Percent_BattMass * (CL / CD) * eta_propulsion; % range based on complete formula (m)
                Rformula = Rformula/1000; % convert to km
                RangeForm(RPMk) = max(Rformula);
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
            RangeForm(RPMk) = nan;
        end

    end

    % store all the outputs here
    %Index = find(Routput == nanmax(Routput),1); % If you want to maximize range
    Index = find(FlightTime == nanmax(FlightTime),1); % If you want to maximize flight time
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
        UAV.FWDPROPULSIONMASS = FWDPROPULSIONMASS(Index);
        UAV.rangeform = RangeForm(Index);
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
        UAV.FWDPROPULSIONMASS = nan;
        UAV.rangeform = nan;
    end

    %% this code plots the RPM outputs. You may use it to get the outputs for the first problem. 
    % You should also use it in order to check that the RPM range input is correct for your vehicle

%     figure; 
%     plot(RPMs,Routput, 'LineWidth', 2);
%     set(gca, 'FontSize', 12);
%     xlabel('max propeller RPM')
%     ylabel('range (km)')
%     fprintf('Max range = %6.2fkm\n', max(Routput));
end