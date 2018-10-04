%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Airfoil_Propeller_Data.m
% ME271E, Fall 2018
% Code to import data on the propeller and airfoil
% Airfoil: NACA23112
% Propeller: APC Thin Electric 9x6
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; % clear workspace
addpath('DontModify'); % Make sure Matlab can see all the functions it needs to call from the folder: 'DontModify'

%% Import airfoil data
AF_polars_name = 'NACA_23112_T1_Re0.250_M0.05_N9.0.txt';   % file with airfoil polars
AFpolars = importdata(AF_polars_name,' ',11);              % import the airfoil polaras

% decompose AFpolars data
c_d = AFpolars.data(:, 3);
c_l = AFpolars.data(:, 2);
alpha = AFpolars.data(:, 1);

%% Linear & Quadratic Fits
% Determine how airfoil drag depends on lift in the linear part of the 0-max lift region
% Uncomment the line below and fill in the appropriate range
linear_range = 14:33;
profile_drag = fit(c_l(linear_range), c_d(linear_range), 'poly2');
profile_lift = fit(alpha(linear_range), c_l(linear_range), 'poly1');

%% H1, Part B plots
c_d_hat = profile_drag(c_l(linear_range));
c_l_hat = profile_lift(alpha(linear_range));

figure
hold on
plot(c_d, c_l);
plot(c_d_hat, c_l(linear_range));
xlabel("c_d");
ylabel("c_l");
title("c_l vs c_d");
legend("data", "fit");

figure
hold on
plot(alpha, c_l);
plot(alpha(linear_range), c_l_hat);
xlabel("alpha");
ylabel("c_l");
title("c_l vs alpha");
legend("data", "fit");

%% propeller data
[propellerdata, RPM_propellers] = propellerstuff();     % run to get the propeller information
staticprop = 'apce_9x6_static_rd0987.txt';              % static prop data filename
static_prop = importdata(staticprop);                   % static prop data
% add on a zero and large RPM so MATLAB can interpolate in the static case
static_prop.data = [0 mean(static_prop.data(:,2)) mean(static_prop.data(:,3)); static_prop.data; 50000 mean(static_prop.data(:,2)) mean(static_prop.data(:,3))];

save('airfoil_and_propeller_info' , 'profile_drag','propellerdata','RPM_propellers','static_prop');