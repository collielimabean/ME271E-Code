clear; close all;
m_payload = 4.5;
v_cruise = 24;
b = 1.5;
AR = 7;
t_hover = 60;
lambda = 0.5;
UAV = range_calculation_2(m_payload, v_cruise, b, AR, t_hover, lambda);