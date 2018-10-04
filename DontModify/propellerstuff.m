function [propellerdata, RPM_propellers] = propellerstuff()
% using APC thin electric 9x6 
x1= importdata('apce_9x6_rd0988_4003.txt');
x2= importdata('apce_9x6_rd0989_5013.txt');
x3= importdata('apce_9x6_rd0990_5000.txt');
x4= importdata('apce_9x6_rd0991_6038.txt');
x5= importdata('apce_9x6_rd0992_6015.txt');
x6= importdata('apce_9x6_rd0993_6701.txt');
x7= importdata('apce_9x6_rd0994_6715.txt');
% select the nearby RPMs
RPM_propellers = [4003, (5013+5000)/2, (6038+6015)/2, (6701+6715)/2];
% put them together and sort
propellerdata(1).data = x1.data; 
propellerdata(1).data = sortrows(propellerdata(1).data,1);
propellerdata(2).data = [x2.data;x3.data(x3.data(:,1)>max(x2.data(:,1)),:)];
propellerdata(2).data = sortrows(propellerdata(2).data,1);
propellerdata(3).data = [x4.data;x5.data(x5.data(:,1)>max(x4.data(:,1)),:)];
propellerdata(3).data = sortrows(propellerdata(3).data,1);
propellerdata(4).data = [x6.data;x7.data(x7.data(:,1)>max(x6.data(:,1)),:)];
propellerdata(4).data = sortrows(propellerdata(4).data,1);
