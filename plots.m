%% Plots
% Constant params (taken at max): lambda 

close all;

% Flight Time as a function of payload and Velocity
figure;
for i = 1:nStepsP
    plot(v_U, reshape(max(max(max(max(TIMES_U(:,:,:,:,:,i))))), [1, nStepsV]), 'DisplayName', strcat('P=', num2str(payload_U(i))));
    hold on
end
hold off
legend
xlabel('Cruise Velocity (m/s)');
ylabel('Flight Time (hr)');

figure;
for i = 1:nStepsP
    plot(v_U, reshape(max(max(max(max(RANGES_U(:,:,:,:,:,i))))), [1, nStepsV]), 'DisplayName', strcat('P=', num2str(payload_U(i))));
    hold on
end
hold off
legend
xlabel('Cruise Velocity (m/s)');
ylabel('Flight Range (km)');

figure;
for i = 1:nStepsP
    plot(lambda_U, reshape(max(max(max(max(TIMES_U(:,:,:,:,:,i))))), [1, nStepsLambda]), 'DisplayName', strcat('P=', num2str(payload_U(i))));
    hold on
end
hold off
legend
xlabel('Lambda (-)');
ylabel('Flight Time (hr)');

figure;
for i = 1:nStepsP
    plot(AR_U, reshape(max(max(max(max(TIMES_U(:,:,:,:,:,i))))), [1, nStepsAR]), 'DisplayName', strcat('P=', num2str(payload_U(i))));
    hold on
end
hold off
legend
xlabel('Aspect Ratio (-)');
ylabel('Flight Time (hr)');

figure;
for i = 1:nStepsP
    plot(AR_U, reshape(max(max(max(max(RANGES_U(:,:,:,:,:,i))))), [1, nStepsAR]), 'DisplayName', strcat('P=', num2str(payload_U(i))));
    hold on
end
hold off
legend
xlabel('Aspect Ratio (-)');
ylabel('Flight Range (km)');


figure;
for i = 1:nStepsAR
    plot(wing_U, reshape(max(max(max(max(TIMES_U(:,:,i,:,:,:))))), [1, nStepsWS]), 'DisplayName', strcat('AR=', num2str(AR_U(i))));
    hold on
end
hold off
legend
xlabel('Wingspan (m)');
ylabel('Flight Time (hr)');


figure;
for i = 1:nStepsAR
    plot(wing_U, reshape(max(max(max(max(RANGES_U(:,:,i,:,:,:))))), [1, nStepsWS]), 'DisplayName', strcat('AR=', num2str(AR_U(i))));
    hold on
end
hold off
legend
xlabel('Wingspan (m)');
ylabel('Flight Range (km)');


figure;
for i = 1:nStepsTHover
    plot(v_U, reshape(max(max(max(max(TIMES_U(:,i,:,:,:,:))))), [1, nStepsV]), 'DisplayName', strcat('t_h=', num2str(t_hover_U(i))));
    hold on
end
hold off
legend
xlabel('Cruise Velocity (m/s)');
ylabel('Flight Time (hr)');

figure;
for i = 1:nStepsTHover
    plot(v_U, reshape(max(max(max(max(RANGES_U(:,i,:,:,:,:))))), [1, nStepsV]), 'DisplayName', strcat('t_h=', num2str(t_hover_U(i))));
    hold on
end
hold off
legend
xlabel('Cruise Velocity (m/s)');
ylabel('Flight Range (km)');




figure;
for i = 1:nStepsWS
    plot(v_U, reshape(max(max(max(max(TIMES_U(:,:,:,i,:,:))))), [1, nStepsV]), 'DisplayName', strcat('WS=', num2str(wing_U(i))));
    hold on
end
hold off
legend
xlabel('Cruise Velocity (m/s)');
ylabel('Flight Time (hr)');

figure;
for i = 1:nStepsWS
    plot(v_U, reshape(max(max(max(max(RANGES_U(:,:,:,i,:,:))))), [1, nStepsV]), 'DisplayName', strcat('WS=', num2str(wing_U(i))));
    hold on
end
hold off
legend
xlabel('Cruise Velocity (m/s)');
ylabel('Flight Range (km)');
