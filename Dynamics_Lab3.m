%% Dynamics Lab 3
clear
clc
close all

%% Constants
Kg = 33.3;
Km = 0.0401;
Rm = 19.2;
J_hub = 0.0005;
J_extra = 0.2 * 0.2794^2;
J_load = 0.0015;
J = J_hub +J_extra + J_load;

K1_set = [10 20 5 10 10 10]; %Kptheta
K3_set = [0 0 0 1 -1 -0.5]; %KDtheta

%% Set testing
figure();
hold on;
for i = 1: length(K1_set)
% Closed Loop System
num = (K1_set(i)*Kg*Km) / (J * Rm);
den = [1 (((Kg^2 * Km^2) / (J*Rm)) + ((K3_set(i)*Kg*Km)/(J*Rm))) ((K1_set(i)*Kg*Km) / (J * Rm))];
sysTF(i) = tf(num,den);
% Step Response
%[x,t] = step(sysTF);

%Lsim
time = 0:0.05:2.5;
u = ones(1,length(time)) .* 0.5;
lsim(sysTF(i),u,time)
end
title('Gain Testing')
legend('K1 = 10, K3 = 0','K1 = 20, K3 = 0','K1 = 5, K3 = 0','K1 = 10, K3 = 1','K1 = 10, K3 = -1','K1 = 10, K3 = -0.5',Location='best')

%% Personalized gains
K1 = 30;
K3 = 1.5;
ExpData = readmatrix('Group-1_10-Experimentaldata_30_1.txt');

% 1 - Time (ms)
% 2 - Position (rad)

num = (K1*Kg*Km) / (J * Rm);
den = [1 (((Kg^2 * Km^2) / (J*Rm)) + ((K3*Kg*Km)/(J*Rm))) ((K1*Kg*Km) / (J * Rm))];
sysTF = tf(num,den);

% Step Response
%[x,t] = step(sysTF);

%Lsim
time = 0.001:0.001:3;
u = ones(1,length(time)) .* 1;
ExpData_crop = ExpData(1:length(time), :);

figure();
hold on;
title('Model vs Hardware (Version 1)')
xlabel('Time (s)')
ylabel('Amplitude (rads)')
plot(time,(ExpData_crop(:,2)*-1),'r');
lsim(sysTF - 0.5,u,time)
title('Designed System')
hold off

figure();
hold on
title('Hardware Time Response (Version 1)')
xlabel('Time (ms)')
ylabel('Amplitude (rads)')
plot(ExpData(:,1),ExpData(:,2))
hold off