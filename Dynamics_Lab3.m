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
ylabel('Amplitude (rads)')
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
time = 0.001:0.001:2;
u = ones(1,length(time)) .* 1;
t_beg1 = find(ExpData(:,1) == 255032);
ExpData_crop = ExpData(5672:5671+length(time), :);
%% Version 1: Model vs Hardware
figure();
hold on;
plot(time,(ExpData_crop(:,2)),'r');
lsim(sysTF - 0.5,u,time)
legend('Hardware Data','Model Data')
xlabel('Time (s)')
ylabel('Amplitude (rads)')
title('Model vs Hardware (Version 1)')
hold off

%% Version 1 hardware
figure();
hold on
xlabel('Time (ms)')
ylabel('Amplitude (rads)')
plot(ExpData(:,1),ExpData(:,2))
title('Hardware Time Response (Version 1)')
hold off

%% Version 2 Model
K1 = 15;
K3 = 1.5;
ExpData2 = readmatrix('3-20_exp2.txt');

num = (K1*Kg*Km) / (J * Rm);
den = [1 (((Kg^2 * Km^2) / (J*Rm)) + ((K3*Kg*Km)/(J*Rm))) ((K1*Kg*Km) / (J * Rm))];
sysTF = tf(num,den);
time = 0.001:0.001:2.5;
u = ones(1,length(time)) .* 1;
t_beg = find(ExpData2(:,1) == 4975020);
ExpData_crop2 = ExpData2(2587:2586+length(time), :);


figure();
hold on;
xlabel('Time (s)')
ylabel('Amplitude (rads)')
lsim(sysTF - 0.5,u,time)
title('Model (Version 2)')
hold off

%% Version 2: Model vs Hardware
Overshoot_HW = max(ExpData_crop2(:,2));
Overshoot_Model = 0.536;

figure();
hold on;
plot(time,(ExpData_crop2(:,2)),'r');
lsim(sysTF - 0.5,u,time)
yline(Overshoot_HW,'r--','LineWidth',1.5)
yline(Overshoot_Model,'b--','LineWidth',1.5)
xline(0.495,'r:','LineWidth',1.5)
xline(0.744,'b:','LineWidth',1.5)
xlabel('Time (s)')
ylabel('Amplitude (rads)')
legend('Hardware Data','Model Data','Hardware Overshoot (from 0.45 rads)','Model Overshoot (from 0.5 rads)','Hardware Settling Time', 'Model Settling Time')
title('Model vs Hardware (Version 2)')
hold off
%% Version 2: Hardware
figure();
hold on;
plot((ExpData2(:,1)),(ExpData2(:,2)))
xlabel('Time (ms)')
ylabel('Amplitude (rads)')
title('Hardware (Version 2)')
hold off

%% Natural Dampening Model
figure();
hold on;
xlabel('Time (s)')
ylabel('Amplitude (rads)')
plot(time,(ExpData_crop2(:,2)),'g');
lsim(sysTF - 0.5,u,time)

B = 0.06;
num = (K1*Kg*Km) / (J * Rm);
den = [1 (((Kg^2 * Km^2) / (J*Rm)) + ((K3*Kg*Km)/(J*Rm))) ((K1*Kg*Km) / (J * Rm)) + B/J];
sysTF = tf(num,den);
time = 0.001:0.001:2.5;
u = ones(1,length(time)) .* 1;
lsim(sysTF - 0.5,u,time);
xlabel('Time (ms)')
ylabel('Amplitude (rads)')
title('Natural Damping Comparison')
legend('Hardware','Model','Natural Damping')
hold off





