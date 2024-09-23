%% This code contains the simulink models that can be used to simulate our control system. 
KP = 6.25; % Proporational Constant
KI = 1; % Integral Constant

K = 1; % Motor K Parameter
sigma = 8; % Motor sigma parameter

open_system('ControlSystem');
out=sim('ControlSystem'); % Full motor control block diagram

%% A Plot of the results
figure(1);
plot(out.Position);
hold on;
plot(out.Voltage);
hold off;

xlabel("Time (s)");
ylabel("Position (m), Voltage (V)");
title("Position and Voltage vs time for a Position Controller")