%% This code contains the simulink models that can be used to simulate our control system. 
out=sim('ControlSystem.slx'); % Full motor control block diagram
%% A Plot of the results
%
load('ControlSystem.slx') % Real data under load
subplot(2,1,1)
plot(out.Voltage,'--','linewidth',2)
hold on
datar = data(:,1)-5500; % shift data over to account for the code delay
plot(datar(:,1)/1000,data(:,3),'linewidth',2)
xlim([0 2])
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')
subplot(2,1,2)
plot(out.Velocity,'--','linewidth',2)
hold on
data2 = data(:,1)-5500;
plot(data2(:,1)/1000,data(:,3),'linewidth',2)
legend('Simulated','Experimental')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
hold off
subplot(2,1,2)
plot(out.Velocity,'linewidth',2)
hold on
plot(out.DesiredVelocity,'--','linewidth',2) 
% This plot does not perfectly reach the desired speed due to steady state
% error. This is fixed in the arduino code with an integral term. 
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
xlim([0.5, 1.5])