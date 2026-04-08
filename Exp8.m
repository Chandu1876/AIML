clc;
clear;
close all;
Vin = 12;       
Vref = 24;      
L = 100e-6;     
C = 100e-6;     
R = 48;         
fs = 50e3;      
dt = 1/fs;      
t = 0:dt:0.01;  
objectiveFunction = @(K) boostCost(Vin,Vref,L,C,R,fs,t,K(1),K(2));
lb = [0 0];
ub = [10 10];
options = optimoptions('ga','PopulationSize',40,'MaxGenerations',60,'Display','iter');
[K_opt, cost_opt] = ga(objectiveFunction,2,[],[],[],[],lb,ub,[],options);
Kp = K_opt(1);
Ki = K_opt(2);
disp('Optimized PI Gains:')
disp(['Kp = ',num2str(Kp)])
disp(['Ki = ',num2str(Ki)])
[vout, il, vc] = boostSimulation(Vin,Vref,L,C,R,fs,t,Kp,Ki);
figure
subplot(3,1,1)
plot(t,vout,'LineWidth',2)
xlabel('Time (s)')
ylabel('Output Voltage (V)')
title('Boost Converter Output Voltage')
grid on 
subplot(3,1,2)
plot(t,il,'LineWidth',2)
xlabel('Time (s)')
ylabel('Inductor Current (A)')
title('Inductor Current')
grid on 
subplot(3,1,3)
plot(t,vc,'LineWidth',2)
xlabel('Time (s)')
ylabel('Capacitor Voltage (V)')
title('Capacitor Voltage')
grid on
function cost = boostCost(Vin,Vref,L,C,R,fs,t,Kp,Ki) 
[vout,~,~] = boostSimulation(Vin,Vref,L,C,R,fs,t,Kp,Ki); 
error = Vref - vout(end);
cost = error^2;
end
function [vout, il, vc] = boostSimulation(Vin,Vref,L,C,R,fs,t,Kp,Ki)
dt = 1/fs;
il = zeros(size(t));
vc = zeros(size(t));
vout = zeros(size(t));
integral = 0;
for i = 2:length(t)
    error = Vref - vc(i-1);
    integral = integral + error*dt;
    D = Kp*error + Ki*integral;
    if D > 0.9
        D = 0.9;
    elseif D < 0
        D = 0;
    end
    if mod(t(i),1/fs) < D*(1/fs)
        il(i) = il(i-1) + (Vin/L)*dt;
 
    else
         il(i) = il(i-1) + ((Vin - vc(i-1))/L)*dt;
    end
    vc(i) = vc(i-1) + ((il(i) - vc(i-1)/R)/C)*dt;
    vout(i) = vc(i);
end
end