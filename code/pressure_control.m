close all; clear all;


%% OBTAIN DATA
% Create serial object for Arduino
ser = serial('COM3'); % change the COM Port number as needed
set_point = 93000;
flag_s = 0;
flag_t = 0;
settling_time = 0;
time_1_3 = 0;
time_2_3 = 0;
delta_wye = 0;
final_time = 0;
start_time = 0;
delta_t =0;

% Connect the serial port to Arduino
ser.InputBufferSize = 1; % read only one byte every time
try
    fopen(ser);
catch err
    fclose(instrfind);
    error('Make sure you select the correct COM Port where the Arduino is connected.');
end
% Create a figure window to monitor the live data
Tmax = 60; % Total time for data collection (s)
figure,
grid on,
xlabel ('Time (s)'), ylabel('Current Pressure (Pa)'), title('Current Pressure vs Time ');
axis([0 Tmax+1 91000 95000]),
% Read and plot the data from Arduino
Ts = 0.035; % Sampling time (s)
i = 0;
data = 0;
t = 0;
tic % Start timer
while toc <= Tmax
    i = i + 1;
    % Read buffer data
    data_1(i) = fread(ser);
    data_2(i) = fread(ser);
    data_3(i) = fread(ser);
    data(i) = data_1(i)*256*256 + data_2(i)*256 + data_3(i) ;
    % Read time stamp
    % If reading faster than sampling rate, force sampling time.
    % If reading slower than sampling rate, nothing can be done. Consider
    % decreasing the set sampling time Ts
    t(i) = toc;
    if i > 1
        T = toc - t(i-1);
        while T < Ts
            T = toc - t(i-1);
        end
    end
    t(i) = toc;
    % Plot live data
    if i > 1
        line([t(i-1) t(i)],[data(i-1) data(i)])
        drawnow
    end
    % Starting time
    if(data(i)>(data(1)+100) & flag_t == 0)
        start_time=t(i);
        y0 = data(i-1);
        flag_t =1;
    end
    
    
     
     
    
end
fclose(ser);

%% Calculation of the Plant Transfer Function
[unique_output,~,jx] = unique(data); 
        unique_time = accumarray(jx, t ,[], @mean);
i=5;
[m,n] = size(unique_output);  
data_dot= abs(gradient(unique_output)./gradient(unique_time'));
while(i<n)        
        if (data_dot(i)+data_dot(i-1)+data_dot(i-3))/3 < 2 
        if unique_time(i) > start_time
        if flag_t==1
            settling_time = unique_time(i)
         delta_wye = (unique_output(i)-unique_output(2));
         flag_t=2;
                     
        end
        end
        end
     i=i+1; 
end
%    delta_wye = 218 - y0; 
    delta_u = 128 - 0; 
    y1b3 = y0 + delta_wye/3; 
    y2b3 = y0 + delta_wye*2/3; 
    Kp = delta_wye / delta_u 
    t1b3 = interp1(unique_output,unique_time,y1b3) 
    t2b3 = interp1(unique_output,unique_time,y2b3) 
    Tp = (1/0.69)*(t2b3-t1b3) 
    theta_p = t1b3 - 0.4*Tp  




s = tf('s');
G_plant = exp(-theta_p*s)*Kp/(Tp*s + 1);

figure(2)
hold on;
grid minor;
plot(t,data,'LineWidth',2);
title('Step Responses');
xlabel('Time(sec)');
ylabel('Pressure(Pa)');
ylim([91000 95000])
opt = stepDataOptions('InputOffset',(y0-100)/dcgain(G_plant),'StepAmplitude',delta_wye/(dcgain(G_plant)));
[step_y,step_t] = step(G_plant,60,opt);
plot(step_t,step_y,'LineWidth',2)
legend('Experimental Result','Step Response of the FOPDT Model');
hold off;

%% P-Controller with ITAE calculation 

Kc = 0.2/Kp*(Tp/theta_p)^1.22
Kc = 2.5*Kc; % aggressive
s = tf('s');
Controller = Kc;
G_plant = Kp/(Tp*s + 1);
composed_system = feedback(G_plant*Controller,1)*exp(-theta_p*s)
figure(4)
hold on;
grid minor;
plot(t,data,'LineWidth',2);
xlabel ('Time (s)'), ylabel('Current Pressure (Pa)'), title(strcat('Current Pressure vs Time (Kp = ',num2str(Kc,3),')'));
ylim([91000 93500]);
xlim([0 60])
opt = stepDataOptions('InputOffset',y0,'StepAmplitude',1.72e3/(dcgain(composed_system)));
[step_y,step_t] = step(composed_system,60,opt);
step_t = step_t + 0; % to substract the waiting time before controller starts
plot(step_t,step_y,'LineWidth',2)
legend('Experimental Result','Simulation');
hold off;

%% PI-Controller with IMC Rules 
Tc=max(0.1*Tp,0.8*theta_p)
Kc = Tp/Kp/(theta_p+Tc)
Ti=Tp
Ki=Kc/Ti
s = tf('s');
Controller = Kc*(1+1/(Ti*s));
G_plant = Kp/(Tp*s + 1);
composed_system = feedback(G_plant*Controller,1)*exp(-theta_p*s)
figure(5)
hold on;
grid minor;
plot(t,data,'LineWidth',2);
xlabel ('Time (s)'), ylabel('Current Pressure (Pa)'), title(strcat('Current Pressure vs Time (Kp = ',num2str(Kc,3),', Ti = ', num2str(Ti,3),')'));
ylim([91000 93500]);
xlim([0 60])
opt = stepDataOptions('InputOffset',(y0-90),'StepAmplitude',1.72e3/(dcgain(composed_system)));
[step_y,step_t] = step(composed_system,60,opt);
step_t = step_t + 5.177; % to substract the waiting time before controller starts
plot(step_t,step_y,'LineWidth',2)
legend('Experimental Result','Simulation');
hold off;

%% PID controller 
Tc=max(0.1*Tp,0.8*theta_p)
Kc = (Tp+0.5*theta_p)/(Kp*(Tc+0.5*theta_p))
Ti=Tp + 0.5*theta_p
Td = Tp*theta_p/(2*Tp+theta_p)
Ki=Kc/Ti
Kd = Td*Kc
s = tf('s');
Controller = Kc*(1+1/(Ti*s)+Td*s);
G_plant = Kp/(Tp*s + 1);
composed_system = feedback(G_plant*Controller,1)*exp(-theta_p*s)

figure(6)
hold on;
grid minor;
plot(t,data,'LineWidth',2);
xlabel ('Time (s)'), ylabel('Current Pressure (Pa)'), title(strcat('Current Pressure vs Time (Kp = ',num2str(Kc,3),', Ti = ', num2str(Ti,3),', Td = ',num2str(Td,3),')'));
ylim([91000 93500]);
xlim([0 60])
opt = stepDataOptions('InputOffset',(y0-90),'StepAmplitude',1.72e3/(dcgain(composed_system)));
[step_y,step_t] = step(composed_system,60,opt);
step_t = step_t + 2.36; % to substract the waiting time before controller starts
plot(step_t,step_y,'LineWidth',2)
legend('Experimental Result','Simulation');
hold off;

