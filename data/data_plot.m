clc
clear all
close all

dataWBC = readmatrix('dataCC1.txt');
torque_sol = dataWBC(:, 1:33);
figure()
sgtitle('torque')
for cnt = 1:1:6
% cnt = 10 
    plot(torque_sol(:,cnt))    
    hold on
    legend()
end

%%
clc
clear all
close all

data = readmatrix('dataWM1.txt');
left_foot_pos = data(:, 1:6);
data = readmatrix('dataWM2.txt');
right_foot_pos= data(:, 1:6);
data = readmatrix('dataWM3.txt');
pelv_pos = data(:, 1:6);
figure()

w = sqrt(9.81 / 0.73)

start_cnt = 0;
for cnt = 1:1:3
    subplot(3,1,cnt)
    
    plot(left_foot_pos(:,start_cnt + cnt));
    hold on
    plot(left_foot_pos(:,start_cnt + cnt + 3));
    plot(right_foot_pos(:, start_cnt + cnt))
    plot(right_foot_pos(:, start_cnt + cnt + 3))
    plot(pelv_pos(:,start_cnt + cnt));
    plot(pelv_pos(:,start_cnt + cnt + 3));
    legend()
    legend('lfoot traj', 'lfoot cur', 'rfoot traj', 'rfoot cur', 'pelv traj', 'pelv cur');
end
%%
clc
clear all
close all

dataCC = readmatrix('dataWBC6.txt');
dt = dataCC(:, 1);

figure()
plot(dt(:,1))