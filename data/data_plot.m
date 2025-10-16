clc
clear all
close all
%
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
for cnt = 1:1:2
    subplot(3,1,cnt)
    
    plot(left_foot_pos(:,start_cnt + cnt));
    hold on
    plot(left_foot_pos(:,start_cnt + cnt + 3));
    plot(right_foot_pos(:, start_cnt + cnt))
    plot(right_foot_pos(:, start_cnt + cnt + 3))
    plot(pelv_pos(:,start_cnt + cnt));
    plot(pelv_pos(:,start_cnt + cnt + 3));

    % ylim([-0.3, 0.3])

    legend('lfoot traj', 'lfoot cur', 'rfoot traj', 'rfoot cur', 'pelv traj', 'pelv cur');
end

%%
data = readmatrix('dataCC1.txt');
LF = data(:, 3);
RF = data(:, 9);


figure()
plot(LF(:,1))
hold on
plot(RF(:,1))

data = readmatrix('dataCC2.txt');
torque_sol = data(:, 1:33);

figure()
sgtitle('TORQUE LEG')
for cnt = 1:1:12
    plot(torque_sol(:,cnt))    
    hold on
end

figure()
sgtitle('TORQUE WAIST')
for cnt = 13:1:15
    plot(torque_sol(:,cnt))    
    hold on
end

figure()
sgtitle('TORQUE UPPER')
for cnt = 16:1:33
    plot(torque_sol(:,cnt))    
    hold on
end