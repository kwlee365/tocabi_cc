clc
clear all
close all

% dataWBC = readmatrix('dataCC5.txt');
% trigger = dataWBC(:, 1);
dataWBC = readmatrix('dataCC6.txt');
torque_sol = dataWBC(:, 1:33);
figure()
sgtitle('torque')
for cnt = 1:1:6
% cnt = 10 
    plot(torque_sol(:,cnt))    
    hold on
    legend()
end
plot(trigger(:,1))

%%
clc
clear all
close all

dataCC = readmatrix('dataCC1.txt');
com_pos = dataCC(:, 1:6);
dataCC = readmatrix('dataCC2.txt');
support_foot_pos = dataCC(:, 1:6);
dataCC = readmatrix('dataCC3.txt');
swing_foot_pos= dataCC(:, 1:6);

figure()

w = sqrt(9.81 / 0.73)

start_cnt = 0;
for cnt = 1:1:3
    subplot(3,1,cnt)
    plot(com_pos(:,start_cnt + cnt));
    hold on
    plot(com_pos(:,start_cnt + cnt + 3));
    
    plot(support_foot_pos(:,start_cnt + cnt));
    plot(support_foot_pos(:,start_cnt + cnt + 3));
    plot(swing_foot_pos(:, start_cnt + cnt))
    plot(swing_foot_pos(:, start_cnt + cnt + 3))
    legend()
    % legend('com', 'support foot', 'swing foot');
end
%%
clc
clear all
close all

dataCC = readmatrix('dataWBC6.txt');
dt = dataCC(:, 1);

figure()
plot(dt(:,1))