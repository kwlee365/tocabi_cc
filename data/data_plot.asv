clc
clear all
close all

dataWBC = readmatrix('dataCC5.txt');
trigger = dataWBC(:, 1);
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
dcm_pos = dataCC(:, 1:6);
dataCC = readmatrix('dataCC3.txt');
zmp_pos = dataCC(:, 1:6);
dataCC = readmatrix('dataCC4.txt');
foot_pos = dataCC(:, 1:6);
lfoot_pos = foot_pos(:, 1:3);
rfoot_pos = foot_pos(:, 4:6);

% figure()
% start_cnt = 0;
% for cnt = 1:1:3
%     subplot(3,1,cnt)
%     plot(dcm_pos(:, start_cnt + cnt));
%     hold on
%     plot(dcm_pos(:, start_cnt + cnt + 3));
%     plot(zmp_pos(:, start_cnt + cnt))
%     plot(zmp_pos(:, start_cnt + cnt + 3))
%     plot(lfoot_pos(:, cnt));
%     plot(rfoot_pos(:, cnt));
% 
%     legend('dcm des', 'dcm mea', 'zmp ref', 'zmp des', 'lfoot', 'rfoot')
% end

figure()
start_cnt = 0;
for cnt = 1:1:3
    subplot(3,1,cnt)
    plot(com_pos(:,start_cnt + cnt));
    hold on
    plot(com_pos(:,start_cnt + cnt + 3 ));
    plot(zmp_pos(:, start_cnt + cnt))
    plot(zmp_pos(:, start_cnt + cnt + 3))

    legend('com des', 'com mea', 'zmp ref', 'zmp des')
end
%%
clc
clear all
close all

dataCC = readmatrix('dataWBC6.txt');
dt = dataCC(:, 1);

figure()
plot(dt(:,1))