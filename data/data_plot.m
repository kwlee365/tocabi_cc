clc
clear all
close all

dataWBC = readmatrix('dataWBC1.txt');
qddot_des = dataWBC(:, 1:39);
dataWBC = readmatrix('dataWBC2.txt');
qddot_act = dataWBC(:, 1:39);
dataWBC = readmatrix('dataWBC3.txt');
torque = dataWBC(:, 1:33);
dataWBC = readmatrix('dataWBC4.txt');
wrench = dataWBC(:, 1:12);
% TORQUE

figure()
start_cnt = 0;
for cnt = 1:1:6
    subplot(6,1,cnt)
    plot(qddot_des(:,start_cnt + cnt + 6));
    hold on
    plot(qddot_act(:,start_cnt + cnt + 6));
    hold on
    plot(torque(:,start_cnt + cnt));
    legend('qddot des', 'qddot act', 'torque')
end

figure()
start_cnt = 0;
for cnt = 1:1:12
    plot(wrench(:,start_cnt + cnt));
    hold on
    legend()
end

%
clc
clear all
% close all

dataCC = readmatrix('dataCC1.txt');
v_ = dataCC(:, 1:6);
dataCC = readmatrix('dataCC2.txt');
w_ = dataCC(:, 1:6);


figure()
start_cnt = 0;
for cnt = 1:1:3
    subplot(3,1,cnt)
    plot(v_(:,start_cnt + cnt));
    hold on
    plot(v_(:,start_cnt + cnt + 3 ));

    legend('v des', 'v mea')
end

figure()
start_cnt = 0;
for cnt = 1:1:3
    subplot(3,1,cnt)
    plot(w_(:,start_cnt + cnt));
    hold on
    plot(w_(:,start_cnt + cnt + 3 ));

    legend('v des', 'v mea')
end
