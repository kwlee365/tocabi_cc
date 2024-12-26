clc
clear all
close all

data1 = readmatrix('data1.txt')
figure(1)
plot(data1(:,1))
hold on
grid on
plot(data1(:,2))
plot(data1(:,3))
% plot(data1(:,4))
% plot(data1(:,5))
legend()
ylim([-1 1])

data2 = readmatrix('data2.txt')
figure(2)
plot(data2(:,1))
hold on
grid on
plot(data2(:,2))
plot(data2(:,3))
% plot(data2(:,4))
% plot(data2(:,5))
legend()
ylim([-1 1])

%

dataMPC_zmp_x = readmatrix('dataMPC_zmp_x.txt')
dataMPC_zmp_y = readmatrix('dataMPC_zmp_y.txt')
dataMPC_zmp_pred_x = readmatrix('dataMPC_zmp_pred_x.txt')
dataMPC_zmp_pred_y = readmatrix('dataMPC_zmp_pred_y.txt')
dataMPC_com_x = readmatrix('dataMPC_com_x.txt')
dataMPC_com_y = readmatrix('dataMPC_com_y.txt')

mpc_tick = 300

figure()
plot(dataMPC_zmp_x(mpc_tick,:))
hold on
grid on
plot(dataMPC_zmp_pred_x(mpc_tick,:))
plot(dataMPC_com_x(mpc_tick,:))
plot(dataMPC_zmp_x(mpc_tick,:) + 0.17, 'k--')
plot(dataMPC_zmp_x(mpc_tick,:) - 0.13, 'k--')
legend()

figure()
plot(dataMPC_zmp_y(mpc_tick,:))
hold on
grid on
plot(dataMPC_zmp_pred_y(mpc_tick,:))
plot(dataMPC_com_y(mpc_tick,:))
plot(dataMPC_zmp_y(mpc_tick,:) + 0.1, 'k--')
plot(dataMPC_zmp_y(mpc_tick,:) - 0.1, 'k--')
legend()

dataMPC_time = readmatrix('dataMPC_time.txt')

figure()
plot(double(1000.0 ./ dataMPC_time(:,1)))