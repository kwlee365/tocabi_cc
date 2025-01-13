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
plot(data1(:,10))
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
plot(data2(:,10))
legend()
ylim([-1 1])
 
%%

data4 = readmatrix('data4.txt');
data5 = readmatrix('data5.txt');
data6 = readmatrix('data6.txt');
data7 = readmatrix('data7.txt');
data8 = readmatrix('data8.txt');
data9 = readmatrix('data9.txt');

%
close all
mpc_tick = 120
figure()
subplot(2,1,1)
plot(data4(mpc_tick,:));
hold on
grid on
plot(data6(mpc_tick,:));
plot(data8(mpc_tick,:));
legend()
subplot(2,1,2)
plot(data5(mpc_tick,:))
hold on
grid on
plot(data7(mpc_tick,:));
plot(data9(mpc_tick,:));
legend()