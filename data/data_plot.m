clc
clear all
close all

data1 = readmatrix('data1.txt')
figure(1)
plot(data1(:,1))
hold on
grid on
% plot(data1(:,2))
% plot(data1(:,3))
plot(data1(:,4))
plot(data1(:,5))
legend()

data2 = readmatrix('data2.txt')
figure(2)
plot(data2(:,1))
hold on
grid on
% plot(data2(:,2))
% plot(data2(:,3))
plot(data2(:,4))
plot(data2(:,5))
legend()

figure(3)
plot(data2(:,6))
hold on
grid on
plot(data2(:,7))
plot(data2(:,8))
plot(data2(:,9))

data3 = readmatrix('data3.txt')
figure()
plot(data3(:,1))
hold on
grid on
plot(data3(:,2))
legend()

figure()
plot(data3(:,3))
hold on
grid on
plot(data3(:,4))
plot(data3(:,5))
plot(data3(:,6))
legend()
%%
clc
clear all
close all
data5 = readmatrix('data5.txt');

figure()
idx = 1
plot(data5(:, idx))
hold on
grid on
plot(data5(:, idx+6))
legend()

figure()
idx = 2
plot(data5(:, idx))
hold on
grid on
plot(data5(:, idx+6))
legend()

figure()
idx = 3
plot(data5(:, idx))
hold on
grid on
plot(data5(:, idx+6))
legend()

