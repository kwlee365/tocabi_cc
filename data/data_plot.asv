clc
clear all
close all

% THREAD1 DATA
dataCOM = readmatrix('dataCOM.txt');
dataZMP = readmatrix('dataZMP.txt');
dataWrench = readmatrix('dataWrench.txt');
dataThread3Time = readmatrix('dataThread3Time.txt');

figure()
subplot(3,1,1)
plot(dataCOM(:,1));
hold on
grid on
plot(dataCOM(:,4));
plot(dataZMP(:,1));
% plot(dataZMP(:,3));
legend('com ref', 'com', 'zmp ref')
subplot(3,1,2)
plot(dataCOM(:,2));
hold on
grid on
plot(dataCOM(:,5));
plot(dataZMP(:,2));
% plot(dataZMP(:,4));
legend('com ref', 'com', 'zmp ref')
subplot(3,1,3)
plot(dataCOM(:,3));
hold on
grid on
plot(dataCOM(:,6));
legend('com ref', 'com')

figure()
for cnt = 1:1:6
    subplot(6,1,cnt)
    plot(dataWrench(:,cnt));
    hold on
    grid on
    plot(dataWrench(:,cnt+6));
    legend('LIPM', 'SRBD')
end
figure()
for cnt = 1:1:6
    subplot(6,1,cnt)
    plot(dataWrench(:,cnt+12));
    hold on
    grid on
    plot(dataWrench(:,cnt+18));
    legend('LIPM', 'SRBD')
end

% MPC REFERENCE DATA
data1 = readmatrix('data1.txt');
data2 = readmatrix('data2.txt');
data3 = readmatrix('data3.txt');
data4 = readmatrix('data4.txt');
data5 = readmatrix('data5.txt');
data6 = readmatrix('data6.txt');
data7 = readmatrix('data7.txt');
data8 = readmatrix('data8.txt');
data9 = readmatrix('data9.txt');
data10 = readmatrix('data10.txt');
data11 = readmatrix('data11.txt');
data12 = readmatrix('data12.txt');
data13 = readmatrix('data13.txt');
data14 = readmatrix('data14.txt');
data15 = readmatrix('data15.txt');
data16 = readmatrix('data16.txt');
data17 = readmatrix('data17.txt');
data18 = readmatrix('data18.txt');
data19 = readmatrix('data19.txt');
data20 = readmatrix('data20.txt');
data21 = readmatrix('data21.txt');
data22 = readmatrix('data22.txt');
data23 = readmatrix('data23.txt');
data24 = readmatrix('data24.txt');
data25 = readmatrix('data25.txt');
data26 = readmatrix('data26.txt');
data27 = readmatrix('data27.txt');
data28 = readmatrix('data28.txt');
data29 = readmatrix('data29.txt');
data30 = readmatrix('data30.txt');
data31 = readmatrix('data31.txt');
data32 = readmatrix('data32.txt');
data33 = readmatrix('data33.txt');
data34 = readmatrix('data34.txt');
data35 = readmatrix('data35.txt');
data36 = readmatrix('data36.txt');
data37 = readmatrix('data37.txt');
data38 = readmatrix('data38.txt');
data39 = readmatrix('data39.txt');
data40 = readmatrix('data40.txt');
data41 = readmatrix('data41.txt');
data42 = readmatrix('data42.txt');
data43 = readmatrix('data43.txt');
data44 = readmatrix('data44.txt');
data45 = readmatrix('data45.txt');
data46 = readmatrix('data46.txt');
data47 = readmatrix('data47.txt');
data48 = readmatrix('data48.txt');

mpc_tick = 151

% state
figure()
subplot(4,1,1)
plot(data1(mpc_tick,:));
hold on
grid on
plot(data3(mpc_tick,:));
plot(data22(mpc_tick,:));
legend('zmp ref', 'com ref', 'com pred')
subplot(4,1,2)
plot(data2(mpc_tick,:));
hold on
grid on
plot(data4(mpc_tick,:));
plot(data23(mpc_tick,:));
legend('zmp ref', 'com ref', 'com pred')
subplot(4,1,3)
plot(data5(mpc_tick,:));
hold on
grid on
plot(data24(mpc_tick,:));
legend('com ref', 'com pred')
subplot(4,1,4)
plot(data11(mpc_tick,:));
hold on
grid on
plot(data12(mpc_tick,:));
legend('eta l ref', 'eta r ref')

%%--- time
figure()
plot(1e-6 * dataThread3Time(:));
hold on 
grid on
plot((1/50) * ones(length(dataThread3Time(:)),1))
legend('dT mpc', 'dT max')

%%--- com dot
figure()
subplot(3,1,1)
plot(data6(mpc_tick,:));
hold on
grid on
plot(data28(mpc_tick,:));
legend('com dot ref', 'com dot pred')
title('COM DOT')
subplot(3,1,2)
plot(data7(mpc_tick,:));
hold on
grid on
plot(data29(mpc_tick,:));
legend('com dot ref', 'com dot pred')
subplot(3,1,3)
plot(data8(mpc_tick,:));
hold on
grid on
plot(data30(mpc_tick,:));
legend('com dot ref', 'com dot pred')

%%--- angular
figure()
subplot(3,2,1)
plot(data19(mpc_tick,:));
legend('theta x')
subplot(3,2,3)
plot(data20(mpc_tick,:));
legend('theta y')
subplot(3,2,5)
plot(data21(mpc_tick,:));
legend('theta z')

subplot(3,2,2)
plot(data25(mpc_tick,:));
legend('w x')
subplot(3,2,4)
plot(data26(mpc_tick,:));
legend('w y')
subplot(3,2,6)
plot(data27(mpc_tick,:));
legend('w z')

%%--- fL
figure()
subplot(3,2,1)
plot(data31(mpc_tick,:));
legend('mL x')
subplot(3,2,3)
plot(data32(mpc_tick,:));
legend('mL y')
subplot(3,2,5)
plot(data33(mpc_tick,:));
legend('mL z')
subplot(3,2,2)
plot(data34(mpc_tick,:));
legend('fL x')
subplot(3,2,4)
plot(data35(mpc_tick,:));
legend('fL y')
subplot(3,2,6)
plot(data36(mpc_tick,:));
hold on
grid on
plot(data17(mpc_tick,:));
legend('fL z', 'fL z ref')


% fR
figure()
subplot(3,2,1)
plot(data37(mpc_tick,:));
legend('mR x')
subplot(3,2,3)
plot(data38(mpc_tick,:));
legend('mR y')
subplot(3,2,5)
plot(data39(mpc_tick,:));
legend('mR z')
subplot(3,2,2)
plot(data40(mpc_tick,:));
legend('fR x')
subplot(3,2,4)
plot(data41(mpc_tick,:));
legend('fR y')
subplot(3,2,6)
plot(data42(mpc_tick,:));
hold on
grid on
plot(data18(mpc_tick,:));
legend('fR z', 'fR z ref')

%--- posvector com to foot
figure()
subplot(3,2,1)
plot(data43(mpc_tick,:));
hold on
grid on
plot(data13(mpc_tick,:));
legend('rL x', 'contact point L x')
subplot(3,2,3)
plot(data44(mpc_tick,:));
hold on
grid on
plot(data14(mpc_tick,:));
legend('rL y', 'contact point L y')
subplot(3,2,5)
plot(data45(mpc_tick,:));
legend('rL z')
subplot(3,2,2)
plot(data46(mpc_tick,:));
hold on
grid on
plot(data15(mpc_tick,:));
legend('rR x', 'contact point Rx')
subplot(3,2,4)
plot(data47(mpc_tick,:));
hold on
grid on
plot(data16(mpc_tick,:));
legend('rR y', 'contact point Ry')
subplot(3,2,6)
plot(data48(mpc_tick,:));
legend('rR z')