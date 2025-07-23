clc
clear all
close all

% THREAD1 DATA
dataWBC = readmatrix('dataWBC.txt');

torque = dataWBC(:, 1:33);
wrench = dataWBC(:, 34:45);

%% TORQUE
figure()
for cnt = 1:1:6
    subplot(6,1,cnt)
    plot(torque(:,cnt));
    hold on
    grid on
end
figure()
for cnt = 1:1:6
    subplot(6,1,cnt)
    plot(torque(:,cnt+6));
end
figure()
for cnt = 1:1:3
    subplot(3,1,cnt)
    plot(torque(:,cnt+12));
end
figure()
for cnt = 1:1:8
    subplot(8,1,cnt)
    plot(torque(:,cnt+15));
end

figure()
for cnt = 1:1:2
    subplot(2,1,cnt)
    plot(torque(:,cnt+23));
end
figure()
for cnt = 1:1:8
    subplot(8,1,cnt)
    plot(torque(:,cnt+25));
end
% %% CONTACT WRENCH
% figure()
% for cnt = 1:1:6
%     subplot(6,1,cnt)
%     plot(wrench(:,cnt));
%     hold on
%     grid on
% end
% figure()
% for cnt = 1:1:6
%     subplot(6,1,cnt)
%     plot(wrench(:,cnt+6));
% end