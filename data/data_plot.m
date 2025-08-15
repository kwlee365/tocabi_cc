clc
clear all
close all

dataWBC = readmatrix('dataWBC1.txt');
torque_qp = dataWBC(:, 1:33);
dataCC = readmatrix('dataCC6.txt');
torque = dataWBC(:, 1:33);



figure()
title('leg torque')
start_cnt = 0;
for cnt = 1:1:6
    plot(torque_qp(:,start_cnt + cnt));
    hold on
    plot(torque(:,start_cnt + cnt));
    legend()
end 

figure()
title('arm torque')
start_cnt = 15;
for cnt = 1:1:7
    plot(torque_qp(:,start_cnt + cnt));
    hold on
    plot(torque(:,start_cnt + cnt));

    legend()

end 
    plot(torque_qp(:,start_cnt + 8),'k');
    plot(torque(:,start_cnt + 8));



%%
clc
clear all
close all

dataCC = readmatrix('dataCC1.txt');
base_pos = dataCC(:, 1:6);
dataCC = readmatrix('dataCC2.txt');
lhand_pos = dataCC(:, 1:6);
dataCC = readmatrix('dataCC3.txt');
rfoot_pos = dataCC(:, 1:6);
dataCC = readmatrix('dataCC4.txt');
lhand_rot = dataCC(:, 1:6);
% dataCC = readmatrix('dataCC5.txt');
% base_rot = dataCC(:, 1:6);


figure()
start_cnt = 0;
for cnt = 1:1:3
    subplot(3,1,cnt)
    plot(rfoot_pos(:,start_cnt + cnt));
    hold on
    plot(rfoot_pos(:,start_cnt + cnt + 3 ));

    legend('des', 'mea')
end

%%
clc
clear all
close all

dataCC = readmatrix('dataWBC6.txt');
dt = dataCC(:, 1);

figure()
plot(dt(:,1))