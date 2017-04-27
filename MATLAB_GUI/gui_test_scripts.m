%% test scripts and functions for the gui
% LOAD
test_data = importdata('/home/chu49/ECE445/Skate/Test data/masterdata.txt');

%% Unpack
session1_ind = find(test_data(:,1) == 0);
session1_data = test_data(session1_ind(1):session1_ind(length(session1_ind)),:);

session2_ind = find(test_data(:,1) == 1);
session2_data = test_data(session2_ind(1):session2_ind(length(session2_ind)),:);

session3_ind = find(test_data(:,1) == 2);
session3_data = test_data(session3_ind(1):session3_ind(length(session3_ind)),:);

%% Testing the FSR data for session 1
FSR1 = convertFSR(session1_data(:,2));
FSR2 = convertFSR(session1_data(:,3));

figure(1)
plot(FSR1)
title('Force from Toe FSR')
ylabel('lbs')

figure(2)
plot(FSR2)
title('Force from Heel FSR')
ylabel('lbs')

% low pass filter 
alpha = .8;
[m, n] = size(FSR2);
lpf_FSR2_test = zeros(m, n);
(1-alpha)*FSR2(i)

for i = 1:m
    if i == 1
        lpf_FSR2_test(i) = (1-alpha)*FSR2(i);
    else
        lpf_FSR2_test(i) = (1-alpha)*FSR2(i) + alpha * lpf_FSR2_test(i-1);
    end
end

figure(3)
plot(lpf_FSR2_test)
title('Force from Heel FSR w/ Low Pass Filter')

figure(4)
plot(FSR1)
hold on
plot(FSR2)
hold off
legend('show')
legend('Toe', 'Heel')


%% Testing the FSR data for session 2
FSR1 = convertFSR(session2_data(:,2));
FSR2 = convertFSR(session2_data(:,3));

figure(1)
plot(FSR1)
title('Force from Toe FSR')
ylabel('lbs')

figure(2)
plot(FSR2)
title('Force from Heel FSR')
ylabel('lbs')

%% Testing imu data for session 1
imu_data_s1 = session1_data(:,4:9);

% acceleration
figure(5)
plot(imu_data_s1(:,1))
title('X axis accerleration')
figure(6)
plot(imu_data_s1(:,2))
title('Y axis accerleration')
figure(7)
plot(imu_data_s1(:,3))
title('Z axis accerleration')

% angle (integrate)     NOTE: y axis is upside down
angles_s1 = [rk_integrator(imu_data_s1(:,4)/60); rk_integrator(-imu_data_s1(:,5)/60); rk_integrator(imu_data_s1(:,6)/60)];
angles_s1 = (angles_s1).';

figure(8)
plot(angles_s1(:,1))
title('X axis angle')
figure(9)
plot(angles_s1(:,2))
title('Y axis angle')
figure(10)
plot(angles_s1(:,3))
title('Z axis angle')

% calculate angles from acceleration
angle_accelX_s1= (atan(imu_data_s1(:,2) ./ sqrt(imu_data_s1(:,1).^2 + imu_data_s1(:,3).^2)))*180/pi;
angle_accelY_s1 = (- atan(imu_data_s1(:,1) ./ sqrt(imu_data_s1(:,2).^2 + imu_data_s1(:,3).^2)))*180/pi;

figure(11)
plot(angle_accelX_s1)
title('X axis angles from acceleration')
figure(12)
plot(angle_accelY_s1)
title('Y axis angles from acceleration')

%% View fft of accleration and gyroscope data of session 1

xaccel_fft = fft(imu_data_s1(:,1));
yaccel_fft = fft(imu_data_s1(:,2));
zaccel_fft = fft(imu_data_s1(:,3));

figure(13)
subplot(3,1,1)
plot(xaccel_fft)
subplot(3,1,2)
plot(yaccel_fft)
subplot(3,1,3)
plot(zaccel_fft)



