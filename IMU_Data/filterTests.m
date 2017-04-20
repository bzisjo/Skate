%% For testing filtering of data

%% Loading 10ms delay acceleration test data
accelX_10ms = importdata('accelX_10ms.txt');
accelY_10ms = importdata('accelY_10ms.txt');
accelZ_10ms = importdata('accelZ_10ms.txt');
% apply offsets to accel test data. gyro has offsets included aready
accelX_10ms(:,1) = accelX_10ms(:,1) - .0206;
accelX_10ms(:,2) = accelX_10ms(:,2) - .0216;
accelX_10ms(:,3) = accelX_10ms(:,3) - .9130;
accelX_10ms(:,4) = accelX_10ms(:,4) - 2.5316;
accelX_10ms(:,5) = accelX_10ms(:,5) - 9.7539;
accelX_10ms(:,6) = accelX_10ms(:,6) - 12.3735;

accelY_10ms(:,1) = accelY_10ms(:,1) - .0206;
accelY_10ms(:,2) = accelY_10ms(:,2) - .0216;
accelY_10ms(:,3) = accelY_10ms(:,3) - .9130;
accelY_10ms(:,4) = accelY_10ms(:,4) - 2.5316;
accelY_10ms(:,5) = accelY_10ms(:,5) - 9.7539;
accelY_10ms(:,6) = accelY_10ms(:,6) - 12.3735;

accelZ_10ms(:,1) = accelZ_10ms(:,1) - .0206;
accelZ_10ms(:,2) = accelZ_10ms(:,2) - .0216;
accelZ_10ms(:,3) = accelZ_10ms(:,3) - .9130;
accelZ_10ms(:,4) = accelZ_10ms(:,4) - 2.5316;
accelZ_10ms(:,5) = accelZ_10ms(:,5) - 9.7539;
accelZ_10ms(:,6) = accelZ_10ms(:,6) - 12.3735;

% offsets already applied during collection
gyroX_10ms = importdata('gyroX_10ms.txt');
gyroY_10ms = importdata('gyroY_10ms.txt');
gyroZ_10ms = importdata('gyroZ_10ms.txt');

%% Plotting raw data as is
%% Translational Data test (10ms delay)
figure(1)
subplot(2,1,1)
plot(accelX_10ms(:,1:3));
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of X axis Translational Test (10ms)')
[m, ~] = size(accelX_10ms(:,1));
axis([0 m -2 2]);
ylabel('gravity (g)')

subplot(2,1,2)
plot(accelX_10ms(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of X axis Translational Test (10ms)')
[m, ~] = size(accelX_10ms(:,1));
axis([0 m -1000 1000]);
ylabel('degrees/sec')

figure(2)
subplot(2,1,1)
plot(accelY_10ms(:,1:3));
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of Y axis Translational Test (10ms)')
[m, ~] = size(accelY_10ms(:,1));
axis([0 m -2 2])
ylabel('gravity (g)')

subplot(2,1,2)
plot(accelY_10ms(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of Y axis Translational Test (10ms)')
[m, ~] = size(accelY_10ms(:,1));
axis([0 m -1000 1000]);
ylabel('degrees/sec')

figure(3)
subplot(2,1,1)
plot(accelZ_10ms(:,1:3));
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of Z axis Translational Test (10ms)')
[m, ~] = size(accelZ_10ms(:,1));
axis([0 m -2 2])
ylabel('gravity (g)')

subplot(2,1,2)
plot(accelZ_10ms(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of Z axis Translational Test (10ms)')
[m, ~] = size(accelZ_10ms(:,1));
axis([0 m -1000 1000]);
ylabel('degrees/sec')

%% Rotational test (10 ms) Note this data set already has mean offsets applied
figure(4)
subplot(2,1,1)
plot(gyroX_10ms(:,1:3));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Translational Data of X axis Rotational Test (10ms)')
[m, ~] = size(gyroX_10ms(:,1));
axis([0 m -2 2]);
ylabel('gravity (g)')

subplot(2,1,2)
plot(gyroX_10ms(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of X axis Rotational Test (10ms)')
[m, n] = size(gyroX_10ms(:,1));
axis([0 m -2500 2500]);
ylabel('degrees/sec')

figure(5)
subplot(2,1,1)
plot(gyroY_10ms(:,1:3));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Translational Data of Y axis Rotational Test (10ms)')
[m, ~] = size(gyroY_10ms(:,1));
axis([0 m -2 2])
ylabel('gravity (g)')

subplot(2,1,2)
plot(gyroY_10ms(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of Y axis Rotational Test (10ms)')
[m, ~] = size(gyroY_10ms(:,1));
axis([0 m -2500 2500]);
ylabel('degrees/sec')

figure(6)
subplot(2,1,1)
plot(gyroZ_10ms(:,1:3));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Translational Data of Z axis Rotational Test (10ms)')
[m, ~] = size(gyroZ_10ms(:,1));
axis([0 m -2 2])
ylabel('gravity (g)')

subplot(2,1,2)
plot(gyroZ_10ms(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of Z axis Rotational Test (10ms)')
[m, ~] = size(gyroZ_10ms(:,1));
axis([0 m -2500 2500]);
ylabel('degrees/sec')

%% Plotting integration based on given matlab function

figure(7)
angleX_test = cumtrapz(gyroX_10ms(:,4));
plot(angleX_test);
title('Rotation around X-axis')
ylabel('degrees')

figure(8)
angleY_test = cumtrapz(gyroY_10ms(:,5));
plot(angleY_test);
title('Rotation around Y-axis')
ylabel('degrees')

figure(9)
angleZ_test = cumtrapz(gyroZ_10ms(:,6));
plot(angleZ_test);
title('Rotation around Z-axis')
ylabel('degrees')

%% Realized that plots had hardcoded calibration in library not
%   set to our IMU

%% obtaining calibration offsets

offsetData = importdata('calibrationRetest.txt');
accelX_offsets = offsetData(:,1);
accelY_offsets = offsetData(:,2);
accelZ_offsets = offsetData(:,3);
gyroX_offsets = offsetData(:,4);
gyroY_offsets = offsetData(:,5);
gyroZ_offsets = offsetData(:,6);

accelX_mean = mean(accelX_offsets)
accelX_std = std(accelX_offsets);
accelX_med = median(accelX_offsets)

accelY_mean = mean(accelY_offsets)
accelY_std = std(accelY_offsets);
accelY_med = median(accelY_offsets)

accelZ_mean = mean(accelZ_offsets)
accelZ_std = std(accelZ_offsets);
accelZ_med = median(accelZ_offsets)

gyroX_mean = mean(gyroX_offsets)
gyroX_median = median(gyroX_offsets)
gyroX_std = std(gyroX_offsets);

gyroY_mean = mean(gyroY_offsets)
gyroY_median = median(gyroY_offsets)
gyroY_std = std(gyroY_offsets);

gyroZ_mean = mean(gyroZ_offsets)
gyroZ_median = median(gyroZ_offsets)
gyroZ_std = std(gyroZ_offsets);

%% Graphs for accelerometer and gyroscope still offsets
figure(10)
subplot(2,1,1)
plot(accelX_offsets)
hold on
meanLine = refline(0, accelX_mean);
meanLine.Color = 'r';
accelX_low_refline = refline(0, accelX_mean - accelX_std);
set(accelX_low_refline, 'LineStyle', ':');
accelX_high_refline = refline(0, accelX_mean + accelX_std);
set(accelX_high_refline, 'LineStyle', ':');
title('X Accelerometer Still Data (run with 10ms delay)')
ylabel('g')
xlabel('samples')
hold off

subplot(2,1,2)
plot(gyroX_offsets)
hold on
meanLine = refline(0, gyroX_mean);
meanLine.Color = 'r';
gyroX_low_refline = refline(0, gyroX_mean - gyroX_std);
set(gyroX_low_refline, 'LineStyle', ':');
gyroX_high_refline = refline(0, gyroX_mean + gyroX_std);
set(gyroX_high_refline, 'LineStyle', ':');
title('X Gyroscope Still Data (run with 10ms delay)')
ylabel('deg/sec')
xlabel('samples')
hold off

figure(11)
subplot(2,1,1)
plot(accelY_offsets)
hold on
meanLine = refline(0, accelY_mean);
meanLine.Color = 'r';
accelY_low_refline = refline(0, accelY_mean - accelY_std);
set(accelY_low_refline, 'LineStyle', ':');
accelY_high_refline = refline(0, accelY_mean + accelY_std);
set(accelY_high_refline, 'LineStyle', ':');
title('Y Accelerometer Still Data (run with 10ms delay)')
ylabel('g')
xlabel('samples')
hold off

subplot(2,1,2)
plot(gyroY_offsets)
hold on
meanLine = refline(0, gyroY_mean)
meanLine.Color = 'r';
gyroY_low_refline = refline(0, gyroY_mean - gyroY_std);
set(gyroY_low_refline, 'LineStyle', ':');
gyroY_high_refline = refline(0, gyroY_mean + gyroY_std);
set(gyroY_high_refline, 'LineStyle', ':');
title('Y Gyroscope Still Data (run with 10ms delay)')
ylabel('deg/sec')
xlabel('samples')
hold off

figure(12)
subplot(2,1,1)
plot(accelZ_offsets)
hold on
meanLine = refline(0, accelZ_mean);
meanLine.Color = 'r';
accelZ_low_refline = refline(0, accelZ_mean - accelZ_std);
set(accelZ_low_refline, 'LineStyle', ':');
accelZ_high_refline = refline(0, accelZ_mean + accelZ_std);
set(accelZ_high_refline, 'LineStyle', ':');
title('Z Accelerometer Still Data (run with 10ms delay)')
ylabel('g')
xlabel('samples')
hold off

subplot(2,1,2)
plot(gyroZ_offsets)
hold on
meanLine = refline(0, gyroZ_mean);
meanLine.Color = 'r';
gyroZ_low_refline = refline(0, gyroZ_mean - gyroZ_std);
set(gyroZ_low_refline, 'LineStyle', ':');
gyroZ_high_refline = refline(0, gyroZ_mean + gyroZ_std);
set(gyroZ_high_refline, 'LineStyle', ':');
title('Z Gyroscope Still Data (run with 10ms delay)')
ylabel('deg/sec')
xlabel('samples')
hold off

%% Plotting test of mahony filter

mahonyStill = importdata('re_mahonyFilterStill.txt');
figure(13)
plot(mahonyStill)

%% Plotting Test raw data





