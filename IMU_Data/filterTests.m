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

%% Offsets for marked IMU

offsetDataMark = importdata('calibrationRetestMarked.txt');

accelX_offsetsMark = offsetDataMark(:,1);
accelY_offsetsMark = offsetDataMark(:,2);
accelZ_offsetsMark = offsetDataMark(:,3);
gyroX_offsetsMark = offsetDataMark(:,4);
gyroY_offsetsMark = offsetDataMark(:,5);
gyroZ_offsetsMark = offsetDataMark(:,6);

accelX_mean = mean(accelX_offsetsMark)
accelX_std = std(accelX_offsetsMark);
accelX_med = median(accelX_offsetsMark)

accelY_mean = mean(accelY_offsetsMark)
accelY_std = std(accelY_offsetsMark);
accelY_med = median(accelY_offsetsMark)

accelZ_mean = mean(accelZ_offsetsMark)
accelZ_std = std(accelZ_offsetsMark);
accelZ_med = median(accelZ_offsetsMark)

gyroX_mean = mean(gyroX_offsetsMark)
gyroX_median = median(gyroX_offsetsMark)
gyroX_std = std(gyroX_offsetsMark);

gyroY_mean = mean(gyroY_offsetsMark)
gyroY_median = median(gyroY_offsetsMark)
gyroY_std = std(gyroY_offsetsMark);

gyroZ_mean = mean(gyroZ_offsetsMark)
gyroZ_median = median(gyroZ_offsetsMark)
gyroZ_std = std(gyroZ_offsetsMark);

%% Graphs for marked accelerometer and gyroscope still offsets
figure(14)
subplot(2,1,1)
plot(accelX_offsetsMark)
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
plot(gyroX_offsetsMark)
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

figure(15)
subplot(2,1,1)
plot(accelY_offsetsMark)
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
plot(gyroY_offsetsMark)
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

figure(16)
subplot(2,1,1)
plot(accelZ_offsetsMark)
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
plot(gyroZ_offsetsMark)
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

mahonyX = importdata('mahony_Xrot.txt');
mahonyY = importdata('mahony_Yrot.txt');
mahonyZ = importdata('mahony_Zrot.txt');

figure(17)
plot(mahonyX)
title('90 degrees X Rotation Test')
legend('show')
legend('GyroX', 'GyroY', 'GyroZ');

figure(18)
plot(mahonyY)
title('90 degrees Y Rotation Test')
legend('show')
legend('GyroX', 'GyroY', 'GyroZ');

figure(19)
plot(mahonyZ)
title('90 degrees Z Rotation Test')
legend('show')
legend('GyroX', 'GyroY', 'GyroZ');

%% Loading retested raw data
accelX = importdata('re_accelX.txt');
accelY = importdata('re_accelY.txt');
accelZ = importdata('re_accelZ.txt');
gyroX = importdata('re_gyroX.txt');
gyroY = importdata('re_gyroY.txt');
gyroZ = importdata('re_gyroZ.txt');

%including offsets (median)
accelX_offset = 0.0222; %.0222
accelY_offset = 0.0371;
accelZ_offset = 0.9134-1;
gyroX_offset = 0.0084;
gyroY_offset = 1.2963;
gyroZ_offset = 1.2545;

offsets = [accelX_offset accelY_offset accelZ_offset gyroX_offset gyroY_offset gyroZ_offset];

for n = 1:6
    accelX(:,n) = accelX(:,n) - offsets(n);
    accelY(:,n) = accelY(:,n) - offsets(n);
    accelZ(:,n) = accelZ(:,n) - offsets(n);
    gyroX(:,n) = gyroX(:,n) - offsets(n);
    gyroY(:,n) = gyroY(:,n) - offsets(n);
    gyroZ(:,n) = gyroZ(:,n) - offsets(n);
end

%% Plotting Test raw data
%% Translational Data test (10ms delay)
figure(20)
subplot(2,1,1)
plot(accelX(:,1:3));
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of X axis Translational Test')
[m, ~] = size(accelX(:,1));
axis([0 m -2 2]);
ylabel('gravity (g)')

subplot(2,1,2)
plot(accelX(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of X axis Translational Test')
[m, ~] = size(accelX(:,1));
axis([0 m -300 300]);
ylabel('degrees/sec')

figure(21)
subplot(2,1,1)
plot(accelY(:,1:3));
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of Y axis Translational Test')
[m, ~] = size(accelY(:,1));
axis([0 m -2 2])
ylabel('gravity (g)')

subplot(2,1,2)
plot(accelY(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of Y axis Translational Test')
[m, ~] = size(accelY(:,1));
axis([0 m -300 300]);
ylabel('degrees/sec')

figure(22)
subplot(2,1,1)
plot(accelZ(:,1:3));
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of Z axis Translational Test')
[m, ~] = size(accelZ(:,1));
axis([0 m -2 2])
ylabel('gravity (g)')

subplot(2,1,2)
plot(accelZ(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of Z axis Translational Test')
[m, ~] = size(accelZ(:,1));
axis([0 m -300 300]);
ylabel('degrees/sec')

%% Rotational test (10 ms)
figure(23)
subplot(2,1,1)
plot(gyroX(:,1:3));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Translational Data of X axis Rotational Test')
[m, ~] = size(gyroX(:,1));
axis([0 m -2 2]);
ylabel('gravity (g)')

subplot(2,1,2)
plot(gyroX(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of X axis Rotational Test')
[m, n] = size(gyroX(:,1));
axis([0 m -300 300]);
ylabel('degrees/sec')

figure(24)
subplot(2,1,1)
plot(gyroY(:,1:3));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Translational Data of Y axis Rotational Test')
[m, ~] = size(gyroY(:,1));
axis([0 m -2 2])
ylabel('gravity (g)')

subplot(2,1,2)
plot(gyroY(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of Y axis Rotational Test')
[m, ~] = size(gyroY(:,1));
axis([0 m -300 300]);
ylabel('degrees/sec')

figure(25)
subplot(2,1,1)
plot(gyroZ(:,1:3));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Translational Data of Z axis Rotational Test')
[m, ~] = size(gyroZ(:,1));
axis([0 m -2 2])
ylabel('gravity (g)')

subplot(2,1,2)
plot(gyroZ(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of Z axis Rotational Test')
[m, ~] = size(gyroZ(:,1));
axis([0 m -300 300]);
ylabel('degrees/sec')

%% Checking difference between cumtrapz and kutta runge integrator
%% 
velocityX = cumtrapz(accelX(:,1));
figure(26)
plot(velocityX)
displacementX = cumtrapz(velocityX);
figure(27)
plot(displacementX)

%% Using kutta runge integrator

velocityX_rk = rk_integrator(accelX(:,1));
figure(28)
plot(velocityX_rk)
hold on
plot(velocityX)

figure(29)
temp = velocityX - velocityX_rk.';
plot(temp)
mean(temp)

% kutta runge integrator seems like same as cumtrapz function
% mean is .0028 so rk integrator seems to underestimate barely

%% Applying kr integrator to rotation tests
% scaling (when running at around 60 Hz)
angles_gyroX = [rk_integrator(gyroX(:,4)/60); rk_integrator(gyroX(:,5)/60); rk_integrator(gyroX(:,6)/60)];
angles_gyroX = (angles_gyroX).';
angles_gyroY = [rk_integrator(gyroY(:,4)/60); rk_integrator(gyroY(:,5)/60); rk_integrator(gyroY(:,6)/60)];
angles_gyroY = (angles_gyroY).';
angles_gyroZ = [rk_integrator(gyroZ(:,4)/60); rk_integrator(gyroZ(:,5)/60); rk_integrator(gyroZ(:,6)/60)];
angles_gyroZ = (angles_gyroZ).';

figure(36)
plot(angles_gyroX);
title('X Rotation Test: Gyroscope Data');
ylabel('degrees')
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');

figure(37)
plot(angles_gyroY);
title('Y Rotation Test: Gyroscope Data');
ylabel('degrees')
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');

figure(38)      %% looks kind of bad
plot(angles_gyroZ);
title('Z Rotation Test: Gyroscope Data');
ylabel('degrees')
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');


%% temp test of yaw
temp = importdata('temp.txt');
%including offsets (median)
accelX_offset = 0.0222; %.0222
accelY_offset = 0.0371;
accelZ_offset = 0.9134-1;
gyroX_offset = 0.0084;
gyroY_offset = 1.2963;
gyroZ_offset = 1.2545;

offsets = [accelX_offset accelY_offset accelZ_offset gyroX_offset gyroY_offset gyroZ_offset];

for n = 1:6
    temp(:,n) = temp(:,n) - offsets(n);
end

angles_gyroZ = [rk_integrator(temp(:,4)/60); rk_integrator(temp(:,5)/60); rk_integrator(temp(:,6)/60)];
angles_gyroZ = (angles_gyroZ).';

figure(38)      %% looks kind of bad
plot(angles_gyroZ);
title('Z Rotation Test');
ylabel('degrees')
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');

%% test integrating still data
tempOffsetData = offsetData;
%including offsets (median)
accelX_offset = 0.0222; %.0222
accelY_offset = 0.0371;
accelZ_offset = 0.9134;
gyroX_offset = 0.0084;
gyroY_offset = 1.2963;
gyroZ_offset = 1.2545;

offsets = [accelX_offset accelY_offset accelZ_offset gyroX_offset gyroY_offset gyroZ_offset];

for n = 1:6
    tempOffsetData(:,n) = tempOffsetData(:,n) - offsets(n);
end

angleTempOffset = [rk_integrator(tempOffsetData(:,4)/60); rk_integrator(tempOffsetData(:,5)/60); rk_integrator(tempOffsetData(:,6)/60)];
angleTempOffset = (angleTempOffset).';

figure()
plot(angleTempOffset)

%% Complimentary Filter
% x rotation test
tau = .65;
dt = 1/60; %.005;
alpha = .7;%tau / (tau + dt);

% low pass filter first
[m, n] = size(gyroX(:,1:3));
lpf_accel_Xtest = zeros(m, n);

for i = 1:m
    if i == 1
        lpf_accel_Xtest(i,1:3) = (1-alpha)*gyroX(i,1:3);
    else
        lpf_accel_Xtest(i,1:3) = (1-alpha)*gyroX(i,1:3) + alpha * lpf_accel_Xtest(i-1,1:3);
    end
end

% high pass filter


angle_accelX_Xtest = (atan(lpf_accel_Xtest(:,2) ./ sqrt(lpf_accel_Xtest(:,1).^2 + lpf_accel_Xtest(:,3).^2)))*180/pi;
angle_accelY_Xtest = (- atan(lpf_accel_Xtest(:,1) ./ sqrt(lpf_accel_Xtest(:,2).^2 + lpf_accel_Xtest(:,3).^2)))*180/pi;

angleX_filt = (1-alpha) * angles_gyroX(:,1) + alpha .* angle_accelX_Xtest;
angleY_filt = (1-alpha) * angles_gyroX(:,2) + alpha .* angle_accelY_Xtest;

figure(36)
plot(angle_accelX_Xtest)
figure(37)
plot(angle_accelY_Xtest)

figure(38)
plot(angleX_filt)
hold on
plot(angles_gyroX(:,1))
hold off
legend('show')
legend('filtered', 'gyroscope')
meanLine = refline(0, 83);
meanLine.Color = 'g';


figure(39)
plot(angleX_filt - angles_gyroX(:,1))


%%

angle_accelX_Xtest = (atan(gyroX(:,2) ./ sqrt(gyroX(:,1).^2 + gyroX(:,3).^2)))*180/pi;
angle_accelY_Xtest = (- atan(gyroX(:,1) ./ sqrt(gyroX(:,2).^2 + gyroX(:,3).^2)))*180/pi;

angleX_filt = (1-alpha) * angles_gyroX(:,1) + alpha .* angle_accelX_Xtest;
angleY_filt = (1-alpha) * angles_gyroX(:,2) + alpha .* angle_accelY_Xtest;

figure(40)
plot(angle_accelX_Xtest)
figure(41)
plot(angle_accelY_Xtest)

figure(42)
plot(angleX_filt)
hold on
plot(angles_gyroX(:,1))
hold off
title('X Axis Data for X Rotation Test with Complimentary Filtering') 
xlabel('samples')
ylabel('degrees')
legend('show')
legend('filtered', 'gyroscope')
meanLine = refline(0, 83);
meanLine.Color = 'g';

figure(43)
plot(angleX_filt - angles_gyroX(:,1))

%%
% y rotation test
angle_accelX_Ytest = (atan(gyroY(:,2) ./ sqrt(gyroY(:,1).^2 + gyroY(:,3).^2)))*180/pi;
angle_accelY_Ytest = (- atan(gyroY(:,1) ./ sqrt(gyroY(:,2).^2 + gyroY(:,3).^2)))*180/pi;

tau = 1.0;
dt = 1/60; %.005;
alpha = .35;%tau / (tau + dt);

angleX_filt_Ytest = alpha * angles_gyroY(:,1) + (1-alpha) .* angle_accelX_Ytest;
angleY_filt_Ytest = alpha * angles_gyroY(:,2) + (1-alpha) .* angle_accelY_Ytest;

figure(44)
plot(angle_accelX_Ytest)
figure(45)
plot(angle_accelY_Ytest)

figure(46)
plot(angleY_filt_Ytest)
hold on
plot(angles_gyroY(:,2))
hold off
title('Y Axis Data for Y Rotation Test with Complimentary Filtering')
xlabel('samples')
ylabel('degrees')
legend('show')
legend('filtered', 'gyroscope')
meanLine = refline(0, 83);
meanLine.Color = 'g';

figure(47)
plot(angleY_filt_Ytest - angles_gyroY(:,2))


%%
%% Drop tests

dropX = importdata('accelDropX.txt');
dropY = importdata('accelDropY.txt');
dropZ = importdata('accelDropZ.txt');

%applying offsets
accelX_offset = 0.0222; %.0222
accelY_offset = 0.0371;
accelZ_offset = 0.9134 - 1; %need to subtract 1 to get offset
gyroX_offset = 0.0084;
gyroY_offset = 1.2963;
gyroZ_offset = 1.2545;

offsets = [accelX_offset accelY_offset accelZ_offset gyroX_offset gyroY_offset gyroZ_offset];

for n = 1:6
    dropX(:,n) = dropX(:,n) - offsets(n);
    dropY(:,n) = dropY(:,n) - offsets(n);
    dropZ(:,n) = dropZ(:,n) - offsets(n);
end

figure(30)
plot(dropX(:,1:3));
title('X axis drop');
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');


figure(31)
plot(dropY(:,1:3));
title('Y axis drop');
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');

figure(32)
plot(dropZ(:,1:3));
title('Z axis drop');
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');

%% Drop test with 4g range

dropX4g = importdata('accelDropX_4g.txt');
dropY4g = importdata('accelDropY_4g.txt');
dropZ4g = importdata('accelDropZ_4g.txt');

%applying offsets
accelX_offset = 0.0222; %.0222
accelY_offset = 0.0371;
accelZ_offset = 0.9134 - 1; %need to subtract 1 to get offset
gyroX_offset = 0.0084;
gyroY_offset = 1.2963;
gyroZ_offset = 1.2545;

offsets = [accelX_offset accelY_offset accelZ_offset gyroX_offset gyroY_offset gyroZ_offset];

for n = 1:6
    dropX4g(:,n) = dropX4g(:,n) - offsets(n);
    dropY4g(:,n) = dropY4g(:,n) - offsets(n);
    dropZ4g(:,n) = dropZ4g(:,n) - offsets(n);
end

figure(33)
plot(dropX4g(:,1:3));
title('X Axis Drop');
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
meanLine = refline(0, 0);
meanLine.Color = 'g';
ylabel('gravity (g)')
xlabel('samples')

figure(34)
plot(dropY4g(:,1:3));
title('Y Axis Arop');
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
meanLine = refline(0, 0);
meanLine.Color = 'g';
ylabel('gravity (g)')
xlabel('samples')

figure(35)
plot(dropZ4g(:,1:3));
title('Z Axis Arop');
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
meanLine = refline(0, 0);
meanLine.Color = 'g';
ylabel('gravity (g)')
xlabel('samples')

%% Smoothing acceleration to use as metric instead of trying to calculate velocity and displacement 

% low pass filter
alpha = .65;
[m, n] = size(accelX(:,1:3));
lpf_accel_Xtest = zeros(m, n);

for i = 1:m
    if i == 1
        lpf_accel_Xtest(i,1:3) = accelX(i,1:3);  %(1-alpha)*accelX(i,1:3);
    else
        lpf_accel_Xtest(i,1:3) = (1-alpha)*accelX(i,1:3) + alpha * lpf_accel_Xtest(i-1,1:3);
    end
end

figure(48)
plot(lpf_accel_Xtest(:,1:3))
figure(49)
plot(accelX(:,1:3))

temp = rk_integrator(lpf_accel_Xtest(:,1));
figure(50)
plot(temp)

%% low pass filter for drop test
% low pass filter
alpha = .65;
[m, n] = size(dropX4g(:,1:3));
lpf_accel_Xtest = zeros(m, n);

for i = 1:m
    if i == 1
        lpf_accel_Xtest(i,1:3) = (1-alpha)*dropX4g(i,1:3);
    else
        lpf_accel_Xtest(i,1:3) = (1-alpha)*dropX4g(i,1:3) + alpha * dropX4g(i-1,1:3);
    end
end

figure(51)
subplot(2,1,1)
plot(lpf_accel_Xtest)
title('X Axis Drop Test with Low Pass Filter')
ylabel('gravity (g)')
xlabel('samples')
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
subplot(2,1,2)
plot(dropX4g(:,1:3))
title('X Axis Drop Test')
ylabel('gravity (g)')
xlabel('samples')
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');

%y test
[m, n] = size(dropY4g(:,1:3));
lpf_accel_Ytest = zeros(m, n);

for i = 1:m
    if i == 1
        lpf_accel_Ytest(i,1:3) = (1-alpha)*dropY4g(i,1:3);
    else
        lpf_accel_Ytest(i,1:3) = (1-alpha)*dropY4g(i,1:3) + alpha * dropY4g(i-1,1:3);
    end
end

figure(52)
subplot(2,1,1)
plot(lpf_accel_Ytest)
title('Y Axis Drop Test with Low Pass Filter')
ylabel('gravity (g)')
xlabel('samples')
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
subplot(2,1,2)
plot(dropY4g(:,1:3))
title('Y Axis Drop Test')
ylabel('gravity (g)')
xlabel('samples')
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');

%z test
[m, n] = size(dropZ4g(:,1:3));
lpf_accel_Ztest = zeros(m, n);

for i = 1:m
    if i == 1
        lpf_accel_Ztest(i,1:3) = (1-alpha)*dropZ4g(i,1:3);
    else
        lpf_accel_Ztest(i,1:3) = (1-alpha)*dropZ4g(i,1:3) + alpha * dropZ4g(i-1,1:3);
    end
end

figure(53)
subplot(2,1,1)
plot(lpf_accel_Ztest)
title('Z Axis Drop Test with Low Pass Filter')
ylabel('gravity (g)')
xlabel('samples')
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
subplot(2,1,2)
plot(dropZ4g(:,1:3))
title('Z Axis Drop Test')
ylabel('gravity (g)')
xlabel('samples')
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');

% isolated axis plotting for targeted axis being tested
% x axis
figure(54)
subplot(2,1,1)
plot(lpf_accel_Xtest(:,1))
title('X Axis Data for X Axis Drop Test with Low Pass Filter')
ylabel('gravity (g)')
xlabel('samples')
meanLine = refline(0, 0);
meanLine.Color = 'g';
ylabel('gravity (g)')
xlabel('samples')
subplot(2,1,2)
plot(dropX4g(:,1))
title('X Axis Data for X Axis Drop Test')
ylabel('gravity (g)')
xlabel('samples')
meanLine = refline(0, 0);
meanLine.Color = 'g';
ylabel('gravity (g)')
xlabel('samples')

%y axis
figure(55)
subplot(2,1,1)
plot(lpf_accel_Ytest(:,2))
title('Y Axis Data for Y Axis Drop Test with Low Pass Filter')
ylabel('gravity (g)')
xlabel('samples')
meanLine = refline(0, 0);
meanLine.Color = 'g';
ylabel('gravity (g)')
xlabel('samples')
subplot(2,1,2)
plot(dropY4g(:,2))
title('Y Axis Data for Y Axis Drop Test')
ylabel('gravity (g)')
xlabel('samples')
meanLine = refline(0, 0);
meanLine.Color = 'g';
ylabel('gravity (g)')
xlabel('samples')

%z axis
figure(56)
subplot(2,1,1)
plot(lpf_accel_Ztest(:,3))
title('Z Axis Data for Z Axis Drop Test with Low Pass Filter')
ylabel('gravity (g)')
xlabel('samples')
meanLine = refline(0, 0);
meanLine.Color = 'g';
ylabel('gravity (g)')
xlabel('samples')
subplot(2,1,2)
plot(dropZ4g(:,3))
title('Z Axis Data for Z Axis Drop Test')
ylabel('gravity (g)')
xlabel('samples')
meanLine = refline(0, 0);
meanLine.Color = 'g';
ylabel('gravity (g)')
xlabel('samples')


