%% Plotting values from RealTerm/IMU data

IMU_data = importdata('capture.txt');

figure(1)
plot(IMU_data);
%%
yaw_temp = IMU_data(:,3);
yaw_temp = yaw_temp(100:200);
figure(2)
plot(yaw_temp);

%%
prev_val = 0;
output = zeros(size(yaw_temp));
for n = 1 : size(yaw_temp)
    output(n) = yaw_temp(n) - prev_val + 1.5128;
    prev_val = yaw_temp(n);
end
figure(3)
plot(output)

%% Loading IMU data

IMU_mahony = importdata('capture.txt');
IMU_1 = importdata('capture_tau1.txt');
IMU_09 = importdata('capture_tau09.txt');
IMU_08 = importdata('capture_tau08.txt');
IMU_07 = importdata('capture_tau07.txt');
IMU_06 = importdata('capture_tau06.txt');
IMU_05 = importdata('capture_tau05.txt');
IMU_04 = importdata('capture_tau04.txt');
IMU_03 = importdata('capture_tau03.txt');
IMU_02 = importdata('capture_tau02.txt');
IMU_01 = importdata('capture_tau01.txt');
IMU_007 = importdata('capture_tau007.txt');
IMU_005 = importdata('capture_tau005.txt');
IMU_003 = importdata('capture_tau003.txt');
IMU_001 = importdata('capture_tau001.txt');
IMU_0007 = importdata('capture_tau0007.txt');
IMU_0005 = importdata('capture_tau0005.txt');
IMU_0003 = importdata('capture_tau0003.txt');

%% plotting IMU data of complementary filter with diff tau
figure(4)
plot(IMU_mahony);
title('IMU Data with Mahony Filter');

figure(5)
plot(IMU_1);
title('IMU Data with Complementary Filter, \tau = 1');

figure(6)
plot(IMU_05);
title('IMU Data with Complementary Filter, \tau = 0.5');cd ..


figure(7)
plot(IMU_06);
title('IMU Data with Complementary Filter, \tau = 0.6');

figure(8)
plot(IMU_07);
title('IMU Data with Complementary Filter, \tau = 0.7');

figure(9)
plot(IMU_08);
title('IMU Data with Complementary Filter, \tau = 0.8');

figure(10)
plot(IMU_09);
title('IMU Data with Complementary Filter, \tau = 0.9');

figure(12)
plot(IMU_01);


%% Plotting different roll of different tau values

figure(11);
plot(IMU_mahony(:,1));
hold on
plot(IMU_1(:,1));
plot(IMU_05(:,1));
plot(IMU_06(:,1));
plot(IMU_07(:,1));
plot(IMU_08(:,1));
plot(IMU_09(:,1));
plot(IMU_01(:,1));
hold off


%% finding max angle of roll for each tau
max_roll = zeros(1, 17);
max_roll(1) = max(IMU_1(:,1));
max_roll(2) = max(IMU_09(:,1));
max_roll(3) = max(IMU_08(:,1));
max_roll(4) = max(IMU_07(:,1));
max_roll(5) = max(IMU_06(:,1));
max_roll(6) = max(IMU_05(:,1));
max_roll(7) = max(IMU_04(:,1));
max_roll(8) = max(IMU_03(:,1));
max_roll(9) = max(IMU_02(:,1));
max_roll(10) = max(IMU_01(:,1));
max_roll(11) = max(IMU_007(:,1));
max_roll(12) = max(IMU_005(:,1));
max_roll(13) = max(IMU_003(:,1));
max_roll(14) = max(IMU_001(:,1));
max_roll(15) = max(IMU_0007(:,1));
max_roll(16) = max(IMU_0005(:,1));
max_roll(17) = max(IMU_0003(:,1));

%%
figure(13)
bar_labels = {'1', '0.9', '0.8', '0.7', '0.6', '0.5', '0.4', '0.3', '0.2' ,'0.1', '0.07', '0.05', '0.03', '0.01', '0.007', '0.005', '0.003'};
bar(max_roll);
set(gca, 'XTickLabel', bar_labels);

%% Get smaller set of roll max value bar graph
max_roll2 = zeros(1, 11);
max_roll2(1) = max(IMU_1(:,1));
max_roll2(2) = max(IMU_05(:,1));
max_roll2(3) = max(IMU_03(:,1));
max_roll2(4) = max(IMU_01(:,1));
max_roll2(5) = max(IMU_007(:,1));
max_roll2(6) = max(IMU_005(:,1));
max_roll2(7) = max(IMU_003(:,1));
max_roll2(8) = max(IMU_001(:,1));
max_roll2(9) = max(IMU_0007(:,1));
max_roll2(10) = max(IMU_0005(:,1));
max_roll2(11) = max(IMU_0003(:,1));

%%
figure(15)
bar_labels2 = {'1', '0.5', '0.3', '0.1', '0.07', '0.05', '0.03', '0.01', '0.007', '0.005', '0.003'};
bar(max_roll2);
set(gca, 'XTickLabel', bar_labels2);

%% Plotting different pitch of different tau values
figure(14);
plot(IMU_mahony(:,2));
hold on
plot(IMU_1(:,2));
plot(IMU_05(:,2));
plot(IMU_06(:,2));
plot(IMU_07(:,2));
plot(IMU_08(:,2));
plot(IMU_09(:,2));
plot(IMU_01(:,2));
hold off

%% temp
figure()
plot(IMU_07(:,1));
hold on
plot(IMU_07_retest(:,1));

%% For calculating offsets of the IMU for calibration

offsetData = importdata('convertedDataOffsets.txt');
accelX_offsets = offsetData(:,1);
accelY_offsets = offsetData(:,2);
accelZ_offsets = offsetData(:,3);
gyroX_offsets = offsetData(:,4);
gyroY_offsets = offsetData(:,5);
gyroZ_offsets = offsetData(:,6);

accelX_mean = mean(accelX_offsets)
accelX_std = std(accelX_offsets)

accelY_mean = mean(accelY_offsets)
accelY_std = std(accelY_offsets)

accelZ_mean = mean(accelZ_offsets)
accelZ_std = std(accelZ_offsets)

gyroX_mean = mean(gyroX_offsets)
gyroX_median = median(gyroX_offsets);
gyroX_std = std(gyroX_offsets)

gyroY_mean = mean(gyroY_offsets)
gyroY_median = median(gyroY_offsets);
gyroY_std = std(gyroY_offsets)

gyroZ_mean = mean(gyroZ_offsets)
gyroZ_median = median(gyroZ_offsets);
gyroZ_std = std(gyroZ_offsets)

%% Graphs for accelerometer still offsets
figure(16)
plot(accelX_offsets)
hold on
refline(0, accelX_mean)
accelX_low_refline = refline(0, accelX_mean - accelX_std);
set(accelX_low_refline, 'LineStyle', ':');
accelX_high_refline = refline(0, accelX_mean + accelX_std);
set(accelX_high_refline, 'LineStyle', ':');
title('X Accelerometer Still Data')
ylabel('g')
hold off

figure(17)
plot(accelY_offsets)
hold on
refline(0, accelY_mean)
accelY_low_refline = refline(0, accelY_mean - accelY_std);
set(accelY_low_refline, 'LineStyle', ':');
accelY_high_refline = refline(0, accelY_mean + accelY_std);
set(accelY_high_refline, 'LineStyle', ':');
title('Y Accelerometer Still Data')
ylabel('g')
hold off

figure(18)
plot(accelZ_offsets)
hold on
refline(0, accelZ_mean)
accelZ_low_refline = refline(0, accelZ_mean - accelZ_std);
set(accelZ_low_refline, 'LineStyle', ':');
accelZ_high_refline = refline(0, accelZ_mean + accelZ_std);
set(accelZ_high_refline, 'LineStyle', ':');
title('Z Accelerometer Still Data')
ylabel('g')
hold off

%% Graphs for gyroscope still offsets
figure(19)
plot(gyroX_offsets)
hold on
refline(0, gyroX_mean)
gyroX_low_refline = refline(0, gyroX_mean - gyroX_std);
set(gyroX_low_refline, 'LineStyle', ':');
gyroX_high_refline = refline(0, gyroX_mean + gyroX_std);
set(gyroX_high_refline, 'LineStyle', ':');
title('X Gyroscope Still Data')
ylabel('deg/sec')
hold off

figure(20)
plot(gyroY_offsets)
hold on
refline(0, gyroY_mean)
gyroY_low_refline = refline(0, gyroY_mean - gyroY_std);
set(gyroY_low_refline, 'LineStyle', ':');
gyroY_high_refline = refline(0, gyroY_mean + gyroY_std);
set(gyroY_high_refline, 'LineStyle', ':');
title('Y Gyroscope Still Data')
ylabel('deg/sec')
hold off

figure(21)
plot(gyroZ_offsets)
hold on
refline(0, gyroZ_mean)
gyroZ_low_refline = refline(0, gyroZ_mean - gyroZ_std);
set(gyroZ_low_refline, 'LineStyle', ':');
gyroZ_high_refline = refline(0, gyroZ_mean + gyroZ_std);
set(gyroZ_high_refline, 'LineStyle', ':');
title('Z Gyroscope Still Data')
ylabel('deg/sec')
hold off

%% Loading converted raw data for movement tests
accelX_move = importdata('accelX_test.txt');
accelY_move = importdata('accelY_test.txt');
accelZ_move = importdata('accelZ_test.txt');
roll_move = importdata('roll90_test.txt');
pitch_move = importdata('pitch90_test.txt');
yaw_move = importdata('yaw90_test.txt');

%% Graphing test translational movement data

figure(22)
plot(accelX_move(:,1:3))
legend('show')
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of X Axis Translational Test')
ylabel('g')

figure(23)
plot(accelY_move(:,1:3))
legend('show')
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of Y Axis Translational Test')

figure(24)
plot(accelZ_move(:,1:3))
legend('show')
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of Z Axis Translational Test')

figure(25)
plot(roll_move(:,1:3))
legend('show')
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of X Axis Rotational Test (Roll)')

figure(26)
plot(pitch_move(:,1:3))
legend('show')
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of Y Axis Rotational Test (Pitch)')

figure(27)
plot(yaw_move(:,1:3))
legend('show')
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of Z Axis Rotational Test (Yaw)')

%% Graphing test translational movement data

figure(28)
plot(accelX_move(:,4:6))
legend('show')
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of X Axis Translational Test')

figure(29)
plot(accelY_move(:,4:6))
legend('show')
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of Y Axis Translational Test')

figure(30)
plot(accelZ_move(:,4:6))
legend('show')
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of Z Axis Translational Test')

figure(31)
plot(roll_move(:,4:6))
legend('show')
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of X Axis Rotational Test (Roll)')

figure(32)
plot(pitch_move(:,4:6))
legend('show')
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of Y Axis Rotational Test (Pitch)')

figure(33)
plot(yaw_move(:,4:6))
legend('show')
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of Z Axis Rotational Test (Yaw)')

%% Loading for replotting tests done with 10 ms between data reads instead of 1 s
accelX_move_10ms = importdata('accelX_10ms.txt');
accelY_move_10ms = importdata('accelY_10ms.txt');
accelZ_move_10ms = importdata('accelZ_10ms.txt');

 %%
figure(34)
subplot(2,1,1)
plot(accelX_move_10ms(:,1:3));
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of X axis Translational Test (10ms)')

subplot(2,1,2)
plot(accelX_move_10ms(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of X axis Translational Test (10ms)')

%% Plotting for 50 ms because 10 ms did not completly capture data
accelX_move_50ms = importdata('accelX_50ms.txt');
accelY_move_50ms = importdata('accelY_50ms.txt');
accelZ_move_50ms = importdata('accelZ_50ms.txt');

%% Plotting for 50 ms because 10 ms did not completly capture data
accelX_move_100ms = importdata('accelX_100ms.txt');
accelY_move_100ms = importdata('accelY_100ms.txt');
accelZ_move_100ms = importdata('accelZ_100ms.txt');

%%

figure(35)
subplot(2,1,1)
plot(accelX_move_100ms(:,1:3));
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of X axis Translational Test (100ms)')
axis([0 120 -2 2]);

subplot(2,1,2)
plot(accelX_move_100ms(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of X axis Translational Test (100ms)')
axis([0 120 -400 400]);

figure(36)
subplot(2,1,1)
plot(accelY_move_100ms(:,1:3));
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of Y axis Translational Test (100ms)')
axis([0 140 -2 2])

subplot(2,1,2)
plot(accelY_move_100ms(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of Y axis Translational Test (100ms)')
axis([0 140 -400 400]);

figure(37)
subplot(2,1,1)
plot(accelZ_move_100ms(:,1:3));
legend('show');
legend('AccelX', 'AccelY', 'AccelZ');
title('Translational Data of Z axis Translational Test (100ms)')
axis([0 120 -2 2])

subplot(2,1,2)
plot(accelZ_move_100ms(:,4:6));
legend('show');
legend('GyroX', 'GyroY', 'GyroZ');
title('Rotational Data of Z axis Translational Test (100ms)')
axis([0 120 -400 400]);

%% Loading for replotting tests done with 10 ms between data reads instead of 1 s
accelX_move_10ms = importdata('accelX_10ms.txt');
accelY_move_10ms = importdata('accelY_10ms.txt');
accelZ_move_10ms = importdata('accelZ_10ms.txt');

%% testing integration with matlab int function
time = 0:.01:.01*(size(accelX_move_10ms(:,1))-1);
integral = cumtrapz(accelX_move_10ms(:,1))
figure()
plot(integral)


%% Speed Verification Test
 speed = importdata('speedVerification_10ms.txt')







