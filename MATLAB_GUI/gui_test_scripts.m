%% test scripts and functions for the gui
% LOAD
%[filename, pathname] = uigetfile('*.txt', 'Select Skate Data to Load');
%test_data = importdata('/home/chu49/ECE445/Skate/Test data/masterdata.txt');    %for linux
test_data = importdata('U:\documents\ECE445\dev\Skate\Test data\masterdata.txt');   %for windows
% Unpack
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

%% View fft of accleration data of session 1

xaccel_fft = fftshift(fft(imu_data_s1(:,1)));
yaccel_fft = fftshift(fft(imu_data_s1(:,2)));
zaccel_fft = fftshift(fft(imu_data_s1(:,3)));

figure(13)
subplot(3,1,1)
plot(1:length(xaccel_fft),abs(xaccel_fft))
axis tight
axis([1000 1500 -100 300])
subplot(3,1,2)
plot(1:length(yaccel_fft),abs(yaccel_fft))
axis tight
%axis([-20 20 -2000 2000])
subplot(3,1,3)
plot(1:length(zaccel_fft),abs(zaccel_fft))
%axis([-20 20 -1000 3000])

%% testing filtering by just deleting values after 20
% x axis
cutoff = 20;

%[~, i] = max(xaccel_fft);
i = floor(length(xaccel_fft)/2);
lower_ind = i - cutoff;
upper_ind = i + cutoff;

z_low = zeros(lower_ind-1,1);
z_up = zeros(length(xaccel_fft)-upper_ind,1);
xaccel_fft_del = xaccel_fft;
xaccel_fft_del(1:lower_ind-1) = z_low;
xaccel_fft_del(upper_ind+1:length(xaccel_fft)) = z_up;

figure(14)
plot(1:length(xaccel_fft_del),abs(xaccel_fft_del))
%axis([-20 200 -100 500])
title('X axis accel fft data after "windowing"')

xaccel_lpf = ifft(fftshift(xaccel_fft_del));
figure(15)
plot(1:length(xaccel_lpf),xaccel_lpf)
title('X axis accel after lpf')

% y axis
i = floor(length(yaccel_fft)/2);
lower_ind = i - cutoff;
upper_ind = i + cutoff;

z_low = zeros(lower_ind-1,1);
z_up = zeros(length(yaccel_fft)-upper_ind,1);
yaccel_fft_del = yaccel_fft;
yaccel_fft_del(1:lower_ind-1) = z_low;
yaccel_fft_del(upper_ind+1:length(yaccel_fft)) = z_up;

figure(16)
plot(1:length(yaccel_fft_del),yaccel_fft_del)
%axis([-20 200 -100 500])
title('Y axis accel fft data after "windowing"')

yaccel_lpf = ifft(fftshift(yaccel_fft_del));
figure(17)
plot(1:length(yaccel_lpf),yaccel_lpf)
title('Y axis accel accelafter lpf')

% z axis
i = floor(length(zaccel_fft)/2);
lower_ind = i - cutoff;
upper_ind = i + cutoff;

z_low = zeros(lower_ind-1,1);
z_up = zeros(length(zaccel_fft)-upper_ind,1);
zaccel_fft_del = zaccel_fft;
zaccel_fft_del(1:lower_ind-1) = z_low;
zaccel_fft_del(upper_ind+1:length(zaccel_fft)) = z_up;


figure(18)
plot(1:length(zaccel_fft_del),zaccel_fft_del)
%axis([-20 200 -100 500])
title('Z axis accel fft data after "windowing"')

zaccel_lpf = ifft(fftshift(zaccel_fft_del));
figure(19)
plot(1:length(zaccel_lpf),zaccel_lpf)
title('Z axis accel after lpf')

%% test function

output = rect_lpf(imu_data_s1(:,1), 20);
figure()
plot(1:length(output),output)

%% View fft of gyroscope data of session 1

xgyro_fft = fft(imu_data_s1(:,4));
ygyro_fft = fft(imu_data_s1(:,5));
zgyro_fft = fft(imu_data_s1(:,6));

figure(24)
subplot(3,1,1)
plot(1:length(xgyro_fft),abs(xgyro_fft))
%axis tight
%axis([-20 200 -100 500])
subplot(3,1,2)
plot(1:length(ygyro_fft),abs(ygyro_fft))
%axis tight
%axis([-20 20 -2000 2000])
subplot(3,1,3)
plot(1:length(zgyro_fft),abs(zgyro_fft))
%axis([-20 20 -1000 3000])

%% Gyroscope filtering test
cutoff = 20;
n = length(xaccel_fft)-cutoff;
z = zeros(1,n).';
xaccel_fft_del = xaccel_fft;
xaccel_fft_del(cutoff+1:length(xaccel_fft)) = z;
xaccel_fft_del = xaccel_fft_del.';

figure(18)
plot(1:length(xaccel_fft_del),xaccel_fft_del)
axis([-20 200 -100 500])
title('X axis accel fft data after "windowing"')

xaccel_lpf = ifft(xaccel_fft_del);
figure(19)
plot(1:length(xaccel_lpf),xaccel_lpf)
title('X axis accel after lpf')

% y axis
n = length(yaccel_fft)-cutoff;
z = zeros(1,n).';
yaccel_fft_del = yaccel_fft;
yaccel_fft_del(cutoff+1:length(yaccel_fft)) = z;
yaccel_fft_del = yaccel_fft_del.';

figure(20)
plot(1:length(yaccel_fft_del),yaccel_fft_del)
axis([-20 200 -100 500])
title('Y axis accel fft data after "windowing"')

yaccel_lpf = ifft(yaccel_fft_del);
figure(21)
plot(1:length(yaccel_lpf),yaccel_lpf)
title('Y axis accel accelafter lpf')

% z axis
n = length(zaccel_fft)-cutoff;
z = zeros(1,n).';
zaccel_fft_del = zaccel_fft;
zaccel_fft_del(cutoff+1:length(zaccel_fft)) = z;
zaccel_fft_del = zaccel_fft_del.';

figure(22)
plot(1:length(zaccel_fft_del),zaccel_fft_del)
axis([-20 200 -100 500])
title('Z axis accel fft data after "windowing"')

zaccel_lpf = ifft(zaccel_fft_del);
figure(23)
plot(1:length(zaccel_lpf),zaccel_lpf)
title('Z axis accel after lpf')
