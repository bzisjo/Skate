function [ output ] = rect_lpf( input, cutoff )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    input_fft = fftshift(fft(input));
    i = floor(length(input_fft)/2);
    lower_ind = i - cutoff;
    upper_ind = i + cutoff;

    z_low = zeros(lower_ind-1,1);
    z_up = zeros(length(input_fft)-upper_ind,1);
    input_fft_del = input_fft;
    input_fft_del(1:lower_ind-1) = z_low;
    input_fft_del(upper_ind+1:length(input_fft)) = z_up;

    %figure(14)
%plot(1:length(input_fft_del),abs(input_fft_del))
%axis([-20 200 -100 500])
%title('X axis accel fft data after "windowing"')

    output = ifft(fftshift(input_fft_del));
%figure(15)
%plot(1:length(xaccel_lpf),xaccel_lpf)
%title('X axis accel after lpf')

end

