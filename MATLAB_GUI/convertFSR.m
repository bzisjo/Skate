function [ weight ] = convertFSR( V_fsr )
%   convertFSR takes in the raw data from the FSR and calculates
%   the weight being applied to FSR
%   The output (weight) is in lbs
%   Detailed explanation goes here
    V_ref = (V_fsr / 255) * 3.3;
    I = V_ref / 4;
    FSR = (3.3 - V_ref) ./ I;
    weight = (FSR / 2.3044) .^(1/-0.665);
end

