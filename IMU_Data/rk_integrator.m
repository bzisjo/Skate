function [ output ] = rk_integrator( input )
%Kutta Runge Integrator
%   Detailed explanation goes here
    [n, ~] = size(input);
    integraled = zeros(1, n);
    prev = [0 0 0];
    
    for i = 1:n
        if i == 1   % case where at first element
            integraled(i) = input(i) / 6;
        else
            integraled(i) = integraled(i-1) + (prev(3) + 2*prev(2) + 2*prev(1) + input(i))/6;
            prev(3) = prev(2);
            prev(2) = prev(1);
            prev(1) = input(i);
        end
    end
    output = integraled;
end

