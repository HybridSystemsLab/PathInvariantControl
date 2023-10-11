function [bool] = targetSetCircle(x)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
x1 = x(1); % x
x2 = x(2); % y
x3 = mod(x(3), 2*pi); % theta
x4 = x(4); % delta

% adjust these thresholds to expand target set
threshhold = 0.3; %5cm
angleThreshhold = pi/2;%approx 12deg

load('targetSet.mat');

bool = false;

[c, index] = min(abs(x_all(1,:)-x1));
target_x = x_all(1, index);
target_y = x_all(2, index);
target_theta = x_all(3, index);
target_delta = x_all(4, index);

if( (abs(x1-target_x)<threshhold) && (abs(x2-target_y)<threshhold) && (abs(x3-target_theta)<angleThreshhold) )
    bool = true;
end

