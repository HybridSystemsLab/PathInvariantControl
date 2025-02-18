clear all 
close all

%% path generation
%%% s curve
t = 0:0.01:10;
rad = 50;
% xpath = t;
% a = 2;
% k = 1;
% ypath = 4 - (5*(1./(1+exp(-k.*(xpath-a)))));

load('motionplanCircle.mat');
xpath = motionplan(3,:);
ypath = motionplan(4,:);

%%% sine wave
% th = 0:pi/100:10*pi;
% rad = 50;
% xpath = th;
% ypath = 5*sin(th/4);

figure(1)
hold on
plot(xpath, ypath);
hold off

%% state

u = [0.1, 0]'; % u = (v, omega)

z = [0, 0, pi, 0;
     0, 0, pi, 0]'; % z = (x, y, theta, zeta)

L = 0.3;
lr = L/2;

Kp = 0.7;
dt = 0.1;


%% controls

for j = 1:200
    % find closest tp in range
    kdd = 10;
    ld = kdd*u(1);
    tp_max = ld+0.2;
    tp_min = ld-0.2;
    min = ld+1;
    xg = 0;
    yg = 0;

%     bwx = z(1, end) - lr*cos(z(3, end));
%     bwy = z(2, end) - lr*sin(z(3, end));
    bwx = z(1,end);
    bwy = z(2, end);
    ispan = j:length(xpath);
    for i = ispan
        d = sqrt( ( bwx-xpath(i) )^2 + ( bwy-ypath(i) )^2 );
        %     a = atan2(ypath(i) - bwy,xpath(i) -bwx);
        if(d<tp_max) && (d>tp_min)
            a =  z(3, end)  - atan2((ypath(i) - bwy),(xpath(i) - bwx));
            if(a > pi)
                a = a-2*pi;
            end
            if (a < pi/2) && (a > -pi/2) && (d<min)
                min = d;
                xg = xpath(i);
                yg = ypath(i);
                alpha=a;
            end
        end
    end

    % steering angle
    % alpha=atan2(yg - bwy,xg - bwx);
    % delta = atan((2*L*sin(alpha))/(min));
    delta = -atan2((2*L*sin(alpha)),(ld));
%     z(4,end) = delta;

    tspan = 1:10;

    for t = tspan
        bwx = z(1, end);
        bwy = z(2, end);
        beta = atan(lr*tan(z(4, end))/L);
        xdot = u(1) * cos(z(3, end));
        ydot = u(1) * sin(z(3, end));
        thetadot = u(1) * (tan(z(4, end)))/(L);
        omega = u(2);

        zdot = [xdot*dt, ydot*dt, thetadot*dt, omega*dt]';

        z = [z, z(:, end)+zdot];

%         p controller
            e = delta - z(4, end);
            u(2) = Kp * e;

        %brake
%         derr = sqrt( ( bwx-xg )^2 + ( bwy-yg )^2 );
%         if(derr<1)
%             u(1) = 0;
%         end
    end



end
    figure(1)
    hold on
    axis equal
    plot (z(1, :), z(2, :));
% 
% figure(2)
% subplot(4,1,1)
% plot(tspan, z(1, 3:end))
% 
% subplot(4,1,2)
% plot(tspan, z(2, 3:end))
% 
% subplot(4,1,3)
% plot(tspan, rad2deg(z(3, 3:end)))
% 
% subplot(4,1,4)
% plot(tspan, rad2deg(z(4, 3:end)))
