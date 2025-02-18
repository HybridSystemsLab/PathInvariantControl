clear all 
close all

%% System Parameters
% Controller state
global q;

v = 0.1; %%% Not the linear velocity input but the fixed speed
k1 = 10; %
k2 = 10;
k3 = 10;%
k4 = 100;
k5 = 100;
L=0.3;

%%
%%% Initial Conditions
X1_0= 0; %% x-position
X2_0= 0;  %% y-position
X3_0=0; %% car orientation
X4_0=0;    %% Wheel angle
X5_0=0; %% Fictitious state and we can always initialize it with zero
X6_0=0; %% Fictitious state and we can always initialize it with zero
r = 1.5;

v_input = 0;

% Initial condition Vector
x0 = [X1_0;X2_0;X3_0;X4_0;X5_0;X6_0]

u = [0.6, 0]'; % u = (v, omega)


% Simulation time
Tmax = 20;  % End point
dt =0.01; % Time step
T = 0:dt:Tmax; % Time vector

% Simulation setup
x_all = zeros(size(x0,1),length(T));  % Intialization of a matrix to save all the values of the states 
x_Old = x0; % Giving x0 as a the name of an old state to use it in the discretized equations
x_all(:,1) = x0; % Saving the initial state in the full state matrix
xi1_plot = zeros(1,length(T));
xi2_plot = zeros(1,length(T));
xi3_plot = zeros(1,length(T));
eta1_plot = zeros(1,length(T));
eta2_plot = zeros(1,length(T));
eta3_plot = zeros(1,length(T));
u1_plot = zeros(1,length(T));
u2_plot = zeros(1,length(T));

%%% Giving values to the individual componets for the ease of coding
x1_Old = x_Old(1);
x2_Old = x_Old(2);
x3_Old = x_Old(3);
x4_Old = x_Old(4);
x5_Old = x_Old(5);
x6_Old = x_Old(6);
v_input_Old = v_input;


%%% for the first iteration the current states are assumed to be the old
%%% states
x1 = x1_Old;
x2 = x2_Old;
x3 = x3_Old;
x4 = x4_Old;
x5 = x5_Old;
x6 = x6_Old;

%%% adding memory for velocity and steering angle
vel = zeros(1,length(T));
ang = zeros(1,length(T));
vel0 = 0;
ang0 = 0;
accel = 0;

% path generation
load('newmotion.mat');
xpath = motionplan(3,:);
ypath = motionplan(4,:);

Kp = 0.7;

switchpoint = 0;

%% Controller
for i=1:length(T)-1
    
    x = [x1;x2;x3;x4;x5;x6];
    q = NCircle(x);

    if(q == 0)

        if( mod(i, 10) == 0)

            % find closest tp in range
            kdd = 2;
            ld = kdd*u(1);
            tp_max = ld+0.2;
            tp_min = ld-0.2;
            min = ld+2;
            xg = 0;
            yg = 0;

            %     bwx = z(1, end) - lr*cos(z(3, end));
            %     bwy = z(2, end) - lr*sin(z(3, end));
            bwx = x1;
            bwy = x2;
            ispan = (i/10):length(xpath);
            for i1 = ispan
                d = sqrt( ( bwx-xpath(i1) )^2 + ( bwy-ypath(i1) )^2 );
                %     a = atan2(ypath(i) - bwy,xpath(i) -bwx);
                if(d<tp_max) && (d>tp_min)
                    a =  x3  - (atan2((ypath(i1) - bwy),(xpath(i1) - bwx)));
                    if(a > pi)
                        a = a-2*pi;
                    end
                    if (a < pi/2) && (a > -pi/2) && (d<min)
                        min = d;
                        xg = xpath(i1);
                        yg = ypath(i1);
                        alpha=a;
                    end
                end
            end

            % steering angle
            % alpha=atan2(yg - bwy,xg - bwx);
            % delta = atan((2*L*sin(alpha))/(min));
            delta = -atan2((2*L*sin(alpha)),(ld));
            %         z(4,end) = delta;

            tspan = 1:2;

            for t = tspan
                xdot = u(1) * cos(x3);
                ydot = u(1) * sin(x3);
                thetadot = u(1) * (tan(x4))/(L);
                omega = -x(4) + delta;

                zdot = [xdot, ydot, thetadot, omega]';

                % The system (Unicycle)
                x1 = x1_Old + 0.05*( xdot );
                x2 = x2_Old + 0.05*( ydot );
                x3 = x3_Old + 0.08*( thetadot );
                x4 = x4_Old + omega*0.5;
                % % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % %
                %      x5 = x5_Old + x6*dt; %%% Fictitius state
                %      x6 = x6_Old + u2*dt; %%% Fictitius state


                %%% Making the state vector, that might be helpful for the debugging
                %%% purposes
                x = [x1;x2;x3;x4;x5;x6];

                x_Old = x; %%% At the end of the an iteration the current values become the old values
                x1_Old = x_Old(1);
                x2_Old = x_Old(2);
                x3_Old = x_Old(3);
                x4_Old = x_Old(4);
                x5_Old = x_Old(5);
                x6_Old = x_Old(6);

                v_input_Old = v_input;
            end

            %%% Saving the current values of the state to the full state matrix,
            %%% that might be useful for plotting purposes



        end
        x_all(:,i+1) = x;
        vel(i) = u(1);
        ang(i) = (ang0 + u(2)*dt);
        clear min;
        ang(i) = min([0.2, x4]);
        vel0 = vel(i);
        ang0 = ang(i);

    else
        if(switchpoint == 0)
            switchpoint = i;
        end

        % Transformed states

        xi1 = - r^2 + x1^2 + x2^2;
        xi2 = 2*(v + x5)*(x1*cos(x3) + x2*sin(x3));
        xi3 = (v*sin(x3) + x5*sin(x3))*(2*v*sin(x3) + 2*x5*sin(x3)) + ((v*tan(x4))/L + (x5*tan(x4))/L)*(2*x2*(v*cos(x3) + x5*cos(x3)) - 2*x1*(v*sin(x3) + x5*sin(x3))) + x6*(2*x1*cos(x3) + 2*x2*sin(x3)) + (v*cos(x3) + x5*cos(x3))*(2*v*cos(x3) + 2*x5*cos(x3));
        Lg1Lf2S = (2*x2*(v*cos(x3) + x5*cos(x3)) - 2*x1*(v*sin(x3) + x5*sin(x3)))*(v/(L*cos(x4)^2) + x5/(L*cos(x4)^2));
        Lg2Lf2S = 2*x1*cos(x3) + 2*x2*sin(x3);
        Lf3S = -(2*v^3*x1*cos(x3)*tan(x4)^2 - 6*L^2*x5*x6 - 6*L^2*v*x6 + 2*x1*x5^3*cos(x3)*tan(x4)^2 + 2*v^3*x2*sin(x3)*tan(x4)^2 + 2*x2*x5^3*sin(x3)*tan(x4)^2 + 6*v*x1*x5^2*cos(x3)*tan(x4)^2 + 6*v^2*x1*x5*cos(x3)*tan(x4)^2 + 6*v*x2*x5^2*sin(x3)*tan(x4)^2 + 6*v^2*x2*x5*sin(x3)*tan(x4)^2 - 6*L*v*x2*x6*cos(x3)*tan(x4) - 6*L*x2*x5*x6*cos(x3)*tan(x4) + 6*L*v*x1*x6*sin(x3)*tan(x4) + 6*L*x1*x5*x6*sin(x3)*tan(x4))/L^2;

        eta1 = atan2(x2,x1);
        eta2 = -(x2*(v*cos(x3) + x5*cos(x3)) - x1*(v*sin(x3) + x5*sin(x3)))/(x1^2 + x2^2);
        eta3 = cos(x3)*(v + x5)*((sin(x3)*(v + x5))/(x1^2 + x2^2) - (2*x1^2*sin(x3)*(v + x5))/(x1^2 + x2^2)^2 + (2*x1*x2*cos(x3)*(v + x5))/(x1^2 + x2^2)^2) - x6*((x2*cos(x3))/(x1^2 + x2^2) - (x1*sin(x3))/(x1^2 + x2^2)) - sin(x3)*(v + x5)*((cos(x3)*(v + x5))/(x1^2 + x2^2) - (2*x2^2*cos(x3)*(v + x5))/(x1^2 + x2^2)^2 + (2*x1*x2*sin(x3)*(v + x5))/(x1^2 + x2^2)^2) + (tan(x4)*((x1*cos(x3)*(v + x5))/(x1^2 + x2^2) + (x2*sin(x3)*(v + x5))/(x1^2 + x2^2))*(v + x5))/L;
        Lg1Lf2P = (((x1*cos(x3)*(v + x5))/(x1^2 + x2^2) + (x2*sin(x3)*(v + x5))/(x1^2 + x2^2))*(v + x5)*(tan(x4)^2 + 1))/L;
        Lg2Lf2P =(x1*sin(x3))/(x1^2 + x2^2) - (x2*cos(x3))/(x1^2 + x2^2);
        Lf3P = -((v + x5)*(tan(x4)*x1^2 - 2*L*sin(x3)*x1 + tan(x4)*x2^2 + 2*L*cos(x3)*x2)*(L*v^2*x1^2 + L*v^2*x2^2 + L*x1^2*x5^2 + L*x2^2*x5^2 + 2*L*v*x1^2*x5 + 2*L*v*x2^2*x5 + 2*L*v^2*x1^2*cos(2*x3) - 2*L*v^2*x2^2*cos(2*x3) + 2*L*x1^2*x5^2*cos(2*x3) - 2*L*x2^2*x5^2*cos(2*x3) - v^2*x2^3*cos(x3)*tan(x4) - x2^3*x5^2*cos(x3)*tan(x4) - 3*L*x1^3*x6*cos(x3) + v^2*x1^3*sin(x3)*tan(x4) + x1^3*x5^2*sin(x3)*tan(x4) - 3*L*x2^3*x6*sin(x3) - v^2*x1^2*x2*cos(x3)*tan(x4) - x1^2*x2*x5^2*cos(x3)*tan(x4) - 3*L*x1*x2^2*x6*cos(x3) + v^2*x1*x2^2*sin(x3)*tan(x4) + x1*x2^2*x5^2*sin(x3)*tan(x4) - 3*L*x1^2*x2*x6*sin(x3) + 4*L*v*x1^2*x5*cos(2*x3) - 4*L*v*x2^2*x5*cos(2*x3) + 4*L*v^2*x1*x2*sin(2*x3) + 4*L*x1*x2*x5^2*sin(2*x3) - 2*v*x2^3*x5*cos(x3)*tan(x4) + 2*v*x1^3*x5*sin(x3)*tan(x4) - 2*v*x1^2*x2*x5*cos(x3)*tan(x4) + 2*v*x1*x2^2*x5*sin(x3)*tan(x4) + 8*L*v*x1*x2*x5*sin(2*x3)))/(L^2*(x1^2 + x2^2)^3);

        xi1_plot(i) = xi1;
        xi2_plot(i) = xi2;
        xi3_plot(i) = xi3;

        eta1_plot(i) = eta1;
        eta2_plot(i) = eta2;
        eta3_plot(i) = eta3;

        %%% Control inputs
        D=[Lg1Lf2P Lg2Lf2P;Lg1Lf2S Lg2Lf2S];
        M=inv(D);
        %det_d=det(D);
        %v_tang = -k4*(eta2-0.5)-k5*(eta3); %%working controller for a constant (takes 1 or 2 min to execute)


        v_tang = -k4*(eta2-0.5)-k5*(eta3); % Trangential controller

        v_tran=-k1*xi1-k2*xi2-k3*xi3;  % Transversal controller

        u=M*[-Lf3P+v_tang;-Lf3S+v_tran];
        u1=u(1);
        u2=u(2);

        %%%%%%%%% Taking the derivative to get the real input
        %%% Controller dynamics %%%
        x6 = x6_Old + u2*dt;
        x5 = x5_Old + dt*x6;
        v_input = x5 + v;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        saturator = 1;

        if u1 > saturator
            u1 =saturator ;
        end

        if u1 < -saturator
            u1 =-saturator ;
        end

        if v_input > saturator
            v_input =saturator ;
        end

        if v_input < -saturator
            v_input =-saturator ;
        end

        u1_plot(i) = u1;
        u2_plot(i) = u2;


        % The system (Unicycle)
        x1 = x1_Old + dt*( v_input *cos(x3) );
        x2 = x2_Old + dt*( v_input*sin(x3) );
        x3 = x3_Old + dt*( (v_input/L)*tan(x4) );
        x4 = x4_Old + u1*dt;
        % % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % %
        %      x5 = x5_Old + x6*dt; %%% Fictitius state
        %      x6 = x6_Old + u2*dt; %%% Fictitius state


        %%% Making the state vector, that might be helpful for the debugging
        %%% purposes
        x = [x1;x2;x3;x4;x5;x6];

        x_Old = x; %%% At the end of the an iteration the current values become the old values
        x1_Old = x_Old(1);
        x2_Old = x_Old(2);
        x3_Old = x_Old(3);
        x4_Old = x_Old(4);
        x5_Old = x_Old(5);
        x6_Old = x_Old(6);

        v_input_Old = v_input;

        %%% Saving the current values of the state to the full state matrix,
        %%% that might be useful for plotting purposes
        x_all(:,i+1) = x;
        accel = accel + u2*dt;
        vel(i) = (vel0 + accel*dt);
        ang(i) = (ang0 + u1*dt);
        clear min;
        ang(i) = min([0.2, ang(i)]);
        vel0 = vel(i);
        ang0 = ang(i);
    end
end

%% Plotting

% Trajectory
figure(1);
hold on;
plot(xpath, ypath, 'LineStyle', '--', 'color', 'black', 'LineWidth',3);

lambda = -pi:0.01:pi;
% desired path
% plot(lambda, cos(lambda), 'r--', 'linewidth',1);
plot(r*cos(lambda),r*sin(lambda), 'r--','color','green', 'linewidth',4);
plot(x_all(1,1:switchpoint), x_all(2,1:switchpoint), 'r','color','red','linewidth',3);
plot(x_all(1,switchpoint:end), x_all(2,switchpoint:end), 'r','color','blue','linewidth',2);
title('Trajectory Generation')
xlabel('x_{1}')
ylabel('x_{2}')
U = [1,0,0.2,0.2;
    -0.5,0.5,0.2,0.2;
    0.2,0.6,0.2,0.2];
rectangle('Position',U(1,:),'FaceColor',[1 0 0])
rectangle('Position',U(2,:),'FaceColor',[1 0 0])
rectangle('Position',U(3,:),'FaceColor',[1 0 0])
%%% displaying the initial marker
plot(x0(1), x0(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
grid on;
axis equal
legend('Generated Trajectory', 'Path to follow', 'Initial Position')
hold off;

figure(2)
subplot(4,1,1)
plot(T, x_all(1,:));
ylabel('x')
subplot(4,1,2)
plot(T, x_all(2,:))
ylabel('y')
subplot(4,1,3)
plot(T, x_all(3,:));
ylabel('\theta')
subplot(4,1,4)
plot(T,ang)
ylabel('\delta')

figure(3);
plot(T,xi1_plot,T,xi2_plot,T,xi3_plot)
title('\xi_1,\xi_2,\xi_3 versus time');
xlabel('t(sec)')
ylabel('position on the set \Gamma^{*}')
grid on;
legend('\xi_1','\xi_2','\xi_3')

figure(4);
plot(T,u1_plot)
title('u_1 versus time');
xlabel('t(sec)')
ylabel('Magnitude of the control input')
grid on;
legend('u_1')

figure(5);
plot(T,u2_plot)
title('u_2 versus time');
xlabel('t(sec)')
ylabel('Magnitude of the control input')
grid on;
legend('u_2')