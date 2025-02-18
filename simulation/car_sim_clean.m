%% PathFollowing of Car-like Robot
clc
clear all
close all

options = optimoptions('quadprog','Display','off');
%% System Parameters
v = 0.1; %%% Not the linear velocity input but the fixed speed
k1 = 10; %
k2 = 10;
k3 = 10;%
k4 = 100;
k5 = 100;
L=0.02;
r=1; %% radius of the circle to follow

%%
%%% Initial Conditions
X1_0= r+ 0.6; %% x-position
X2_0= 0;  %% y-position
X3_0=pi/2; %% car orientation
X4_0=0;    %% Wheel angle
X5_0=0; %% Fictitious state and we can always initialize it with zero
X6_0=0; %% Fictitious state and we can always initialize it with zero

v_input = 0;

% Initial condition Vector
x0 = [X1_0;X2_0;X3_0;X4_0;X5_0;X6_0]


% Simulation time
Tmax = 200;  % End point
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

c = 0.6;
eps3 = 0;


freq = 0.1;
eta1_ref = sin(freq*T);
temp = sin(freq*T);
eta2_ref = temp(1:end-1);
% eta2_ref = diff(eta1_ref)/dt;
eta3_ref = diff(eta2_ref)/dt;
eta4_ref = diff(eta3_ref)/dt;

for i=1:length(T)-4
%% Transformed states

xi1 = - r^2 + x1^2 + x2^2;
xi2 = 2*(v + x5)*(x1*cos(x3) + x2*sin(x3));
xi3 = (v*sin(x3) + x5*sin(x3))*(2*v*sin(x3) + 2*x5*sin(x3)) + ((v*tan(x4))/L + (x5*tan(x4))/L)*(2*x2*(v*cos(x3) + x5*cos(x3)) - 2*x1*(v*sin(x3) + x5*sin(x3))) + x6*(2*x1*cos(x3) + 2*x2*sin(x3)) + (v*cos(x3) + x5*cos(x3))*(2*v*cos(x3) + 2*x5*cos(x3));
Lg1Lf2S = (2*x2*(v*cos(x3) + x5*cos(x3)) - 2*x1*(v*sin(x3) + x5*sin(x3)))*(v/(L*cos(x4)^2) + x5/(L*cos(x4)^2));
Lg2Lf2S = 2*x1*cos(x3) + 2*x2*sin(x3);
Lf3S = -(2*v^3*x1*cos(x3)*tan(x4)^2 - 6*L^2*x5*x6 - 6*L^2*v*x6 + 2*x1*x5^3*cos(x3)*tan(x4)^2 + 2*v^3*x2*sin(x3)*tan(x4)^2 + 2*x2*x5^3*sin(x3)*tan(x4)^2 + 6*v*x1*x5^2*cos(x3)*tan(x4)^2 + 6*v^2*x1*x5*cos(x3)*tan(x4)^2 + 6*v*x2*x5^2*sin(x3)*tan(x4)^2 + 6*v^2*x2*x5*sin(x3)*tan(x4)^2 - 6*L*v*x2*x6*cos(x3)*tan(x4) - 6*L*x2*x5*x6*cos(x3)*tan(x4) + 6*L*v*x1*x6*sin(x3)*tan(x4) + 6*L*x1*x5*x6*sin(x3)*tan(x4))/L^2;

eta1 = atan(x2/x1);
eta2 = -(x2*(v*cos(x3) + x5*cos(x3)) - x1*(v*sin(x3) + x5*sin(x3)))/(x1^2 + x2^2);
eta3 = ((v*sin(x3) + x5*sin(x3))/(x1^2 + x2^2) + (2*x1*(x2*(v*cos(x3) + x5*cos(x3)) - x1*(v*sin(x3) + x5*sin(x3))))/(x1^2 + x2^2)^2)*(v*cos(x3) + x5*cos(x3)) - (v*sin(x3) + x5*sin(x3))*((v*cos(x3) + x5*cos(x3))/(x1^2 + x2^2) - (2*x2*(x2*(v*cos(x3) + x5*cos(x3)) - x1*(v*sin(x3) + x5*sin(x3))))/(x1^2 + x2^2)^2) - (x6*(x2*cos(x3) - x1*sin(x3)))/(x1^2 + x2^2) + (((v*tan(x4))/L + (x5*tan(x4))/L)*(x1*(v*cos(x3) + x5*cos(x3)) + x2*(v*sin(x3) + x5*sin(x3))))/(x1^2 + x2^2);
Lg1Lf2P = ((x1*(v*cos(x3) + x5*cos(x3)) + x2*(v*sin(x3) + x5*sin(x3)))*((v*(tan(x4)^2 + 1))/L + (x5*(tan(x4)^2 + 1))/L))/(x1^2 + x2^2);
Lg2Lf2P = -(x2*cos(x3) - x1*sin(x3))/(x1^2 + x2^2);
Lf3P = -((v + x5)*(tan(x4)*x1^2 - 2*L*sin(x3)*x1 + tan(x4)*x2^2 + 2*L*cos(x3)*x2)*(L*v^2*x1^2 + L*v^2*x2^2 + L*x1^2*x5^2 + L*x2^2*x5^2 + 2*L*v*x1^2*x5 + 2*L*v*x2^2*x5 + 2*L*v^2*x1^2*cos(2*x3) - 2*L*v^2*x2^2*cos(2*x3) + 2*L*x1^2*x5^2*cos(2*x3) - 2*L*x2^2*x5^2*cos(2*x3) - v^2*x2^3*cos(x3)*tan(x4) - x2^3*x5^2*cos(x3)*tan(x4) - 3*L*x1^3*x6*cos(x3) + v^2*x1^3*sin(x3)*tan(x4) + x1^3*x5^2*sin(x3)*tan(x4) - 3*L*x2^3*x6*sin(x3) - v^2*x1^2*x2*cos(x3)*tan(x4) - x1^2*x2*x5^2*cos(x3)*tan(x4) - 3*L*x1*x2^2*x6*cos(x3) + v^2*x1*x2^2*sin(x3)*tan(x4) + x1*x2^2*x5^2*sin(x3)*tan(x4) - 3*L*x1^2*x2*x6*sin(x3) + 4*L*v*x1^2*x5*cos(2*x3) - 4*L*v*x2^2*x5*cos(2*x3) + 4*L*v^2*x1*x2*sin(2*x3) + 4*L*x1*x2*x5^2*sin(2*x3) - 2*v*x2^3*x5*cos(x3)*tan(x4) + 2*v*x1^3*x5*sin(x3)*tan(x4) - 2*v*x1^2*x2*x5*cos(x3)*tan(x4) + 2*v*x1*x2^2*x5*sin(x3)*tan(x4) + 8*L*v*x1*x2*x5*sin(2*x3)))/(L^2*(x1^2 + x2^2)^3);

xi1_plot(i) = xi1;
xi2_plot(i) = xi2;
xi3_plot(i) = xi3;

eta1_plot(i) = eta1;
eta2_plot(i) = eta2;
eta3_plot(i) = eta3;


    %%
    %%% Control inputs
    D=[Lg1Lf2P Lg2Lf2P;Lg1Lf2S Lg2Lf2S];
    M=inv(D);
    v_tran =- k1*xi1-k2*xi2-k3*xi3;  % Transversal controller -- v1_fbl
    v_tang = -0*(eta1 - eta1_ref(i)) -k4*(eta2 - eta2_ref(i)) -k4*(eta3 - eta3_ref(i)) + eta4_ref(i);
    
    %%%%%%% Barrier Function Logic  %%%%%%%%%%%%%%%%%%%%%%%%%
    B = (eta2 - 0.02) + eta3;
    Lf2B = eta3;
    LgLfB = 1;
    
    Q = 0.01;
    A_ineq = [-LgLfB];
    B_ineq = [Lf2B + 20*B*B];
    u = quadprog(Q,-v_tang/1000,A_ineq, B_ineq, [], [], [], [],[], options);
    
    v_tang_new = u;
    v_tran_new = v_tran;
    %%%%%%%% Barrier Function Logic ends %%%%%%%%%%%%%%%%%%%%%

    u=M*[-Lf3P+v_tang_new;-Lf3S+v_tran_new];
    u1=u(1);
    u2=u(2);

%%%%%%%%% Taking the derivative to get the real input
%%% Controller dynamics %%%
x6 = x6_Old + u2*dt;
x5 = x5_Old + dt*x6;
v_input = x5 + v;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

u1_plot(i) = u1;
u2_plot(i) = u2;


% The system (Unicycle)   
    x1 = x1_Old + dt*( v_input *cos(x3) );
    x2 = x2_Old + dt*( v_input*sin(x3) );
    x3 = x3_Old + dt*( (v_input/L)*tan(x4) );
    x4 = x4_Old + u1*dt;
% % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
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
    
    
   end

%% Plotting

% Trajectory
figure();
hold on;

lambda = -pi:0.01:pi;
    % desired path
    %plot(lambda, cos(lambda), 'r--', 'linewidth',1);
    plot(r*cos(lambda), r*sin(lambda), 'r--','color','green', 'linewidth',3);
%   title('Unicycle Following circle with different initial conditions')
    xlabel('$x_1[m]$','FontSize',16,'Interpreter','latex')
    ylabel('$x_2[m]$','FontSize',16,'Interpreter','latex')
    %%% displaying the initial marker
    plot(x0(1), x0(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
    plot(x_all(1,(1:end-4)), x_all(2,(1:end-4)), 'r','color','red','linewidth',2);
    legend({'desired path $\gamma$','robots Initial position','robots path'},'FontSize',10,'Interpreter','latex')
    grid on;


figure();
plot(T(1:end-1),eta1_plot(1:end-1),'r','color','red','linewidth',2)
hold on; 
plot(T(1:end-2),eta2_ref(1:end-1),'--r','color','green','linewidth',2)
plot(T(1:end-1),eta2_plot(1:end-1),'r','color','green','linewidth',2)
grid on;
legend({'$\eta_{1}$','$\eta^{ref}_{2}$','$\eta_{2}$'},'FontSize',14,'Interpreter','latex')
xlabel('$t[sec]$','FontSize',16,'Interpreter','latex')
ylabel('$\eta_{1}[rads],\eta_{2}[rads/sec]$','FontSize',16,'Interpreter','latex')

figure();
plot(T(1:end),v + x_all(5,:),'r','color','red','linewidth',2)
hold on;
plot(T(1:end-2),eta2_ref(1:end-1),'--r','color','green','linewidth',2)
plot(T(1:end-1),eta2_plot(1:end-1),'r','color','green','linewidth',2)
grid on;
legend({'$v + x_{5}$','$\eta^{ref}_{2}$','$\eta_{2}$'},'FontSize',14,'Interpreter','latex')
xlabel('$t[sec]$','FontSize',16,'Interpreter','latex')
ylabel('$(v + x_{5}),\eta_{2}$','FontSize',16,'Interpreter','latex')



figure();
plot(T(1:end-1),xi1_plot(1:end-1),'r','color','red','linewidth',2)
hold on; 
plot(T(1:end-1),xi2_plot(1:end-1),'r','color','green','linewidth',2)
plot(T(1:end-1),xi3_plot(1:end-1),'b','color','blue','linewidth',2)
grid on;
legend({'$\xi_{1}$','$\xi_{2}$','$\xi_{3}$'},'FontSize',14,'Interpreter','latex')
xlabel('$t[sec]$','FontSize',16,'Interpreter','latex')
ylabel('$\xi$','FontSize',16,'Interpreter','latex')

figure();
plot(T,u1_plot)
title('u_1 versus time');
xlabel('t(sec)')
ylabel('Magnitude of the control input')
grid on;
legend('u_1')

figure();
plot(T,u2_plot)
title('u_2 versus time');
xlabel('t(sec)')
ylabel('Magnitude of the control input')
grid on;
legend('u_2')


