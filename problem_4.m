% TTK4135 - Helicopter lab
% Hints/template for problem 2.
% Updated spring 2018, Andreas L. Flåten

%% Initialization and model definition
init; % Change this to the init file corresponding to your helicopter

% Discrete time system model. x = [lambda r p p_dot e e_dot]' 
delta_t	= 0.25; % sampling time
A2 = [
    1, delta_t, 0, 0, 0, 0;
    0, 1, -K_2*delta_t, 0, 0, 0;
    0, 0, 1, delta_t, 0, 0;
    0, 0, -K_1 * K_pp * delta_t, 1-K_1*K_pd*delta_t, 0, 0;
    0, 0, 0, 0, 1, delta_t;
    0, 0, 0, 0, -K_3 * K_ep * delta_t, 1 - K_3 * K_ed * delta_t
];

B2 = [
    0, 0;
    0, 0;
    0, 0;
    K_1*K_pp*delta_t, 0;
    0, 0;
    0, K_3 * K_ep * delta_t
];

% Number of states and inputs
mx = size(A2,2); % Number of states (number of columns in A)
mu = size(B2,2); % Number of inputs(number of columns in B)

% Initial values
x1_0 = pi;                             % Lambda
x2_0 = 0;                              % r
x3_0 = 0;                              % p
x4_0 = 0;                              % p_dot
x5_0 = 0;                              % e
x6_0 = 0;                              % e_dot
x0 = [x1_0 x2_0 x3_0 x4_0 x5_0 x6_0]'; % Initial values

% Time horizon and initialization
N  = 40;                 % Time horizon for states
M  = N;                  % Time horizon for inputs
z  = zeros(N*mx+M*mu,1); % Initialize z for the whole horizon
z0 = z;                  % Initial value for optimization

% Bounds
ul 	    = [deg2rad(-30); deg2rad(-30)]; % Lower bound on control
uu 	    = [deg2rad(30); deg2rad(30)];   % Upper bound on control
% TODO: This will put the constraint in p_c and not p

% Bound on states
xl      = -Inf*ones(mx,1);
xu      = Inf*ones(mx,1);
xl(3)   = ul(1);
xu(3)   = uu(1);
xl(5)   = ul(2);
xu(5)   = uu(2);
% This will put in the contrains in p from p_c

% Generate constraints on measurements and inputs
[vlb,vub]       = gen_constraints(N,M,xl,xu,ul,uu); % hint: gen_constraints
vlb(N*mx+M*mu - 1)  = 0;                % We want the last input to be zero
vub(N*mx+M*mu - 1)  = 0;                % We want the last input to be zero
vlb(N*mx+M*mu)  = 0;                    % We want the last input to be zero
vub(N*mx+M*mu)  = 0;                    % We want the last input to be zero

vub(N*(mx-1) + 1) = 0;
vub(N*(mx-1) + 2) = 0;
vub(N*(mx-1) + 3) = 0;
vub(N*(mx-1) + 4) = 0;
vub(N*(mx-1) + 5) = 0;
vub(N*(mx-1) + 6) = 0;
vlb(N*(mx-1) + 1) = 0;
vlb(N*(mx-1) + 2) = 0;
vlb(N*(mx-1) + 3) = 0;
vlb(N*(mx-1) + 4) = 0;
vlb(N*(mx-1) + 5) = 0;
vlb(N*(mx-1) + 6) = 0;

% Generate the matrix Q and the vector c (objecitve function weights in the QP problem) 
Q2 = zeros(mx,mx);
Q2(1,1) = 1;                            % Weight on state lambda
Q2(2,2) = 0;                            % Weight on state x2
Q2(3,3) = 0;                            % Weight on state x3
Q2(4,4) = 0;                            % Weight on state x4
Q2(5,5) = 0;
Q2(6,6) = 0;
P2 = diag([2, 1]);
Q = gen_q(Q2,P2,N,M);                                  % Generate Q, hint: gen_q
c = zeros(size(Q, 1), 1);                                  % Generate c, this is the linear constant term in the QP

% Cost function
cost_function = @(x) 0.5 * x' * Q * x + c' * x;

%% Generate system matrixes for linear model
Aeq = gen_aeq(A2,B2,N,mx,mu);             % Generate A, hint: gen_aeq
beq = [x0; zeros(size(Aeq, 1) - mx, 1)];  % Generate b

%% Solve QP problem with fmincon
options = optimoptions('fmincon', 'MaxFunctionEvaluations', 50000);
initialGuess = quadprog(Q, c, [], [], Aeq, beq, vlb, vub);
%initialGuess = zeros(size(Aeq, 2), 1);
tic
%[z,lambda] = quadprog(Q, c, [], [], Aeq, beq, vlb, vub); % hint: quadprog. Type 'doc quadprog' for more info 
z = fmincon(cost_function, initialGuess, [], [], Aeq, beq, vlb, vub, @(x) nonlin_constraints(x, N, mx), options);
t1=toc;

% Calculate objective value
phi1 = 0.0;
PhiOut = zeros(N*mx+M*mu,1);
for i=1:N*mx+M*mu
  phi1=phi1+Q(i,i)*z(i)*z(i);
  PhiOut(i) = phi1;
end

%% Extract control inputs and states
%u = [z(N*mx+1:N*mx+M*mu);z(N*mx+M*mu)]; % Control input from solution
u1 = [z(N*mx+1:mu:N*mx+M*mu); z(N*mx+M*mu)];
u2 = [z(N*mx+2:mu:N*mx+M*mu); z(N*mx+M*mu)];

x1 = [x0(1);z(1:mx:N*mx)];              % State x1 from solution
x2 = [x0(2);z(2:mx:N*mx)];              % State x2 from solution
x3 = [x0(3);z(3:mx:N*mx)];              % State x3 from solution
x4 = [x0(4);z(4:mx:N*mx)];              % State x4 from solution
x5 = [x0(5);z(5:mx:N*mx)];              % State x5 from solution
x6 = [x0(6);z(6:mx:N*mx)];              % State x6 from solution

num_variables = 5/delta_t;
zero_padding = zeros(num_variables,1);
unit_padding  = ones(num_variables,1);

u1   = [zero_padding; u1; zero_padding];
u2   = [zero_padding; u2; zero_padding];
x1  = [x1_0*unit_padding; x1; zero_padding];
x2  = [zero_padding; x2; zero_padding];
x3  = [zero_padding; x3; zero_padding];
x4  = [zero_padding; x4; zero_padding];
x5  = [zero_padding; x5; zero_padding];
x6  = [zero_padding; x6; zero_padding];

%% Plotting
simulation_time = delta_t*(length(u1)-1);
t = 0:delta_t:simulation_time;

figure(2)
subplot(811)
stairs(t,u1),grid
ylabel('Pitch u')
subplot(812)
stairs(t,u2),grid
ylabel('Elevation u')
subplot(813)
plot(t,x1,'m',t,x1,'mo'),grid
ylabel('lambda')
subplot(814)
plot(t,x2,'m',t,x2','mo'),grid
ylabel('r')
subplot(815)
plot(t,x3,'m',t,x3,'mo'),grid
ylabel('p')
subplot(816)
plot(t,x4,'m',t,x4','mo'),grid
xlabel('tid (s)'),ylabel('pdot')
subplot(817)
plot(t,x5,'m',t,x5','mo'),grid
xlabel('tid (s)'),ylabel('e')
subplot(818)
plot(t,x6,'m',t,x6','mo'),grid
xlabel('tid (s)'),ylabel('edot')

fig = gcf;
fig.Position = [500, 000, 1000, 1100];

% Nonlin Constraints
function [c, ceq] = nonlin_constraints(x, N, mx)
    alpha = 0.2;
    beta = 20;
    lambda_t = 2*pi/3;
    e = x(5:mx:N*mx);
    lambda = x(1:mx:N*mx);
    
    c = alpha * exp(-beta * (lambda - lambda_t).^2) - e;
    ceq = [];
end