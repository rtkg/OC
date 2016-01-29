clear all; close all; clc;

import casadi.*

nk = 20; %Control discretization
tf = 10.0; %End time

%Declare variables (use scalar graph)
u  = SX.sym('u');    %control
x  = SX.sym('x',2); %states

% ODE right hand side and quadratures
xdot = [(1 - x(2)*x(2))*x(1) - x(2) + u; x(1)];
qdot = x(1)*x(1) + x(2)*x(2) + u*u;

%DAE residual function
dae = struct('x', x, 'p', u, 'ode', xdot, 'quad', qdot);
F = integrator('F', 'cvodes', dae, struct('tf',tf/nk));

% All controls (use matrix graph)
x = MX.sym('x',nk); % nk-by-1 symbolic variable
U = vertsplit(x); % cheaper than x(1), x(2) ...

%The initial state (x_0=0, x_1=1)
X  = MX([0; 1]);

% Objective function
f = 0;

% Build a graph of integrator calls
for k=1:nk
	res = F(struct('x0',X,'p',U(k)));
	X = res.xf;
	f = f+res.qf;
end

%Terminal constraints: x_0(T)=x_1(T)=0
g = X;

%Allocate an NLP solver
nlp = struct('x',x, 'f',f, 'g',g);
solver = nlpsol('solver', 'ipopt', nlp);

%Solve the NLP
arg = struct;
arg.x0 = 0; % solution guess
arg.lbx = -0.75; % lower bound on x
arg.ubx =  1; % upper bound on x
arg.lbg =    0; % lower bound on g
arg.ubg =    0; % upper bound on g
res = solver(arg);

u_opt = full(res.x); 
tgrid_u = linspace(0,10,nk);

plot(tgrid_u, u_opt,'-.'); grid on;



