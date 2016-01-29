clear all; close all;
addpath(genpath('../'));

import casadi.*

% Control
u=MX.sym('u',2);
v=u(1);
w=u(2);

% State
s = MX.sym('s',3);
x = s(1);
y = s(2);
th = s(3);


% ODE right hand side
xdot = v*cos(th);
ydot = v*sin(th);
thdot = w;
sdot = vertcat([xdot,ydot,thdot]);
%
% ODE right hand side function
f = Function('f',{s,u},{sdot});

%Integrate with Explicit Euler over 0.2 seconds
dt=0.01;
sj = s;
for j=1:20
	fj=f({sj,u});
	sj = sj+dt*fj{:}';
end

% Discrete time dynamics function
F = Function('F',{s,u},{sj});

% Number of control segments
nu = 50

% Control for all segments
U = MX.sym('U',nu);

%  Initial conditions
S0 = MX([-1,-1,0]);

% Integrate over all intervals
S=S0;
for k =1:nu
	S_ = F({S,U(k)});
	S=S_{:};
end

%  Objective function and constraints
J = U'*U;
G = S(1:2);

%  NLP
nlp = struct('x', U, 'f', J, 'g', G);

% Allocate an NLP solver
opts = struct('ipopt',struct('tol',1e-10),'expand',true);
solver = nlpsol('solver', 'ipopt', nlp,opts);


% Solve the NLP
arg = struct;
% arg.x0 = [2.5 3.0 0.75]; % solution guess
% arg.lbx = -inf; % lower bound on x
% arg.ubx =  inf; % upper bound on x
arg.lbg =    0; % lower bound on g
arg.ubg =    0; % upper bound on g
res = solver(arg);


f_opt = full(res.f);        
u_opt = full(res.x);        
lam_u_opt = full(res.lam_x);
lam_g_opt = full(res.lam_g);


%
% # Bounds on u and initial condition
% arg["lbx"] = -0.5
% arg["ubx"] =  0.5
% arg["x0"] =   0.4
%
% # Bounds on g
% arg["lbg"] = [10,0]
% arg["ubg"] = [10,0]
%
% # Solve the problem
% res = solver(arg)
%
% # Get the solution
% plot(res["x"])
% plot(res["lam_x"])
% grid()
% show()
