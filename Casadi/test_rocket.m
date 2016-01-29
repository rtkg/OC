clear all; close all; clc;
import casadi.*;

% Control
u = MX.sym('u');

% State
x = MX.sym('x',3);
s = x(1); % position
v = x(2); % speed
m = x(3); % mass

% ODE right hand side
sdot = v;
vdot = (u - 0.05 * v*v)/m;
mdot = -0.1*u*u;
xdot = vertcat([sdot,vdot,mdot]);

% ODE right hand side function
f = Function('f',{x,u},{xdot});

%  Integrate with Explicit Euler over 0.2 seconds
dt = 0.01; %Time step
xj = x;

for j = 1:20
	[fj] = f({xj,u});
	xj = xj+dt*fj{:}';
end

%  Discrete time dynamics function
F = Function('F',{x,u},{xj});

%  Number of control segments
nu = 50;

% Control for all segments
U = MX.sym('U',nu);

% Initial conditions
X0 = MX([0,0,1]);

% Integrate over all intervals
X=X0;
for k =1:nu
	X_ = F({X,U(k)});
	X=X_{:};
end

%  Objective function and constraints
J = U'*U; % u'*u in Matlab
G = X(1:2); % x(1:2) in Matlab (0:2)?

%  NLP
nlp = struct('x', U, 'f', J, 'g', G);

% Allocate an NLP solver
opts = struct('ipopt',struct('tol',1e-10),'expand',true);
solver = nlpsol('solver', 'ipopt', nlp,opts);

arg = struct;
%Bounds on u and initial condition
arg.x0 = zeros(50,1); % solution guess
arg.lbx = -0.5; % lower bound on x
arg.ubx =  0.5; % upper bound on x

%Bounds on g
arg.lbg = [10,0]; % lower bound on g
arg.ubg = [10,0]; % upper bound on g

% Solve the NLP
res = solver(arg);

f_opt = full(res.f);
u_opt = full(res.x);
lam_u_opt = full(res.lam_x);
lam_g_opt = full(res.lam_g);


