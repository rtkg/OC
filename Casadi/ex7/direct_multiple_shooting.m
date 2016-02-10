clear all; close all; clc;
import casadi.*;

N = 20;      %Control discretization
T = 10.0;    %End time

u  = SX.sym('u');    %control
x  = SX.sym('x',2);  %states

%System dynamics
xdot =  [(1 - x(2)^2)*x(1) - x(2) + u; x(1)];
qdot = x(1)^2 + x(2)^2 + u^2;

ode = struct('x', x, 'p', u, 'ode', xdot, 'quad', qdot);
F = integrator('F', 'cvodes', ode, struct('tf',T/N));

%Formulate NLP (use matrix graph)
nv = 1*N + 2*(N+1);
v = MX.sym('v', nv);

xk=[];
for k=1:N+1
  xk=[xk; v(3*k-2:3*k-1)];
end

uk=[];
for k=1:N
  uk=[uk; v(3*k)];	
end	

%Variable bounds and initial guess
vmin=-inf*ones(nv,1); vmin(3:3:end)=-1; vmin(1)=0; vmin(2)=1; vmin(end)=0; vmin(end-1)=0;
vmax=inf*ones(nv,1); vmax(3:3:end)=1;  vmax(1)=0; vmax(2)=1; vmax(end)=0; vmax(end-1)=0;
x0=zeros(nv,1); x0(2)=1;
gmin=zeros(2*N,1);
gmax=zeros(2*N,1);

% Objective function
J = 0;

% Build a graph of integrator calls
g=[];
for k=1:N
	res = F(struct('x0',xk(2*k-1:2*k),'p',uk(k)));
	xf = res.xf;
	qf=res.qf;
	J=J+qf;
	g=[g; xf-xk(2*(k+1)-1:2*(k+1))]; 
end

%Allocate an NLP solver
nlp = struct('x',v, 'f',J, 'g',g);
opts = struct('ipopt',struct('tol',1e-10,'print_level',5,'linear_solver','mumps'));
solver = nlpsol('solver', 'ipopt', nlp,opts);

%Solve the NLP
arg = struct;
arg.x0 = x0; % solution guess
arg.lbx = vmin; % lower bound on x
arg.ubx =  vmax; % upper bound on x
arg.lbg =  gmin % lower bound on g
arg.ubg =  gmax; % upper bound on g
res = solver(arg);
% 
opt = full(res.x); 
x1_opt=opt(1:3:end);
x2_opt=opt(2:3:end);
u_opt=opt(3:3:end);
tgrid_u = linspace(0,10,N);
tgrid_x = linspace(0,10+10/N,N+1);

stairs(tgrid_u, u_opt,'r'); grid on; hold on;
plot(tgrid_x, x1_opt,'b'); 
plot(tgrid_x, x2_opt,'g'); 