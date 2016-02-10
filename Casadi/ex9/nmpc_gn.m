clear all; close all; clc;
import casadi.*;

T = 10.0;    %End time
N = 20;      %Control discretization
M = 10;      %RK discretization

u  = SX.sym('u');    %control
x  = SX.sym('x',2);  %states

%System dynamics
xdot =  [(1 - x(2)^2)*x(1) - x(2) + u; x(1)];

f=Function('f',{x,u},{xdot});

% RK4 with M steps
U = MX.sym('U');
X0 = MX.sym('X0',2);
DT = T/(N*M);
XF = X0;
QF = 0;
R_terms = [];  %Terms in the Gauss-Newton objective
for j =1:M
	res=f({XF,             U});
	k1=res{1};
	res= f({XF + DT/2 * k1, U});
	k2=res{1};
	res= f({XF + DT/2 * k2, U});
	k3=res{1};
	res=f({XF + DT   * k3, U});
	k4=res{1};
	XF = XF+ DT/6*(k1   + 2*k2   + 2*k3   + k4);
	R_terms=[R_terms; XF; U];
end
F = Function('F',{X0,U},{XF,R_terms});

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

%NLP constraints
g = [];

% Terms in the Gauss-Newton objective
R = [];

% Build a graph of integrator calls
for k=1:N
	res = F({xk(2*k-1:2*k),uk(k)});
	x_next_k = res{1};
	R_terms=res{2};
	g=[g; x_next_k-xk(2*(k+1)-1:2*(k+1))];
	R=[R; R_terms];
end

% Objective function
obj = R'*R/2;


%Allocate an NLP solver
nlp = struct('x',v, 'f',obj, 'g',g);
opts = struct('ipopt',struct('linear_solver','ma27'));
solver = nlpsol('solver', 'ipopt', nlp,opts);

arg = struct;
% All constraints are equality constraints in this case
arg.lbg = 0; % lower bound on g
arg.ubg =  0; % upper bound on g

%Construct and populate the vectors with upper and lower simple bounds on u
vmin=-inf*ones(nv,1); vmin(3:3:end)=-1;
vmax=inf*ones(nv,1); vmax(3:3:end)=1;

x_current=[1;0];
v_k=zeros(nv,1);
t=0;
while(true)
	vmin(1:2)=x_current;
	vmax(1:2)=x_current;
	arg.x0 = v_k; % solution guess
	arg.lbx = vmin; % lower bound on x
	arg.ubx =  vmax; % upper bound on x
	
	res = solver(arg);
	u_nmpc=full(res.x(3)); %control
	
	plot(t,x_current(1),'g.'); grid on; hold on;
	plot(t,x_current(2),'b.');
	plot(t,u_nmpc,'r.');
	
	%potentially read in different x_current to mimic a disturbance
% 	if t> 6
% 		x_current=[0.5;0.5];
% 	end
	
	
	%Simulate the system with this control
	output=F({x_current,u_nmpc});
	
	% Update the current state
	x_current = full(output{1});
	t = t+ T/N;
	
	%Shift the time to have a better initial guess for the next time horizon
	%   w_k["X",:-1] = sol["X",1:] (all except the first element -> 20 X)
	%   w_k["U",:-1] = sol["U",1:] (all except the first element -> 19 u)
	%   w_k["X",-1] = sol["X",-1] (last element of X)
	%   w_k["U",-1] = sol["U",-1] (last element of u)
	v_k(1:end-3)=full(res.x(4:end));
	v_k(end-2:end)=full(res.x(end-2:end));
	
    if t > 25
		break
	end
end

