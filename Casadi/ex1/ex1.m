clear all; close all; clc;

import casadi.*

N = 40;
mi = 40.0/N;
Di = 70.0*N;
g0 = 9.81;


%Hessian
H = zeros(2*N, 2*N);
for k=1:2*N-2
	H(k+2, k) = -1.0;
	H(k, k+2) = -1.0;
end
for k=1:2*N
	H(k,k)=2.0;
end
H(1,1)=1; H(2,2)=1; H(end,end)=1; H(end-1,end-1)=1;
H=H*Di;

%linear objective
g = zeros(1,2*N);
g(2:2:end)=g0*mi;

%box constraints
lbx=-Inf*ones(2*N,1); lbx(1)=-2; lbx(2)=1; lbx(end-1)=2; lbx(end)=1;
ubx=inf*ones(2*N,1); ubx(1)=-2; ubx(2)=1; ubx(end-1)=2; ubx(end)=1;
lbg=0.5*ones(N,1);
ubg=inf*ones(N,1);


%create qp
x=SX.sym('x',2*N,1);
f=0.5*x'*H*x+g*x;
c=x(2:2:end)-x(1:2:end)*0.1;
qp=struct('x',x,'f',f,'g',c);
solver = qpsol('solver','gurobi',qp);

%solve the qp
arg=struct;
arg.lbx=lbx;
arg.ubx=ubx;
arg.lbg=lbg;
arg.ubg=ubg;

tic
res = solver(arg);
t=toc

x_opt=full(res.x);
yi=x_opt(1:2:end);
zi=x_opt(2:2:end);

plot(yi,zi,'b.'); grid on; axis equal;


