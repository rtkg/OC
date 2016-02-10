clear all; close all; clc;
import casadi.*;

a=0.02;
g0=9.81;
d=20;
T=3;
N=20;
px0=0; py0=1.5;
vx0=5; vy0=5;
x0=[px0,vx0,py0,vy0]';
pxF = 20.0;
pyF = 0.0;

px = SX.sym('px'); vx = SX.sym('vx');
py = SX.sym('py'); vy = SX.sym('vy');
x = [px , vx , py , vy ]';
px_dot = vx; vx_dot=-a*vx*sqrt(vx^2+vy^2);
py_dot = vy; vy_dot=-a*vy*sqrt(vx^2+vy^2)-g0;

x_dot = [px_dot,vx_dot,py_dot,vy_dot]';

ode = struct('x', x, 'ode', x_dot);
F = integrator('F', 'cvodes', ode, struct('tf',T/N));

v0=MX.sym('v0',2,1);
X=[x0(1); v0(1); x0(3); v0(2)];
for k=1:N
	res = F(struct('x0',X));
	X = res.xf;
end
pf=[X(1); X(3)];

nlp = struct('x',v0, 'f',0, 'g',pf-[pxF; pyF]);
opts = struct('ipopt',struct('max_iter',30));
solver = nlpsol('solver', 'ipopt', nlp);

arg = struct;
arg.x0 = [5; 5]; % solution guess
arg.lbg =    0; % lower bound on g
arg.ubg =    0; % upper bound on g
res = solver(arg);
full(res.x)


F=Function('F',{v0},{pf-[pxF; pyF]});
G=F.jacobian();
vk=[5;5];
tol=1e-7;
for k=1:1000
	res=G({vk});
	J=full(res{1});
	f=full(res{2});
	
	vk1=vk-pinv(J)*f;
	if norm(vk1-vk)<tol
		break
	end
	vk=vk1;
end
vk






