clear all; close all; clc;
import casadi.*;

x=SX.sym('x',2);
u=SX.sym('u',1);

xdot=[0 1;-1 0]*x+[0;1]*u;
x0=10;
T=10;

ode = struct('x', x, 'p', u, 'ode', xdot);
F = integrator('F', 'cvodes', ode, struct('tf',T));

res=F(struct('x0',x0,'p',0));

xf=full(res.xf);