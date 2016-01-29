clear all; close all; clc;
import casadi.*;

a=0.02;
g0=9.81;
d=20;
T=3;
N=40;
px0=0; py0=1.5;
vx0=5; vy0=5;
x0=[px0,vx0,py0,vy0]';

px = SX.sym('px'); vx = SX.sym('vx');
py = SX.sym('py'); vy = SX.sym('vy');
x = [px , vx , py , vy ]';
px_dot = vx; vx_dot=-a*vx*sqrt(vx^2+vy^2);
py_dot = vy; vy_dot=-a*vy*sqrt(vx^2+vy^2)-g0;

x_dot = [px_dot,vx_dot,py_dot,vy_dot]';

ode = struct('x', x, 'ode', x_dot);
F = integrator('F', 'cvodes', ode, struct('tf',T/N));