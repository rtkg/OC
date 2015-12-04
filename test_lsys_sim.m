clear all; close all; clc;
import casadi.*;

%%%%%%%%%%%%%%%%% system definition %%%%%%%%%%%%%%%%%%%%
p=[-10; -0.5];
A=[ 0    1;
    p(1) p(2)];
b=[0;
   1];
c=[1 0];
d=0;

%%%%%%%% initial values and controls %%%%%%%%%%%%%%%%%%%
x0=[1; 0];
tgrid=linspace(0,10,100)';
u=rand(size(tgrid))*10;

%%%%%%%%%%%%%%%%%%%%%% simulation %%%%%%%%%%%%%%%%%%%%%%
sys=ss(A,b,c,d);
[y,t,x]=lsim(sys,u,tgrid,x0);

%%%%%%%%%%%%%%%%%%%%%% symbolic description %%%%%%%%%%%%%%%%%%%%%%
x_=SX.sym('x',2,1);
p_=SX.sym('p',2,1);
A_=SX.sym('A',2,2);
u_=SX.sym('u');

A_(1,1)=0; A_(1,2)=1; A_(2,1)=p_(1); A_(2,2)=p_(2);
dx_=A_*x_+b*u_;

f = SXFunction(controldaeIn('x',x_,'p',p_,'u',u_),daeOut('ode',dx_));
f.setOption('name','f');
f.init();

%%%%%%%%%%%%%%%%%%%%%% control simulator %%%%%%%%%%%%%%%%%%%%%%
csim = ControlSimulator(f,tgrid);
csim.setOption('integrator', 'cvodes');
csim.init();

csim.setInput(x0,'x0');
csim.setInput(p,'p');
csim.setInput(u(1:end-1),'u');

csim.evaluate();
output = full(csim.getOutput());
%%%%%%%%%%%%%%%%%%%%%% plotting  %%%%%%%%%%%%%%%%%%%%%%
subplot(1,2,1);
plot(tgrid,x); grid on;
title('via lsim');
legend('q','dq');
subplot(1,2,2);
plot(tgrid,output); grid on;
title('via CASADI ControlSimulator');
legend('q','dq');