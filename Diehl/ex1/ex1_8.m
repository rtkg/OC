clear all; close all; clc;

f = @(x)x'*x;
x0=zeros(50,1);
umax=2;
lb=ones(50,1)*-umax;
ub=ones(50,1)*umax;


x = fmincon(f,x0,[],[],[],[],lb,ub,@c_ex1_4);
t=0:0.2:50*0.2-0.2;
plot(t,x,'r');

%x = fmincon(@(x)norm(x)^2,x0,A,b);

