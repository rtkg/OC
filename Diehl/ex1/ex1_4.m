clear all; close all; clc;

f = @(x)x'*x; 
x0=zeros(50,1);

x = fmincon(f,x0,[],[],[],[],[],[],@c_ex1_4);
t=0:0.2:50*0.2-0.2;
plot(t,x,'r');

%x = fmincon(@(x)norm(x)^2,x0,A,b);

