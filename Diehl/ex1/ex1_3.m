clear all; close all; clc;

f = @(x)[0 1]*x; 
Aeq=[1 0];
beq=1;
A=[];
b=[];

x0=[0; 0];

x = fmincon(f,x0,A,b,Aeq,beq,[],[],@c_ex1_3);

%x = fmincon(@(x)norm(x)^2,x0,A,b);

