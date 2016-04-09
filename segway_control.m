clear all; close all; clc;

segway=Segway; %create a 2D segway instance
axis([-3,3,-1.5,1.5]); pbaspect([3 1.5 1]); grid on; %adjust the visualization settings

segway.x_=[0; 0; 0.5; 0]; %set the initial state (x=[x; dx; theta; dtheta])
segway.dt_= 1e-2;   %set the sampling rate

g=segway.g_;
l=segway.l_;
m=segway.m_;
M=segway.M_;
dt=segway.dt_;

A=[0 1 0 0; 0 0 m/M*g 0; 0 0 0 1; 0 0 g/l*(M+m)/M 0];
B=[0; 1/M; 0; 1/m/l];
P=[-1; -1; -5; -5];
K=acker(A,B,P);

r=zeros(4,1);
for i=1:10000 %simulate for 10000 steps
	r(1)=sin(i*dt);
	
	segway.u_=-K*(segway.x_-r);
	
	tic;
	segway.x_=segway.step; %integrate forward according to x_new=f(x,u,dt) and update the state vector
	t=toc;
	
	pause(segway.dt_-t); %a crude way of making the visualization appear in real-time
end