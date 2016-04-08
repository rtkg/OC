clear all; close all; clc;

segway=Segway; %create a 2D segway instance
axis([-3,3,-1.5,1.5]); pbaspect([3 1.5 1]); grid on; %adjust the visualization settings

segway.x_=[0; 0; 0.5; 0]; %set the initial state (x=[x; dx; theta; dtheta])
segway.dt_= 1e-2;   %set the sampling rate

while(1) %simulate for 1000 steps
	segway.u_ = 0; %set the control input at the current time step
	
	tic;
	segway.x_=segway.step; %integrate forward according to x_new=f(x,u,dt) and update the state vector
	t=toc;
	
	pause(segway.dt_-t); %a crude way of making the visualization appear in real-time
end