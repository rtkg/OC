clear all; close all; clc;

inverted_pendulum=InvertedPendulum; %create an inverted pendulum instance
axis([-1.2,1.2,-1.2,1.2]); grid on; %adjust the visualization settings

inverted_pendulum.x_=[0.5; 0]; %set the initial state (x=[theta; dtheta])
inverted_pendulum.dt_= 1e-2;   %set the sampling rate

for i=1:1000 %simulate for 1000 steps
	inverted_pendulum.u_ = 0; %set the control input at the current time step
	
	tic;
	inverted_pendulum.x_=inverted_pendulum.step; %integrate forward according to x_new=f(x,u,dt) and update the state vector
	t=toc;
	
	pause(inverted_pendulum.dt_-t); %a crude way of making the visualization appear in real-time
end