clear all; close all; clc;
addpath('./tools');

inverted_pendulum=InvertedPendulum;
axis([-1.2,1.2,-1.2,1.2]); grid on;


inverted_pendulum.x_=[0; 0.1];
for i=1:10000
	tic
	inverted_pendulum.x_=inverted_pendulum.step;
	t=toc;
	
	pause(inverted_pendulum.dt_-t)
end	