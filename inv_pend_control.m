clear all; close all; clc;
addpath('./tools');

inverted_pendulum=InvertedPendulum;
axis([-1.2,1.2,-1.2,1.2]); grid on;

inverted_pendulum.x_=[3; 0];

A=[0 1; inverted_pendulum.g_/inverted_pendulum.l_ 0];
B=[0; inverted_pendulum.c_];
P=[-10; -2];

K=acker(A,B,P);

for i=1:10000
    inverted_pendulum.u_=-K*inverted_pendulum.x_;
      inverted_pendulum.u_
	tic
	inverted_pendulum.x_=inverted_pendulum.step;
	t=toc;
	
	pause(inverted_pendulum.dt_-t)
end	