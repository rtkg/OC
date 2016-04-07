clear all; close all; clc;
addpath('./tools');

inverted_pendulum=InvertedPendulum;
axis([-1.2,1.2,-1.2,1.2]); grid on;
Kp=50;
figure(2);
inverted_pendulum.x_=[0.1; 0];
for i=1:10000
    e=-inverted_pendulum.x_(1);
   inverted_pendulum.u_=Kp*e;
   
   plot(0,e,'rs');
   axis([-0.1,0.1,-5,5]); 
   
	tic
	inverted_pendulum.x_=inverted_pendulum.step;
	t=toc;
	
	pause(inverted_pendulum.dt_-t)
end	