function xnew=RK4step(x,u)

dt=0.2;
B=[0; 1];
C=180/pi;

f=@(x,u)[x(2);-C*sin(x(1)/C)]+B*u;

k1=f(x,u);
k2=f(x+0.5*dt*k1,u);
k3=f(x+0.5*dt*k2,u);
k4=f(x+dt*k3,u);
	
xnew=x+dt/6*(k1+2*k2+2*k3+k4);

