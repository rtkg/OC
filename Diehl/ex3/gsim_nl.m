function xN=gsim_nl(U)

dt=0.2;
x1=[10;0];
N=51;

assert(numel(U) == N-1);

B=[0; 1];
C=180/pi;
f=@(x,u)[x(2);-C*sin(x(1)/C)]+B*u;

xN=x1;
for i=1:N-1
	%plot(dt*(i-1),xN(1),'bs'); hold on;
	
	xk_=xN;           k1=f(xk_,U(i));
	xk_=xN+0.5*dt*k1; k2=f(xk_,U(i));
	xk_=xN+0.5*dt*k2; k3=f(xk_,U(i));
	xk_=xN+dt*k3;     k4=f(xk_,U(i));
	
	xN=xN+dt/6*(k1+2*k2+2*k3+k4);
end
