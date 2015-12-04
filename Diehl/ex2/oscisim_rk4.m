function xN=oscisim_rk4(U)

dt=0.2;
x1=[10;0];
N=51;

assert(numel(U) == N-1);

A=[0 1; -1 0];
B=[0; 1];

f=@(x,u)A*x+B*u;

xN=x1;
for i=1:N-1
    plot(dt*(i-1),xN(1),'bs'); hold on;

    xk_=xN;           k1=f(xk_,U(i));
    xk_=xN+0.5*dt*k1; k2=f(xk_,U(i));
    xk_=xN+0.5*dt*k2; k3=f(xk_,U(i));
    xk_=xN+dt*k3;     k4=f(xk_,U(i));
    
    xN=xN+dt/6*(k1+2*k2+2*k3+k4);


end
