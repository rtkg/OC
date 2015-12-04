function xN=oscisim_euler(U)

dt=0.2;
x1=[10;0];
N=51;

assert(numel(U) == N-1);

A=[0 1; -1 0];
B=[0; 1];

xN=x1;
for i=1:N-1
    plot(dt*(i-1),xN(1),'bs'); hold on;
    xN=xN+dt*(A*xN+B*U(i));
end
