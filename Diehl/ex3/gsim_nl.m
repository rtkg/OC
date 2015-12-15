function xN=gsim_nl(U)

dt=0.2;
x1=[10;0];
N=51;

assert(numel(U) == N-1);

xN=x1;
for i=1:N-1
	%plot(i*dt-dt,xN(1),'bs'); grid on; hold on;
	xN=RK4step(xN,U(i));
end
