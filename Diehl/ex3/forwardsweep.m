function [x, Atraj, Btraj]=forwardsweep(U)

N=51;
xk=[10;0];
assert(numel(U) == N-1);
for i=1:N-1
	[x(:,i), Atraj(:,:,i), Btraj(:,:,i)]=RK4stepJac(xk,U(i));
	xk=x(:,i);
end	


