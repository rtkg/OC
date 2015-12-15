function [xnew, A, B]=RK4stepJac(x,u)

dlt=1e-4;
N=length(x);

xnew=RK4step(x,u);

A=zeros(N,N);
for j=1:N
	ej=zeros(N,1);
	ej(j)=1;
	A(:,j)=(RK4step(x+dlt*ej,u)-xnew)/dlt;
end

B=(RK4step(x,u+dlt)-xnew)/dlt;



