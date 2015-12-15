function U=gaussNewton_full(U0)

eps=1e-5;
N=length(U0);
U_k=U0;
iter=0;
while 1
	iter=iter+1;
	B_k=2*eye(N);
	
	x_k1=[U_k; zeros(2,1)] - pinv([B_k Jsim(U_k)'; Jsim(U_k) zeros(2,2)])*[2*U_k; gsim_nl(U_k)];
	U_k1=x_k1(1:N);
	lmbd_k1=x_k1(N+1:N+2);
	
	dU_k=norm(U_k-U_k1);
	kktres_k=norm([2*U_k1+Jsim(U_k1)'*lmbd_k1; gsim_nl(U_k1)]);
	%plot(iter,log10(kktres_k),'rd');hold on; grid on;
	
	%	if (dU_k < eps)
	if(kktres_k < eps)
		break;
	end
	U_k=U_k1;
end

disp(['solved in ' num2str(iter) ' iterations.']);
U=U_k;