function U=gaussNewton_full_BFGS(U0)

eps=1e-5;
N=length(U0);
U_k=U0;
iter=0;
B_k=eye(N);
lmbd_k=zeros(2,1);
n_u_L=@(u,lmbd)2*u+Jsim(u)'*lmbd;
while 1
	kktres_k=norm([2*U_k+Jsim(U_k)'*lmbd_k; gsim_nl(U_k)]);
	plot(iter,log10(kktres_k),'rd');hold on; grid on;
	if(kktres_k < eps)
		break;
	end
		
	x_k1=[U_k; zeros(2,1)] - pinv([B_k Jsim(U_k)'; Jsim(U_k) zeros(2,2)])*[2*U_k; gsim_nl(U_k)];
	U_k1=x_k1(1:N);
	lmbd_k1=x_k1(N+1:N+2);
	
	%%% Update B_k here !!!!!!
	s_k=U_k1-U_k;
	y_k=n_u_L(U_k1,lmbd_k1)-n_u_L(U_k,lmbd_k1);
	
	
	B_k1=B_k-(B_k*s_k*s_k'*B_k)/(s_k'*B_k*s_k)+(y_k*y_k')/(s_k'*y_k);
	
	
	B_k=B_k1;
	U_k=U_k1;
	lmbd_k=lmbd_k1;
	iter=iter+1;
end

disp(['solved in ' num2str(iter) ' iterations.']);
U=U_k;