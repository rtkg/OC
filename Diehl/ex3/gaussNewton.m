function U=gaussNewton(U0)

eps=1e-5;
N=length(U0);
U_k=U0;
iter=0;
while 1
	iter=iter+1;
	U_k1=U_k-[eye(N) zeros(N,2)]*pinv([2*eye(N) Jsim(U_k)'; Jsim(U_k) zeros(2,2)])*[2*U_k; gsim_nl(U_k)];
	
	if (norm(U_k-U_k1) < eps)
		break;
	end
	U_k=U_k1;
end

disp(['solved in ' num2str(iter) ' iterations.']);
U=U_k;