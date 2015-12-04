function J=Jsim(U)

dlt=1e-4;
N=length(U);
U=U(:);

J=zeros(2,N);
for i=1:N
   U_p=U;
   U_p(i)=U_p(i)+dlt;
   J(:,i)=(gsim_nl(U_p)-gsim_nl(U))/dlt;
end    
