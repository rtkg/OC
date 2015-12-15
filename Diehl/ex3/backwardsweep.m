function Jsim=backwardsweep(Atraj,Btraj)

N=length(Atraj);
d=size(Atraj,1);

G=eye(d);
Jsim=zeros(d,N);
for k=N:-1:1
	Jsim(:,k)=G*Btraj(:,:,k);
	G=G*Atraj(:,:,k);
end

