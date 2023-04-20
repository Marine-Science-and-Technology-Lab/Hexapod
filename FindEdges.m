function [ind_edge]=FindEdges(BinY);
Edge_N=1;

dY=diff(BinY);
N_rise=length(find(dY==1));
N_fall=length(find(dY==-1));


if BinY(1)==1
    dY(1)=1;
    N_rise=N_rise+1;
end

if BinY(end)==1;
    dY(end)=-1;
N_fall=N_fall+1;
end

rise=find(dY==1);
fall=find(dY==-1);

N_trans=max(N_rise,N_fall);

for N_pulse=1:N_trans;
ind_edge(N_pulse,1)=rise(N_pulse);
ind_edge(N_pulse,2)=fall(N_pulse);
end
