%% Generating Sin wave
% OE 3,3,3,3,3,3 causes abort to happen

% maybe clear the last values for ydiff
close all

DT=2;
dt=2^DT/1000;
Tf=20;
tt=[0:dt:Tf]';
A=[0.1 0.1 0.1 0.1 0.1 0.1];

ctperm=500000;
Act=A*ctperm;
f=[0.5 0.5 0.5 0.5 0.5 0.5];
phi=[0 0 0 0 0 0];


T_ramp=1;
N_ramp=T_ramp/dt;
T_env=ones(size(tt));
T_env(1:N_ramp)=linspace(0,1,N_ramp);
T_env(end-N_ramp+1:end)=linspace(1,0,N_ramp);

 
for n_ind=1:size(A,2)
    yy(:,n_ind)=Act(n_ind)*sin(tt*2*pi*f(n_ind)+phi(n_ind))%+5000*sin(0.25*tt*2*pi+pi/2)+2500*sin(10*tt);
    yy(:,n_ind)=yy(:,n_ind).*T_env;
end
ydiff=diff(yy);
ydiff=round(ydiff);
ff=figure
plot(tt,yy)
waitfor(ff)


%% Connecting to the controller via python wrapper
g=py.gclib.py
g.GOpen('192.168.42.2')
ss=g.GInfo;
ss.string;

%% Contouring mode
contourABCEFG(g,ydiff)

g.GClose