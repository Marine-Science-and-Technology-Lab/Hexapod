% [fname fpath]=uigetfile('*.py')
% 
addpath('C:\Program Files (x86)\Galil\gclib\source\wrappers\python\')

g=py.gclib.py


buff=511;

DT=2;

dt=2^DT/1000;

Tf=20;

tt=[0:dt:Tf]';

A=[0.03 0.03 0.05];
ctperm=500000;
Act=A*ctperm;
f=[0.75 0.75 0.5];
phi=[0 90];

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

g.GOpen('192.168.1.2')
ss=g.GInfo;
ss.string;
g.GCommand('AB')
g.GCommand('DPA=0')
g.GCommand('SHA')
g.GCommand('WT100')

TargetBuff=250;
TargetTime=TargetBuff*dt;
N=length(ydiff);

n1=1;
n2=500;

g.GCommand('CMA')
g.GCommand('DT 2')
n=1
while n<N;
    
    g.GCommand(['CD ' num2str(ydiff(n))]);

    if ~mod(n,100)
                LS=g.GCommand('TSA');
        LS=fliplr(dec2bin(str2num(LS.string),8));
        HS=~str2num(LS(2));
        RLS=~str2num(LS(3));
        FLS=~str2num(LS(4));
        
exitflag=0;
if or(RLS,FLS)
    n=N;
    exitflag=1;
    'LIMIT SWITCH'
end
if(~exitflag)
        'Checking Buffer'
        cont=0
        while ~cont
        buffsize=g.GCommand('CM?');
            cont=str2num(buffsize.string)>TargetBuff;
        end
end

    end
n=n+1;
end

g.GClose