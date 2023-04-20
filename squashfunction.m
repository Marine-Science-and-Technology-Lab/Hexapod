function y = squashfunction(t,llim,ulim);

tnorm=linspace(-pi,pi,length(t));
tfunc=0.5*(tanh(tnorm)+1);
tfunc=(tfunc-min(tfunc))*1/range(tfunc);
y=llim+tfunc*(ulim-llim);
