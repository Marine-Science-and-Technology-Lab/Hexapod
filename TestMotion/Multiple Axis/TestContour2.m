%% Generating Sin wave
% OE 3,3,3,3,3,3 causes abort to happen

% maybe clear the last values for ydiff
% clear all
% close all
if ~exist('g')
    connect_to_galil
end

clearvars -except g

DT=2;
dt=2^DT/1000;
Tf=20;

[fname fpath]=uigetfile("*.mat")
load([fpath fname])
yy=round(yy)
ydiff=diff(yy);
% ydiff=round(ydiff);
ff=figure
plot(tt,yy)
waitfor(ff)


%% Connecting to the controller via python wrapper
% g=py.gclib.py
% g.GOpen('192.168.42.2')
% ss=g.GInfo;
% ss.string;

%% Contouring mode
contourABCEFG(g,ydiff)

% g.GClose