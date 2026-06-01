%% Demo for checking collisions
clear all; close all;

load("yoke_meshes.mat") % Load meshed geometries of yokes
N1=length(MSH_A); N2=length(MSH_B);
N3=length(MSH_C); N4=length(MSH_D);

%% Input joint angles
theta=30; %Rotation about first axis (uhat);
phi=20; % Rotation about second axis (vhat);


%% Rotate geometries and check for collision;
Rotn=eul2rotm(deg2rad([theta,phi,0]),'XYZ'); %Rotation matrix
Posn=[0 0 0]; %Translation (set to zero)
TF=[Rotn [0;0;0]; Posn 1]; %Transformation matrix (4x4) for rigid body translation and rotation

for n=1:N2 % Loop through the motor yoke subcomponents and set the pose
    MSH_B(n).Pose=TF;
end

[iscoll mindist witnesspoints]=checkYokeCollision(MSH_A,MSH_B); % Check whether collision has occurred between any of the components and, if not, the distance separating the nearest points.

%% Plot yoke meshes
ff=figure
aa=axes()
hold on

for n=1:N1
    [null ptc(n)]=show(MSH_A(n))
    ptc(n).FaceColor='b';
    ptc(n).EdgeColor=[0 0 0.2];
    ptc(n).FaceAlpha=0.3;
    [null ptc2(n)]=show(MSH_B(n))
    ptc2(n).FaceColor='r';
    ptc2(n).EdgeColor=[0.2 0 0];
    ptc2(n).FaceAlpha=0.3;
    axis equal
    witpt=plot3(witnesspoints(1,:),witnesspoints(2,:),witnesspoints(3,:),'-og','MarkerFaceColor','g','MarkerSize',8,'linewidth',2);
end

aa.View=[  -51.2013   18.2182];
aa.YLim=[  -60 60]
aa.XLim=[  -60 60]
aa.ZLim=[-50 50];
minhist=[];

%Label the distance between nearest points
if ~iscoll
text(witnesspoints(1,1),witnesspoints(2,1),witnesspoints(3,1),['Separation Distance = ' num2str(mindist) 'mm'])
else
    text(0,-30,-40,['Separation Distance = ' num2str(mindist) 'mm'])
end