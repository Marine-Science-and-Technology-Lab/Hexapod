function [hex_path newpose]=JogFromCurrentPosition(hex_obj,hex_setup,hex_path,jogdist)
% jogdist
jogdist(4:6)=deg2rad(jogdist(4:6));
currentpose=[0;0;0;0;0;0];%hex_obj.pose %Grab current pose of end effector

newpose=currentpose+jogdist %specify new pose

dt=hex_path.dt; %Grab timestep size

Tdur=2; %Specify duration of jog motion
ttemp=[0:dt:Tdur]
for j=1:6
newpose_t(j,:)=squashfunction(ttemp,currentpose(j),newpose(j));
end

% hex_path.pose_t=newpose;
hex_path.T=ttemp;
hex_path.pose_t=newpose_t;
