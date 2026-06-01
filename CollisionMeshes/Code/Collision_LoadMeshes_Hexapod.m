

load("yoke_meshes.mat")

N1=length(MSH_B); N2=length(MSH_M);

ff=figure
aa=subplot(4,1,[1:3]);
aa2=subplot(4,1,4);
axes(aa)
hold on

for n=1:N1  
[null ptc(n)]=show(MSH_B(n))
ptc(n).FaceColor='b';
ptc(n).EdgeColor=[0 0 0.5];
ptc(n).FaceAlpha=0.3;
[null ptc2(n)]=show(MSH_M(n))
ptc2(n).FaceColor='r';
ptc2(n).EdgeColor=[0.5 0 0];
ptc2(n).FaceAlpha=0.3;
axis equal
witpt=plot3(0,0,0);
% plot3(witnesspoints(1,:),witnesspoints(2,:),witnesspoints(3,:),'ok','MarkerFaceColor','g','MarkerSize',8)
end
aa.View=[  -51.2013   18.2182];
aa.YLim=[  -60 60]
aa.XLim=[  -60 60]
aa.ZLim=[-50 50];
minhist=[];

set(gcf,'units','inches','position',[0 0 10 8],'color','w');

%Define video file name
            [vidfile, vidpath] = uiputfile('*.mp4','Save Movie File As...');
            save_file = strcat(vidpath,vidfile);

            Define video object
            vid_obj = VideoWriter(save_file,'MPEG-4');
            vid_obj.FrameRate = 30;
            open(vid_obj);
            
N_ang=300; % Number of angles to evaluate

% Create timeseries of angles for U joint
theta_vec=deg2rad(linspace(-20,35,N_ang/2));
theta_vec=-[theta_vec theta_vec(end)*ones(1,N_ang/2)];
phi_vec=deg2rad(linspace(0,45,N_ang/2));
phi_vec=[zeros(1,N_ang/2) phi_vec];

%Plot angles. Wait to continue code until figure is closed.
ftemp=figure 
plot([1:length(theta_vec)]',[theta_vec',phi_vec'])
waitfor(ftemp)

% Loop through angles

for n_a=1:N_ang
axes(aa)
theta=theta_vec(n_a);
phi=phi_vec(n_a);
Rotn=eul2rotm([theta,phi,0],'XYZ'); %Rotation matrix
Posn=[0 0 0]; %Tranlation

 TF=[Rotn [0;0;0]; Posn 1];%rigid3d(Rotn',Posn);


 %Ease plot objects that cannot be re-used.
delete(ptc2); %Moving yoke
delete(witpt); %Nearest points

% Re-plot new position of yoke
for n=1:N2
    MSH_M(n).Pose=TF;
    [null ptc2(n)]=show(MSH_M(n));
ptc2(n).FaceColor='r';
ptc2(n).EdgeColor=[0.5 0 0];
ptc2(n).FaceAlpha=0.3;
end

%Check for collision
[iscoll mindist witnesspoints]=checkYokeCollision(MSH_B,MSH_M);

if ~iscoll
    minhist=[minhist mindist];
    witpt=plot3(witnesspoints(1,:),witnesspoints(2,:),witnesspoints(3,:),'-og','MarkerFaceColor','g','MarkerSize',8,'linewidth',2);
else
    minhist=[minhist 0];
    witpt=plot3(nan,nan,nan)
end

plot(aa2,minhist,'ok')
ylabel(aa2,'Minimum Distance (mm)');

drawnow %Update frame

 vid_frame = getframe(ff);
            writeVideo(vid_obj,vid_frame);

end

close(vid_obj);
