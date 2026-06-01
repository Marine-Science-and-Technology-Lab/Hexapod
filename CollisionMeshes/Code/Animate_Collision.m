N_downsample=10;

handles.UserData.AnimFig=figure
handles.UserData.ModelDown=patch(fv_WD,'FaceColor',       [0.8 0.8 1.0], ...
        'FaceColor', [1 1 0] , ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
handles.UserData.ModelUp=patch(fv_WU,'FaceColor',       [0.8 0.8 1.0], ...
        'FaceColor', [1 1 0] , ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
     camlight('headlight');
material('dull');


% Fix the axes scaling, and set a nice view angle
axis('image');

view([-135 35]);
handles.UserData.AnimAxis=gca;
hold on
% Beachsurf=patch(BeachMesh)

Water=patch('XData',SWL.X,'YData',SWL.Y,'ZData',SWL.Z,'facecolor','b','facealpha',0.3);
 BeachLowerSection=patch('XData',Beach.Lower.X,'YData',Beach.Lower.Y,'ZData',Beach.Lower.Z,'facecolor',[0.5 0.5 0.5]);
            BeachUpperSection=patch('XData',Beach.Upper.X,'YData',Beach.Upper.Y,'ZData',Beach.Upper.Z,'facecolor',[0.5 0.5 0.5]);
            

            Pos=[Trial_TS.Position_Gapfilled.X.Data Trial_TS.Position_Gapfilled.Y.Data Trial_TS.Position_Gapfilled.Z.Data];
            RPY=[-Trial_TS.Orientation_Gapfilled.Roll.Data  Trial_TS.Orientation_Gapfilled.Pitch.Data Trial_TS.Orientation_Gapfilled.Yaw.Data];
            
nt=1;

Posn=Pos(nt,:);
Rotn=eul2rotm(deg2rad(RPY(nt,:)),'XYZ')
Quatn=eul2quat(deg2rad(RPY(nt,:)),'XYZ')

Tf=rigid3d(Rotn,Posn)

fvpts_down=fv_WD.vertices;
fvpts_up=fv_WU.vertices;
fv_n=Tf.transformPointsForward(fvpts_down);


ModelDown.Vertices=fv_n;
drawnow


N_T=length(Pos);

dt=1/(Trial_TS.Position_Gapfilled.X.UserData.SamplingInfo.Sample_Rate)

WS=Trial_TS.ManeuveringInputs.WheelState.Data;

handles.UserData.Pos=Pos;
handles.UserData.RPY=RPY;
handles.UserData.fvpts_down=fvpts_down;
handles.UserData.fvpts_up=fvpts_up;
handles.TS=Trial_TS;
handles.UserData.dt=dt;
handles.UserData.MSH_WU=MSH_WU;
handles.UserData.MSH_WD=MSH_WD;
handles.UserData.BeachMesh=BeachMesh;
handles.UserData.WS=WS;
% MshBnd=plot3(BoundV.WD(:,1),BoundV.WD(:,2),BoundV.WD(:,3),'ok');
% MshBnd_WU=plot3(BoundV.WU(:,1),BoundV.WU(:,2),BoundV.WU(:,3),'ok');

for nt=1:N_downsample:N_T
[handles]=UpdateMQSAnim(handles,nt);
end

% for nt=1:N_downsample:N_T
%     if(WS(nt))
%         fvpts=fvpts_up;
%         Model=ModelUp;
%         ModelDown.Visible='off'
%         ModelUp.Visible='on'
%         BndPts=BoundV.WU;
%         MSH=MSH_WU;
%     else
%         fvpts=fvpts_down;
%         Model=ModelDown;
%         ModelDown.Visible='on'
%         ModelUp.Visible='off'
%         BndPts=BoundV.WD;
%         MSH=MSH_WD;
%     end
%     
% Posn=Pos(nt,:);
% Rotn=eul2rotm(deg2rad(YPR(nt,:)),'XYZ');
% Quatn=eul2quat(deg2rad(YPR(nt,:)),'XYZ');
% 
% Tf=rigid3d(Rotn,Posn);
%    fv_n=Tf.transformPointsForward(fvpts);
%    
% %    BndPts_n=Tf.transformPointsForward(BndPts);
%    
% %    MshBnd.XData=BndPts_n(:,1); MshBnd.YData=BndPts_n(:,2);MshBnd.ZData=BndPts_n(:,3);
%       
%    Model.Vertices=fv_n;
%    
%  
%    
%    MSH.Pose=Tf.T';
%    
% [collisionStatus,sepdist,witnesspts] = checkCollision(MSH,BeachMesh);
%   
% if collisionStatus;
%     Model.FaceColor=[1 0 0];
% else
%     Model.FaceColor=[1 1 0];
% end
% drawnow;
%    
% %    pause(N_downsample*dt);
%     
% end