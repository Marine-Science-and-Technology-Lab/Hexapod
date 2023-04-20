function fig_obj = hexapodGraphic_nocalc(hex_obj,fig_obj)
    fig_obj
%     if nargin ~=3 % the first time this function is called, it will great a fig_obj for you
%         fig_obj = struct();
%         fig_obj.ax=uiaxes();
%     end
    pose=hex_obj.pose;
    r0 = pose(1:3); % assuming position is first three elements of pose (x,y,z)
    r=r0+hex_obj.Home;
    E = pose(4:6); % assuming Euler angles are next three elements of pose (phi,theta,psi)
    
    r_rel = hex_obj.r_rel; % need the relative position of the point of interest in platform frame
    base = hex_obj.base; % need the locations of the base joints in world frame
    base_link=hex_obj.base_link;
    plat = hex_obj.plat0; % need the locations of the platform joints in platform frame (assume when E=0, the world and platform frames are aligned)
    plat_link=hex_obj.plat_link_0;
    z_min = hex_obj.z; % need minimum vertical distance for visualization purposes
    L0 = hex_obj.L0; % need minimum link length
    dL = hex_obj.dL; % need link stroke length
    ex=hex_obj.ex;ey=hex_obj.ey;ez=hex_obj.ez;
    px=hex_obj.px;py=hex_obj.py;pz=hex_obj.pz;
    pose_platform=hex_obj.pose_platform;

    % Inverse Kinematics
    
    plat_CM = pose_platform(1:3); % platform CM position resolved in world frame; [m]
    p_W = hex_obj.plati; % platform link joints resolved in a world frame
    l_W=hex_obj.plat_link_i;
    link = zeros(3,6); % each column is a vector describing a link
    q = hex_obj.axisPos; % link lengths; [m]
     Zwat=hex_obj.Z_water; 


    for i = 1:6
        check(i) = q(i) < L0 || q(i) > L0+dL;
    end
    
quiverscale=0.5;
    % Visualization
    if ~isfield(fig_obj,'Plat')
         ax=fig_obj.ax; 
       
        fig_obj.Base=fill3(fig_obj.ax,[base(1,:),base(1,1)],[base(2,:),base(2,1)],[base(3,:),base(3,1)],'k','FaceAlpha',0.25);
       
        hold(ax,'on');
        fig_obj.Plat = fill3(ax,[p_W(1,:),p_W(1,1)],[p_W(2,:),p_W(2,1)],[p_W(3,:),p_W(3,1)],'k','FaceAlpha',0.1);
        fig_obj.B = plot3(ax,plat_CM(1),plat_CM(2),plat_CM(3),'ok');
        fig_obj.C = plot3(ax,r(1),r(2),r(3),'or');
      fig_obj.X=quiver3(ax,0,0,0,ex(1),ex(2),ex(3),quiverscale,'color','b');fig_obj.Y=quiver3(ax,0,0,0,ey(1),ey(2),ey(3),quiverscale,'color','b');fig_obj.Z=quiver3(ax,0,0,0,ez(1),ez(2),ez(3),quiverscale,'color','b');
        fig_obj.x=quiver3(ax,plat_CM(1),plat_CM(2),plat_CM(3),px(1),px(2),px(3),quiverscale,'color','r');fig_obj.y=quiver3(ax,plat_CM(1),plat_CM(2),plat_CM(3),py(1),py(2),py(3),quiverscale,'color','r');fig_obj.z=quiver3(ax,plat_CM(1),plat_CM(2),plat_CM(3),pz(1),pz(2),pz(3),quiverscale,'color','r');
           fig_obj.xm=quiver3(ax,r(1),r(2),r(3),px(1),px(2),px(3),quiverscale,'color','g');fig_obj.ym=quiver3(ax,r(1),r(2),r(3),py(1),py(2),py(3),quiverscale,'color','g');fig_obj.zm=quiver3(ax,r(1),r(2),r(3),pz(1),pz(2),pz(3),quiverscale,'color','g');
fig_obj.wat=patch(ax,[1 -1 -1 1 1],[-1 -1 1 1 -1],Zwat(ones(1,5)),[0.25 0.25 1]);
fig_obj.wat.FaceAlpha=0.25;

        
        fig_obj.axis_lim = [xlim,ylim,zlim];
        fig_obj.axis_lim(end-1:end) = [-2,0.1];
        
%         c = [zeros(6,1), linspace(0,1,6)', ones(6,1)];
c=[0 0 0; 0.5 0.5 0.5; 0 0.5 1; 0 0.5 1; 0 0.5 1; 0 0.5 1];
        for i = 1:6
            if check(i) == 0
                fig_obj.D(i) = plot3(ax,[base_link(1,i),l_W(1,i)],[base_link(2,i),l_W(2,i)],[base_link(3,i),l_W(3,i)],'color',c(i,:),'LineWidth',2);
            else
                fig_obj.D(i) = plot3(ax,[base_link(1,i),l_W(1,i)],[base_link(2,i),l_W(2,i)],[base_link(3,i),l_W(3,i)],'color','r','LineWidth',2);
            end
        end
        axis(ax,'equal');
%         view(ax,40,25);
         hold(ax,'off');
    else
        'woohoo'
        ax=fig_obj.ax;
        delete(fig_obj.Plat); delete(fig_obj.B); delete(fig_obj.C); delete(fig_obj.D); delete(fig_obj.X);delete(fig_obj.Y);delete(fig_obj.Z); delete(fig_obj.x);delete(fig_obj.y);delete(fig_obj.z);delete(fig_obj.xm);delete(fig_obj.ym);delete(fig_obj.zm);
        hold(ax,'on');
        fig_obj.Plat= fill3(ax,[p_W(1,:),p_W(1,1)],[p_W(2,:),p_W(2,1)],[p_W(3,:),p_W(3,1)],'k','FaceAlpha',0.1);
        fig_obj.B = plot3(ax,plat_CM(1),plat_CM(2),plat_CM(3),'ok');
        fig_obj.C = plot3(ax,r(1),r(2),r(3),'or');
          fig_obj.X=quiver3(ax,0,0,0,ex(1),ex(2),ex(3),quiverscale,'color','b');fig_obj.Y=quiver3(ax,0,0,0,ey(1),ey(2),ey(3),quiverscale,'color','b');fig_obj.Z=quiver3(ax,0,0,0,ez(1),ez(2),ez(3),quiverscale,'color','b');
        fig_obj.x=quiver3(ax,plat_CM(1),plat_CM(2),plat_CM(3),px(1),px(2),px(3),quiverscale,'color','r');fig_obj.y=quiver3(ax,plat_CM(1),plat_CM(2),plat_CM(3),py(1),py(2),py(3),quiverscale,'color','r');fig_obj.z=quiver3(ax,plat_CM(1),plat_CM(2),plat_CM(3),pz(1),pz(2),pz(3),quiverscale,'color','r');
      fig_obj.xm=quiver3(ax,r(1),r(2),r(3),px(1),px(2),px(3),quiverscale,'color','g');fig_obj.ym=quiver3(ax,r(1),r(2),r(3),py(1),py(2),py(3),quiverscale,'color','g');fig_obj.zm=quiver3(ax,r(1),r(2),r(3),pz(1),pz(2),pz(3),quiverscale,'color','g');
      
%         c = [zeros(6,1), linspace(0,1,6)', ones(6,1)];
c=[0 0 0; 0.5 0.5 0.5; 0 0.5 1; 0 0.5 1; 0 0.5 1; 0 0.5 1];
        for i = 1:6
            if check(i) == 0
                fig_obj.D(i) = plot3(ax,[base_link(1,i),l_W(1,i)],[base_link(2,i),l_W(2,i)],[base_link(3,i),l_W(3,i)],'color',c(i,:),'LineWidth',2);
            else
                fig_obj.D(i) = plot3(ax,[base_link(1,i),l_W(1,i)],[base_link(2,i),l_W(2,i)],[base_link(3,i),l_W(3,i)],'color','r','LineWidth',2);
            end
        end 
%         view(40,25);
%         axis(ax,fig_obj.axis_lim);
        drawnow;
         hold(ax,'off');
    end

 

end