function fig_obj = hexapodGraphic(hex_obj,fig_obj)
    
    if nargin ~=2 % the first time this function is called, it will great a fig_obj for you
        fig_obj = struct();
        fig_obj.ax=uiaxes();
    end
    pose=hex_obj.pose;
    r0 = pose(1:3); % assuming position is first three elements of pose (x,y,z)
    r=r0+hex_obj.Home;
    E = pose(4:6); % assuming Euler angles are next three elements of pose (phi,theta,psi)
    
    r_rel = hex_obj.r_rel; % need the relative position of the point of interest in platform frame
    base = hex_obj.base; % need the locations of the base joints in world frame
    plat = hex_obj.plat; % need the locations of the platform joints in platform frame (assume when E=0, the world and platform frames are aligned)
    z_min = hex_obj.z; % need minimum vertical distance for visualization purposes
    L0 = hex_obj.L0; % need minimum link length
    dL = hex_obj.dL; % need link stroke length
    


    % Inverse Kinematics
    R = E2R(E); % convert Euler angles to rotation matrix
    plat_CM = r - R*r_rel; % platform CM position resolved in world frame; [m]
    
    link = zeros(3,6); % each column is a vector describing a link
    p_W = zeros(3,6); % platform link joints resolved in a world frame
    q = zeros(6,1); % link lengths; [m]
    l_hat = zeros(3,6); % unit vectors describing longitudinal axis of links (base to platform) resolved in world frame
    for i = 1:6
        p_W(:,i) = plat_CM + R*plat(:,i);
        link(:,i) = p_W(:,i) - base(:,i);
        q(i) = sqrt(link(:,i)'*link(:,i));
        l_hat(:,i) = link(:,i)./q(i);
        check(i) = q(i) < L0 || q(i) > L0+dL;
    end
    

    % Visualization
    if ~isfield(fig_obj,'A')
        ax=fig_obj.ax;
        hold(ax,'on');
        fig_obj.Base=fill3(ax,[base(1,:),base(1,1)],[base(2,:),base(2,1)],[base(3,:),base(3,1)],'k','FaceAlpha',0.25);
        fig_obj.A = fill3(ax,[p_W(1,:),p_W(1,1)],[p_W(2,:),p_W(2,1)],[p_W(3,:),p_W(3,1)],'k','FaceAlpha',0.1);
        fig_obj.B = plot3(ax,plat_CM(1),plat_CM(2),plat_CM(3),'ok');
        fig_obj.C = plot3(ax,r(1),r(2),r(3),'or');
        fig_obj.X=quiver3(0,0,0,ex(1),ex(2),ex(3),'color','b');fig_obj.Y=quiver3(0,0,0,ey(1),ey(2),ey(3),'color','b');fig_obj.Z=quiver3(0,0,0,ez(1),ez(2),ez(3),'color','b');
        fig_obj.x=quiver3(plat_CM(1),plat_CM(2),plat_CM(3),px(1),px(2),px(3),'color','r');fig_obj.y=quiver3(plat_CM(1),plat_CM(2),plat_CM(3),py(1),py(2),py(3),'color','r');fig_obj.z=quiver3(plat_CM(1),plat_CM(2),plat_CM(3),pz(1),pz(2),pz(3),'color','r');
        fig_obj.axis_lim = [xlim,ylim,zlim];
        fig_obj.axis_lim(end-1:end) = [-2,0.1];
        
        c = [zeros(6,1), linspace(0,1,6)', ones(6,1)];
        for i = 1:6
            if check(i) == 0
                fig_obj.D(i) = plot3(ax,[base(1,i),base(1,i)+link(1,i)],[base(2,i),base(2,i)+link(2,i)],[base(3,i),base(3,i)+link(3,i)],'color',c(i,:),'LineWidth',2);
            else
                fig_obj.D(i) = plot3(ax,[base(1,i),base(1,i)+link(1,i)],[base(2,i),base(2,i)+link(2,i)],[base(3,i),base(3,i)+link(3,i)],'color','r','LineWidth',2);
            end
        end
        axis(ax,'equal');
        view(ax,40,25);
         hold(ax,'off');
    else
        ax=fig_obj.ax;
        delete(fig_obj.A); delete(fig_obj.B); delete(fig_obj.C); delete(fig_obj.D);
        hold(ax,'on');
        fig_obj.A = fill3(ax,[p_W(1,:),p_W(1,1)],[p_W(2,:),p_W(2,1)],[p_W(3,:),p_W(3,1)],'k','FaceAlpha',0.1);
        fig_obj.B = plot3(ax,plat_CM(1),plat_CM(2),plat_CM(3),'ok');
        fig_obj.C = plot3(ax,r(1),r(2),r(3),'or');
        c = [zeros(6,1), linspace(0,1,6)', ones(6,1)];
        for i = 1:6
            if check(i) == 0
                fig_obj.D(i) = plot3(ax,[base(1,i),base(1,i)+link(1,i)],[base(2,i),base(2,i)+link(2,i)],[base(3,i),base(3,i)+link(3,i)],'color',c(i,:),'LineWidth',2);
            else
                fig_obj.D(i) = plot3(ax,[base(1,i),base(1,i)+link(1,i)],[base(2,i),base(2,i)+link(2,i)],[base(3,i),base(3,i)+link(3,i)],'color','r','LineWidth',2);
            end
        end 
%         view(40,25);
%         axis(ax,fig_obj.axis_lim);
        drawnow;
         hold(ax,'off');
    end

 

end