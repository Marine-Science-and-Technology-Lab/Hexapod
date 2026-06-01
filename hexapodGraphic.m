function fig_obj = hexapodGraphic(hex_obj,fig_obj)
    
    if nargin ~=2 % the first time this function is called, it will great a fig_obj for you
        fig_obj = struct();
        fig_obj.ax=uiaxes();
    end
    % Three-frame chain (matches InverseKinematics_hexapod.m):
    T_datum_POI  = poseToTransform(hex_obj.pose);
    T_WD         = composeTransform(hex_obj.T_world_datum_platform, hex_obj.T_platform_POI);
    T_WQ         = composeTransform(T_WD, T_datum_POI);
    T_WP         = composeTransform(T_WQ, invertTransform(hex_obj.T_platform_POI));
    r       = T_WQ.t;    % POI world position (for end-effector marker)
    plat_CM = T_WP.t;    % platform CM world position
    R       = T_WP.R;    % platform rotation in world

    base = hex_obj.base;
    plat = hex_obj.plat;
    z_min = hex_obj.z;
    L0 = hex_obj.L0;
    dL = hex_obj.dL;
    
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