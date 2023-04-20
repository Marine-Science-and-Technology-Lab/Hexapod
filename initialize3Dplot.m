function fig_obj=initialize3Dplot(fig_obj)  
ax=fig_obj.ax;
hold(ax);
fig_obj.A = fill3(ax,[1:6]',[1:6]',zeros(6,1),'k','FaceAlpha',0.1);
        fig_obj.B = plot3(ax,0,0,0,'ok');
        fig_obj.C = plot3(ax,0,0,0,'or');
        for i=1:6
        fig_obj.D(i) = plot3(ax,[0 1],[0 1],[0 1],'LineWidth',2);
        end 
        fig_obj.Base=fill3(ax,[1:6]',[1:6]',zeros(6,1),'k','FaceAlpha',0.25);
        
       