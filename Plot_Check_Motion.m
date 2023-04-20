function [fig Check Pass]=Plot_Check_Motion(hex_path,hex_setup)

WithinLengthLimits=min(and(hex_path.axis_t>hex_setup.Actuators.MinLength,hex_path.axis_t<hex_setup.Actuators.MaxLength));
WithinVelLimits=min(abs(hex_path.axis_dt)<hex_setup.Actuators.MaxSpeed);
WithinAccLimits=min(abs(hex_path.axis_ddt)<hex_setup.Actuators.MaxAcceleration);

WithinForceLimits=min(abs(hex_path.F_links<hex_setup.Actuators.MaxThrust));
WithinRPMLimits=min(abs(hex_path.MotorRPM<hex_setup.Motors.MaxRPM));
WithinTorqueLimits=min(abs(hex_path.MotorTorques<hex_setup.Motors.MaxTorque));
WithinCurrentLimits=min(abs(hex_path.MotorCurrent<hex_setup.Motors.MaxCurrent));

Check.LinkLength=(WithinLengthLimits);
Check.LinkVelocity=(WithinVelLimits);
Check.LinkAcceleration=(WithinAccLimits);
Check.LinkForce=(WithinForceLimits);
Check.MotorRPM=(WithinRPMLimits);
Check.MotorTorque=(WithinTorqueLimits);
Check.CurrentDraw=(WithinCurrentLimits);

RMStorque=rms(hex_path.MotorTorques(:));

T=hex_path.T;

fig.f=figure;
fig.f.Name='Kinematics';
set(fig.f,'Color','w')
        set(fig.f,'Units','pixels')
        fig.f.Position=[0 0 1920 1080]

fig.a(1)=subplot(3,6,[1:2]);
plot(T,hex_path.pose_t(1:3,:),'LineWidth',1.5);
ylabel('m');
xlabel('time, s');
legend({'X','Y','Z'});
title('Platform Translations');

fig.a(2)=subplot(3,6,[3:4]);
plot(T,hex_path.pose_t(4:6,:)*180/pi,'LineWidth',1.5);
ylabel('deg');
xlabel('time, s');
legend({'R_X','R_Y','R_Z'});
title('Platform Rotations');


fig.a(3)=subplot(3,6,[5]);
plot(T,hex_path.FT_platform(1:3,:));
ylabel('N');
xlabel('time, s');
legend({'F_X','F_Y','F_Z'});
title('Forces at Platform CG');

fig.a(4)=subplot(3,6,[6]);
plot(T,hex_path.FT_platform(4:6,:));
ylabel('N-m');
xlabel('time, s');
legend({'M_X','M_Y','M_Z'});
title('Reaction Moments at Platform CG');

fig.a(5)=subplot(3,6,[7 8]);
plot(T,hex_path.axis_t);
hold on
plot(xlim,hex_setup.Actuators.MinLength*ones(2,1),'--k','LineWidth',2);
plot(xlim,(hex_setup.Actuators.MaxLength)*ones(2,1),'--k','LineWidth',2);
xlabel('time, s');
ylabel('m');
title('Link Lengths');


fig.a(6)=subplot(3,6,[9 10]);
plot(T,hex_path.axis_dt);
hold on;
plot(xlim,hex_setup.Actuators.MaxSpeed*ones(2,1),'--k','LineWidth',2);
plot(xlim,-hex_setup.Actuators.MaxSpeed*ones(2,1),'--k','LineWidth',2);
xlabel('time, s');
ylabel('m/s');
title('Link Velocities');

fig.a(7)=subplot(3,6,[11 12]);
plot(T,hex_path.axis_ddt);
ylabel('m/s^2');
xlabel('time, s');
title('Link Accelerations');
hold on;
plot(xlim,hex_setup.Actuators.MaxAcceleration*ones(2,1),'--k','LineWidth',2);
plot(xlim,-hex_setup.Actuators.MaxAcceleration*ones(2,1),'--k','LineWidth',2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% fig.f2=figure;
% fig.f2.Name='Dynamics';
fig.a(8)=subplot(3,6,[13:14]);
plot(T,hex_path.F_links);
hold on
plot(xlim,hex_setup.Actuators.MaxThrust*ones(2,1),'--k','LineWidth',2);
plot(xlim,-hex_setup.Actuators.MaxThrust*ones(2,1),'--k','LineWidth',2);
xlabel('time,s');
ylabel('Newtons');
title('Approximate Actuator Forces');

set(gcf,'units','inches','position',[0 0 4.5 4.5],'color','w');
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman');
set(findall(gcf,'-property','FontSize'),'FontSize',12);



fig.a(9)=subplot(3,6,[15:16]);
title('Motor Speeds');
plot(T,hex_path.MotorRPM);
hold on
plot(xlim,hex_setup.Motors.RatedRPM*ones(2,1),'--k','LineWidth',1.5);
plot(xlim,-hex_setup.Motors.RatedRPM*ones(2,1),'--k','LineWidth',1.5);
plot(xlim,hex_setup.Motors.MaxRPM*ones(2,1),'--r','LineWidth',3);
plot(xlim,-hex_setup.Motors.MaxRPM*ones(2,1),'--r','LineWidth',3);
xlabel('time,s');
ylabel('Motor Rev Speed (RPM)');
% title('Motor speed and torque')

% 
% Torquelimit=3.18
% TorquelimitMax=9.5



fig.a(10)=subplot(3,6,[17]);
plot(T,hex_path.MotorTorques);
hold on
plot(xlim,hex_setup.Motors.RatedTorque*ones(2,1),'--k','LineWidth',1.5);
plot(xlim,-hex_setup.Motors.RatedTorque*ones(2,1),'--k','LineWidth',1.5);
plot(xlim,hex_setup.Motors.MaxTorque*ones(2,1),'--r','LineWidth',3);
plot(xlim,-hex_setup.Motors.MaxTorque*ones(2,1),'--r','LineWidth',3);
plot(xlim,RMStorque*ones(2,1),'-.b','LineWidth',2);
% text(0,-Torquelimit,['Rated Torque = ' num2str(Torquelimit,2) ' N-m'],'VerticalAlignment','Top','HorizontalAlignment','Left','Color','k')
% text(0,TorquelimitMax,['Max Torque = ' num2str(TorquelimitMax,2) ' N-m'],'VerticalAlignment','Top','HorizontalAlignment','Left','Color','r')
% text(0,TorquelimitMax-2,['RMS Torque = ' num2str(RMStorque,2) ' N-m'],'VerticalAlignment','Top','HorizontalAlignment','Left','Color','b')
xlabel('time,s');
ylabel('N-m');
% title('Motor speed and torque')
title('Motor Torque');

% RMStorque=rms(hex_path.MotorTorques(:));

fig.a(11)=subplot(3,6,[18]);
plot(T,abs(hex_path.MotorCurrent));
hold on
plot(xlim,hex_setup.Motors.RatedCurrent*ones(2,1),'--k','LineWidth',1.5);
% plot(xlim,-hex_setup.Motors.RatedCurrent*ones(2,1),'--k','LineWidth',1.5)
plot(xlim,hex_setup.Motors.MaxCurrent*ones(2,1),'--r','LineWidth',3);
% plot(xlim,-hex_setup.Motors.MaxCurrent*ones(2,1),'--r','LineWidth',3)
% plot(xlim,RMStorque*ones(2,1),'-.b','LineWidth',2)
% text(0,-Torquelimit,['Rated Torque = ' num2str(Torquelimit,2) ' N-m'],'VerticalAlignment','Top','HorizontalAlignment','Left','Color','k')
% text(0,TorquelimitMax,['Max Torque = ' num2str(TorquelimitMax,2) ' N-m'],'VerticalAlignment','Top','HorizontalAlignment','Left','Color','r')
% text(0,TorquelimitMax-2,['RMS Torque = ' num2str(RMStorque,2) ' N-m'],'VerticalAlignment','Top','HorizontalAlignment','Left','Color','b')
xlabel('time,s');
ylabel('Amps');
% title('Motor speed and torque')
title('Motor Current');
;

linkaxes([fig.a(:)],'x');


Fchecks=fieldnames(Check);
Nchecks=length(Fchecks);
Nax=length(fig.a);
Pass=1;
for nch=1:Nchecks
if ~min(Check.(Fchecks{nch}));
Pass=0;
    ind_edges=FindEdges(~Check.(Fchecks{nch}));
    N_violations=size(ind_edges,1);

    
    for nax=1:Nax
   if and(nch==1,nax==1);
       handlevis='on';
   else
       handlevis='off';
   end
    
    axes(fig.a(nax));
    hold on
    yl=ylim;
        for n_v=1:N_violations
    patch(T([ind_edges(n_v,:), fliplr(ind_edges(n_v,:))]),yl([1 1 2 2]),'r','EdgeColor','none','FaceAlpha',0.2,'DisplayName','Limit(s) Exceeded','HandleVisibility',handlevis);
        end
    ylim(yl);
    end
end
end

set(gcf,'units','normalized','position',[0 0 0.8 0.8],'color','w');
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman');
set(findall(gcf,'-property','FontSize'),'FontSize',10);
