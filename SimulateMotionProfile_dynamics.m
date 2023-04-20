function [hex_path]=SimulateMotionProfile_dynamics(hex_obj,hex_setup,hex_path,includeFex)

Time=hex_path.T;
dt=hex_path.dt;
r_rel=hex_obj.r_rel;

I_platform=diag([hex_setup.Platform.Ixx hex_setup.Platform.Iyy hex_setup.Platform.Izz]);
I_model0=diag([hex_setup.Effector.Ixx hex_setup.Effector.Iyy hex_setup.Effector.Izz]);
I_model_p=I_model0+hex_setup.Effector.Mass*((r_rel'*r_rel)*eye(3)-r_rel*r_rel');
I_total=I_platform+I_model_p;
M_total=hex_setup.Effector.Mass+hex_setup.Platform.Mass;
pose_t=hex_path.pose_t;
pose_ddt=hex_path.pose_ddt;
r_ddt=pose_ddt(1:3,:);
E_ddt=pose_ddt(4:6,:);

N_t=length(Time);

F_inertial=M_total*(r_ddt+repmat([0;0;9.81],1,length(r_ddt)));
if includeFex
F_ext=[hex_setup.F_ex.Fx;hex_setup.F_ex.Fy;hex_setup.F_ex.Fz];
else
    F_ext=[0;0;0];
end
F_total=F_inertial+repmat(F_ext,1,size(F_inertial,2));

T_ext=cross(r_rel,F_ext);

for nt=1:N_t;
    T_inertial(:,nt)=I_total*E_ddt(:,nt);
end

T_total=T_inertial+repmat(T_ext,1,size(T_inertial,2));

L_total=[F_total;T_inertial];


for nt=1:N_t
    p_W=hex_path.plati(:,:,nt);
        T_L=[hex_path.lhat(:,:,nt); zeros(3,6)];
l_hat_temp=hex_path.lhat(:,:,nt);
r=pose_t(1:3,:);
for i=1:6
Delta(:,i)=r(:,nt)-p_W(:,i);
T_L([4:6],i)=[l_hat_temp([2 1 1],i).*[-Delta(3,i);Delta(3,i);-Delta(2,i)]+l_hat_temp([3 3 2],i).*[Delta(2,i);-Delta(1,i);Delta(1,i)]];
end

F_links(:,nt)=-T_L\L_total(:,nt);

end

hex_path.F_links=F_links;
hex_path.FT_platform=L_total;

hex_path.MotorTorques=hex_path.F_links/250;
hex_path.MotorCurrent=hex_path.MotorTorques/hex_setup.Motors.Kt;




hex_path.MotorRPM=hex_path.axis_dt/hex_setup.Actuators.LeadScrewPitch*60;