function hex_obj=EffectorFromPlatformPose(hex_obj);
pose_platform=hex_obj.pose_platform;
r_plat=pose_platform(1:3);

E=pose_platform(4:6);
 R = E2R(E)


r_rel=hex_obj.r_rel;

effector_CM=r_plat+R*r_rel;

e_CM=effector_CM-hex_obj.Home;
hex_obj.pose=[e_CM; E];
