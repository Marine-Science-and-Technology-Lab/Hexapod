function [hex_obj hex_setup]=InitializeHexapodObject()

%% Create Hexapod structure and initialize values
the_dr = 20*(pi/180); % angular distance between pairs of links (1&2, 3&4, 5&6); [rad]
the_roff = (2*pi - 3*the_dr)/3; % angular distance between off pairs of links (2&3, 4&5, 6&1); [rad]
base_r = 0.8; % radius of the base links; [m]
plat_r = 0.4; % radius of the platform links; [m]
hex_obj.dL = 0.4394; % maximum link (linear actuator) stroke length; [m]
hex_obj.L0 = 0.769813+0.04936; % minimum link (linear actuator) length; [m] Includes new spindle length

% --- Three-frame motion model (see Docs/APP_DESIGNER_WIRING.md and the
% "Datum / POI frames" section of UPGRADE_PLAN.md).
%
%   World (W)        base frame. Hexapod geometry (base, plat0, base_link,
%                    plat_link_0) lives here.
%   Platform (P)     rigid body of the hexapod platform. Rotates + translates
%                    as motion commands execute.
%   POI (Q)          point of interest attached to the platform - the frame
%                    motion commands describe. Default: coincident with
%                    platform body origin (identity transform). User sets
%                    T_platform_POI to describe a mounted test article's
%                    offset + orientation relative to the platform plate.
%   Datum (D)        motion-planning origin. At pose = 0 the POI sits at
%                    the datum origin. The datum frame is derived from
%                    T_world_datum_platform (where the platform sits at
%                    pose = 0) composed with T_platform_POI.
%
% Motion command: hex_obj.pose = [x; y; z; roll; pitch; yaw] is
% interpreted as T_datum_POI - the POI's pose relative to the datum.
% Pose rotation axes are the datum frame's axes (= POI axes at pose = 0).

hex_obj.Home_platform = [0; 0; -0.966446+0.000250667]; % Immutable: physical platform-CM world position after re-homing.

hex_obj.T_world_datum_platform = struct('R', eye(3), 't', hex_obj.Home_platform); % Platform-CM world pose when motion command pose = 0. User-settable via "Set Datum Here" / "Reset Datum to Home".
hex_obj.T_platform_POI         = struct('R', eye(3), 't', [0; 0; 0]);              % POI pose relative to platform body. User-settable via Coordinate System tab.

hex_obj.Cam = [0; 0; 0];    % Camera position in world frame (Basler checkerboard tracking).
hex_obj.pose          = zeros(6, 1);   % T_datum_POI expressed as [x; y; z; roll; pitch; yaw].
hex_obj.pose_platform = zeros(6, 1);   % T_world_platform expressed the same way (populated by IK).
% Link joints on base in base-fixed frame (base-fixed frame = world frame)
hex_obj.Base_Zlink=0.0471297; % Height to axis of U joint yoke on platform
hex_obj.Platform_Zlink=0.0407797; %0.04564246; Height to axis of U joint yoke on base
b_W = zeros(3,1); % origin of base-fixed frame and world frame
the_b_r1 = 0+the_dr/2+the_roff; % angular position of the first link; [rad]

hex_obj.base = zeros(3,6);
hex_obj.base(:,1) = [base_r*cos(the_b_r1); base_r*sin(the_b_r1); 0];
the_temp = the_b_r1;

for i = 2:6
    the_temp = the_temp + mod(i+1,2)*the_dr + mod(i,2)*the_roff;
    hex_obj.base(:,i) = [base_r*cos(the_temp); base_r*sin(the_temp); 0]; % assuming no vertical displacement of the links relative to base frame origin
end

% Link joints on platform in platform-fixed frame

the_p_r1 = (pi/3)+the_dr/2+the_roff; % angular position of the first link; [rad]

hex_obj.plat0 = zeros(3,6);
hex_obj.plat0(:,1) = [plat_r*cos(the_p_r1); plat_r*sin(the_p_r1); 0];
the_temp = the_p_r1;

for i = 2:6
    the_temp = the_temp + mod(i+1,2)*the_dr + mod(i,2)*the_roff;
    hex_obj.plat0(:,i) = [plat_r*cos(the_temp); plat_r*sin(the_temp); 0]; % assuming no vertical displacement of the links relative to platform frame origin
end
hex_obj.plat0 = [hex_obj.plat0(:,end),hex_obj.plat0(:,1:end-1)]; % reordering links on platform so the appropriate joints on the base are connected to the appropriate joints on the platform
hex_obj.plati=hex_obj.plat0;


% Locations of linkage attachment points, accounting for offset in UJoint
% Yokes
hex_obj.plat_link_0 = hex_obj.plat0 + [0; 0; hex_obj.Platform_Zlink];
hex_obj.base_link   = hex_obj.base  - [0; 0; hex_obj.Base_Zlink];
hex_obj.plat_link_i = hex_obj.plati + [0; 0; hex_obj.Platform_Zlink];


hex_obj.z=-1.5;

hex_obj.axisPos=[0;0;0;0;0;0];




%% Initialize hexapod setup/parameters
hex_setup.Motors.Kt=0.331; %N-m/A
hex_setup.Motors.RatedTorque=3.18; %N-m
hex_setup.Motors.MaxTorque=9.54; %N-m
hex_setup.Motors.MaxRPM=5500; %Rev per minute (max) 
hex_setup.Motors.RatedRPM=3000; %Rev per minute (cont rated)
hex_setup.Motors.RatedCurrent=10; %Ampere 
hex_setup.Motors.MaxCurrent=15; %Ampere (Limited by circuit breakers)
hex_setup.Motors.CountsPerRev=2500*4; %Encoder counts per revolution
hex_setup.Motors.RatedPower=1000; %Watts
hex_setup.Actuators.LeadScrewPitch=-0.02; %Lead screw pitch in m
hex_setup.Actuators.CountsPerM=hex_setup.Motors.CountsPerRev/hex_setup.Actuators.LeadScrewPitch;
hex_setup.Actuators.MaxSpeed=820/1000; %m/s
hex_setup.Actuators.MaxThrust=2224; %mm/s
hex_setup.Actuators.MaxAcceleration=9.810; %m/s^2
hex_setup.Actuators.MinLength=hex_obj.L0;
hex_setup.Actuators.MaxLength=hex_obj.L0+hex_obj.dL;
hex_setup.Actuators.DatumLength=1.0397604; % Distance between U-joint yokes at home position (AVERAGED)
hex_setup.Actuators.DatumLength_Individual=[1.039726542
1.039688442
1.039777342
1.039650342
1.039777342
1.039802742];

hex_setup.UhatRotations=[];
hex_setup.YokeA_Rotations=[50 -170 170 -50 -70 70]; %Rotation angles of yoke A in world frame
hex_setup.YokeA.Uhat=[50 -170 170 -50 -70 70];

hex_obj.axisCt=(hex_obj.axisPos-hex_setup.Actuators.DatumLength)*hex_setup.Actuators.CountsPerM;
hex_obj.axisRel=hex_obj.axisPos-hex_setup.Actuators.DatumLength;

% Mass properties of the platform (about platform CM)
hex_setup.Platform.Mass=40; 
hex_setup.Platform.Ixx=10; 
hex_setup.Platform.Iyy=10; 
hex_setup.Platform.Izz=10; 

% Mass properties of the model/end effector (in end effector frame)
hex_setup.Effector.Mass=40; 
hex_setup.Effector.Ixx=10; 
hex_setup.Effector.Iyy=10; 
hex_setup.Effector.Izz=10; 

%External forces, specified in end-effector frame
hex_setup.F_ex.Fx=0;
hex_setup.F_ex.Fy=0;
hex_setup.F_ex.Fz=0;
hex_setup.F_ex.Mx=0;
hex_setup.F_ex.My=0;
hex_setup.F_ex.Mz=0;

hex_obj.Z_water=-(46-4+6)*0.0254; % Location of water surface in platform frame


hex_setup.collisionthreshold=0.5; % minimum allowable separation distance between Ujoint yokes
