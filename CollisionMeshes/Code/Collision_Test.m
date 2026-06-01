% [fname_down fpath]=uigetfile('*.stl','Select Wheels-Down')
% [fname_up fpath]=uigetfile('*.stl','Select Wheels-Up')
% MQSbod_WD=rigidBody('Hull')
% addVisual(MQSbod_WD,"Mesh",[fpath fname_down])
% MQSbod_WU=rigidBody('Hull')
% addVisual(MQSbod_WU,"Mesh",[fpath fname_up])
% Robot=rigidBodyTree()
% addBody(Robot,MQSbod,'base')

% gm=importGeometry([fpath fname])

[fv_WD]=stlread('./imgs/Ph2_SimplifiedMesh_WD.stl');
[fv_WU]=stlread('./imgs/Ph2_SimplifiedMesh_WU.stl');

fv_WD.vertices=fv_WD.vertices*diag([1 -1 -1]); %Roll model by 180 to conform to basin coordinate system
fv_WU.vertices=fv_WU.vertices*diag([1 -1 -1]); %Roll model by 180 to conform to basin coordinate system

figure
subplot(2,1,1)
patch(fv_WD,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');

% Fix the axes scaling, and set a nice view angle
axis('image');

view([-135 35]);

V_WD=fv_WD.vertices;

[K,av]=convhull(V_WD(:,1),V_WD(:,2),V_WD(:,3),'Simplify',true);

Vred=V_WD(K(:),:);
Vred=unique(Vred,'rows');
hold on
plot3(V_WD(:,1),V_WD(:,2),V_WD(:,3),'ok')
plot3(Vred(:,1),Vred(:,2),Vred(:,3),'or')

BoundV.WD=Vred;
MSH_WD=collisionMesh(Vred);

hold on
mshp=show(MSH_WD)
mshp.Children(1).FaceAlpha=0.5

% addCollision(MQSbod_WD,MSH_WD)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,1,2)
patch(fv_WU,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');

% Fix the axes scaling, and set a nice view angle
axis('image');

view([-135 35]);

V_WU=fv_WU.vertices;


[K,av]=convhull(V_WU(:,1),V_WU(:,2),V_WU(:,3),'Simplify',true)

Vred=V_WU(K(:),:);
Vred=unique(Vred,'rows');
BoundV.WU=Vred;
hold on
plot3(V_WU(:,1),V_WU(:,2),V_WU(:,3),'ok')
plot3(Vred(:,1),Vred(:,2),Vred(:,3),'or')

MSH_WU=collisionMesh(Vred);


hold on
mshp=show(MSH_WU)
mshp.Children(1).FaceAlpha=0.5

% addCollision(MQSbod_WU,MSH_WU)


%% Beach Collsion object
X.WestWall=-3633;
            X.EastWall=36453;
            X.BeachTop=-3633;
            X.Knuckle=6080-3633;
            X.End=6080+7180-3633;

            Y.Northwall=10000;
            Y.Southwall=-10000;

            Z.BeachTop=452.4;
            Z.Knuckle=-298.7;
            Z.End=-1681.8;
            Z.Bottom=-2935.8;
            Z.walltop=1333.8;

            %% Create patch node coordinates for each wall
            % Walls
            Walls.West.X=X.WestWall*ones(4,1);
            Walls.West.Y=[Y.Northwall Y.Northwall Y.Southwall Y.Southwall]';
            Walls.West.Z=[Z.Bottom Z.walltop Z.walltop Z.Bottom]';

            Walls.East.X=X.EastWall*ones(4,1);
            Walls.East.Y=Walls.West.Y;
            Walls.East.Z=Walls.West.Z;


            Walls.North.X=[X.WestWall X.WestWall X.EastWall X.EastWall]';
            Walls.North.Y=Y.Northwall*ones(4,1);
            Walls.North.Z=Walls.West.Z;

            Walls.South.X=Walls.North.X;
            Walls.South.Y=Y.Southwall*ones(4,1);
            Walls.South.Z=Walls.North.Z;

            % Basin floor
            Bottom.X=Walls.South.X;
            Bottom.Z=Z.Bottom*ones(4,1);
            Bottom.Y=[Y.Northwall Y.Southwall Y.Southwall Y.Northwall]';

            % Beach panels
            Beach.Upper.X=[X.WestWall X.WestWall X.Knuckle X.Knuckle]';
            Beach.Upper.Y=[Y.Northwall Y.Southwall Y.Southwall Y.Northwall]';
            Beach.Upper.Z=[Z.BeachTop Z.BeachTop Z.Knuckle Z.Knuckle]';

            Beach.Lower.X=[X.Knuckle X.Knuckle X.End X.End]';
            Beach.Lower.Y=Beach.Upper.Y;
            Beach.Lower.Z=[Z.Knuckle Z.Knuckle Z.End Z.End]';

            % Still waterline
            SWL.X=Bottom.X;
            SWL.Y=Bottom.Y;
            SWL.Z=zeros(4,1);


%             BeachBody=rigidBody('BeachSurface')

BeachVerts=[Beach.Lower.X Beach.Lower.Y Beach.Lower.Z;
            Beach.Upper.X Beach.Upper.Y Beach.Upper.Z]

BeachVerts=unique(BeachVerts,'rows');

BeachMesh=collisionMesh(BeachVerts)
figure
show(BeachMesh)