clear all; close all;

% [fname fpath]=uigetfile('*.png')

load('Basler_Params.mat');

% img=imread([fpath fname]);
cam = ImaqScript;
% cam=webcam();
prevfig=figure;
 img=getsnapshot(cam);
   II=imshow(img);

prevfig.Position=[2   562   958   434];
extfig=figure;
extfig.Position=[962    42   958   954];

% t_wait=0.1;

run=1;

refrectY=[-62.5 62.5 62.5 -62.5 -62.5]';
refrectX=[-100 -100 100 100 -100]';
refrectZ=[0 0 0 0 0]';

pp=patch(refrectX,refrectY,refrectZ,'r')
pp.EdgeColor='k';
axis equal
 grid on
 
 xlim([-300 300])
 ylim([-200 200])
 zlim([-1000 -700])
 view([15 50])
 camproj('perspective')
while run==1
     img=getsnapshot(cam);
   II.CData=img; 
    
   try
    [imagePoints, boardSize] = detectCheckerboardPoints(img);
    squareSize = 25;  % in units of 'millimeters'
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    [rotationMatrix,translationVector] = extrinsics(imagePoints,worldPoints,cameraParams);
   hold on
    plot(worldPoints(:,1),worldPoints(:,2),'rx')
    hold off
    rotform=rotm2tform(rotationMatrix);
%     tform=rigid3d(rotationMatrix,translationVector);   
%     [X Y Z]=tform.transformPointsInverse(refrectX,refrectY,refrectZ)
[XYZ]=rotationMatrix*[refrectX,refrectY,refrectZ]';

X=XYZ(1,:)+translationVector(1); Y=XYZ(2,:)-translationVector(2); Z=XYZ(3,:)-translationVector(3);

 figure(extfig)
 pp.XData=X; pp.YData=Y; pp.ZData=Z;
 drawnow
 
 
 rpy=rad2deg(rotm2eul(rotationMatrix));
 
%  sprintf(['X = %3.1f \t Y = %3.1f \t Z = %3.1f mm \n Rx = %3.1f \t Ry = %3.1f \t Rz = %3.1f',translationVector(1),translationVector(2),translationVector(3),RPY(1),RPY(2),RPY(3))
 sprintf('X = %3.1f mm \t\t Y = %3.1f  mm \t Z = %3.1f mm \n Rx = %3.1f deg \t Ry = %3.1f deg \t Rz = %3.1f deg',translationVector(1),translationVector(2),translationVector(3),rpy(3),rpy(2),rpy(1))

   catch
%        run=0
   end
% pause(t_wait);
% run=0;
end
    

