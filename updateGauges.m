function [ActuatorPositions]=updateGauges(hex_obj,ActuatorPositions);

for j=1:6
    ActuatorPositions{j}.Value=hex_obj.axisPos(j);
    ActuatorPositions{j}.Limits=[hex_obj.L0 hex_obj.L0+hex_obj.dL];
end

for j=7:12
    ActuatorPositions{j}.Value=hex_obj.axisCt(j-6);
end


for j=13:15
    ActuatorPositions{j}.Value=hex_obj.pose(j-12);
end

for j=16:18
    ActuatorPositions{j}.Value=rad2deg(hex_obj.pose(j-12));
end