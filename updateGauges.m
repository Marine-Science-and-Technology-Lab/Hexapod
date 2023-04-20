function [ActuatorPositions]=updateGauges(hex_obj,ActuatorPositions);

for j=1:6
    ActuatorPositions{j}.Value=hex_obj.axisPos(j);
    ActuatorPositions{j}.Limits=[hex_obj.L0 hex_obj.L0+hex_obj.dL];
end