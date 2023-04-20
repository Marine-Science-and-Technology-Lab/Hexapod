function [rotm] = AA2R(ax,ang)
    
    % converts axis angle representation to a rotation matrix

    AX = skew(ax);
    rotm = eye(3) + sin(ang)*AX + (1-cos(ang))*(AX*AX); % Rodrigues formula
end