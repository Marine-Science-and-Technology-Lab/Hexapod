 function [rotm] = E2R(EULER)
        % converts the Euler angle representation to a rotation matrix
        % assumes squence of rotations about x then y then z body-fixed axes
        % post-multiply Euler rotation matrices

        the = EULER(1); % rotation about x
        phi = EULER(2); % rotation about y
        psi = EULER(3); % rotation about z

        RZ = [cos(psi),-sin(psi), 0;...
              sin(psi), cos(psi), 0;...
                    0,         0, 1];
        
        RY = [cos(phi), 0, sin(phi);...
                    0, 1,         0;...
             -sin(phi), 0, cos(phi)];

        RX = [1,        0,        0;...
              0, cos(the),-sin(the);...
              0, sin(the), cos(the)];

        rotm = RZ*RY*RX;
    end