 function [rotm] = E2R(EULER)
        % converts the Euler angle representation to a rotation matrix
        % assumes xyz squence of rotations about body-fixed axes
        % post-multiply Euler rotation matrices

        e1 = EULER(1); % rotation about x
        e2 = EULER(2); % rotation about y
        e3 = EULER(3); % rotation about z

        R1 = [1,       0,       0;...
              0, cos(e1),-sin(e1);...
              0, sin(e1), cos(e1)];

        R2 = [cos(e2), 0, sin(e2);...
                    0, 1,       0;...
             -sin(e2), 0, cos(e2)];

        R3 = [cos(e3),-sin(e3), 0;...
              sin(e3), cos(e3), 0;...
                    0,       0, 1];

        rotm = R1*R2*R3;
    end