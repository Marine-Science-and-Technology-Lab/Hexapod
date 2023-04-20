function skew_u = skew(u)

    % creates a skew symmetric matrix from a vector
    % useful for cross products

    skew_u = [   0,-u(3), u(2);...
              u(3),    0,-u(1);...
             -u(2), u(1),    0];
end