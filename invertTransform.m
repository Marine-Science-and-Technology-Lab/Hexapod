function T_inv = invertTransform(T)
% invertTransform - Rigid inverse of a transform.
%
%   T_inv = invertTransform(T)
%
%   If p_a = T.R * p_b + T.t, the inverse maps p_a back to p_b:
%     p_b = T.R' * (p_a - T.t) = T.R' * p_a - T.R' * T.t
%   so  T_inv.R = T.R'  and  T_inv.t = -T.R' * T.t.
%
%   Inputs
%     T - transform struct with fields .R (3x3 rotation) and .t (3x1 translation)
%
%   Outputs
%     T_inv - transform struct for the inverse

T_inv.R = T.R';
T_inv.t = -T.R' * T.t;
end
