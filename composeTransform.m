function T = composeTransform(T1, T2)
% composeTransform - Compose two rigid transforms: T = T1 * T2.
%
%   T = composeTransform(T1, T2)
%
%   Convention: T_a_b maps points from frame b to frame a. If
%     T1 = T_a_b  and  T2 = T_b_c,  then  T = T_a_c.
%
%   For a point p expressed in frame c:
%     p_a = T1 * T2 * p_c
%         = T1.R * (T2.R * p_c + T2.t) + T1.t
%         = (T1.R * T2.R) * p_c + (T1.R * T2.t + T1.t)
%
%   so the composed transform has R = T1.R * T2.R and
%                                 t = T1.R * T2.t + T1.t.
%
%   Inputs
%     T1, T2 - transform structs with fields .R (3x3) and .t (3x1)
%
%   Outputs
%     T - composed transform (same struct layout)

T.R = T1.R * T2.R;
T.t = T1.R * T2.t + T1.t;
end
