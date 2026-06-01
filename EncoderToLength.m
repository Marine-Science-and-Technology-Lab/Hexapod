function ax_length = EncoderToLength(hex_setup, ax_counts)
% EncoderToLength - Inverse of LengthToEncoder. Convert encoder counts
% back to actuator lengths (meters) using the per-axis datum and scale.
%
%   ax_length = EncoderToLength(hex_setup, ax_counts)
%
%   Inputs
%     hex_setup - struct with fields
%                   .Actuators.CountsPerM
%                   .Actuators.DatumLength_Individual  (6x1)
%     ax_counts - Nx6 or 6xN encoder-count array. Shape is preserved.
%
%   Output
%     ax_length - same shape as ax_counts, in meters.

scale = hex_setup.Actuators.CountsPerM;
datum = hex_setup.Actuators.DatumLength_Individual(:);   % 6x1

if size(ax_counts, 1) == 6
    ax_length = ax_counts ./ scale + datum;
elseif size(ax_counts, 2) == 6
    ax_length = ax_counts ./ scale + datum.';
else
    error('EncoderToLength:BadShape', ...
        'ax_counts must have 6 rows or 6 columns; got %dx%d', ...
        size(ax_counts, 1), size(ax_counts, 2));
end
end
