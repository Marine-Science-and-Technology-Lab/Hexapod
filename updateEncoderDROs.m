function updateEncoderDROs(g, hex_setup, ActuatorPositions)
% updateEncoderDROs - Poll the controller for current encoder counts
% (via TP) and push them into the encoder-count / actuator-length
% widgets on the HexControl main panel.
%
%   updateEncoderDROs(g, hex_setup, ActuatorPositions)
%
%   Updates:
%     ActuatorPositions{1..6}  - Ax1..Ax6 linear gauges (actuator lengths,
%                                derived from counts via EncoderToLength)
%     ActuatorPositions{7..12} - AxNumeric_1..AxNumeric_6 (raw encoder counts)
%
%   Leaves pose readouts (13..18) and joint-separation displays (19..30)
%   untouched - those require forward kinematics (digital-shadow scope,
%   Docs/UPGRADE_PLAN.md §5.3).
%
%   Works against both py.gclib.py and GalilEmulator because both expose
%   a GCommand method that returns a struct with a .string field.
%
%   Axis mapping: the hexapod uses Galil axes A,B,C,E,F,G (skipping D).
%   TP returns all 8 axes A..H; we select [1,2,3,5,6,7].

ACTIVE_SLOTS = [1 2 3 5 6 7];   % A B C E F G
N_ACT = numel(ACTIVE_SLOTS);

try
    resp = g.GCommand('TP');
catch
    return  % controller busy or disconnected mid-poll; silently skip this tick
end

if ~isfield(resp, 'string') || isempty(resp.string)
    return
end

% Parse via explicit split (robust against sscanf edge cases on the
% trailing field without a terminating comma).
parts = strsplit(strtrim(resp.string), ',');
vals = str2double(parts);
if numel(vals) < max(ACTIVE_SLOTS) || any(isnan(vals(ACTIVE_SLOTS)))
    return
end

counts = vals(ACTIVE_SLOTS);         % 1 x 6
lengths = EncoderToLength(hex_setup, counts);

% Raw counts -> AxNumeric_1..AxNumeric_6 (indices 7..12)
% Lengths   -> Ax1Gauge..Ax6Gauge     (indices 1..6)
for k = 1:N_ACT
    ActuatorPositions{k + 6}.Value = counts(k);
    ActuatorPositions{k}.Value     = lengths(k);
end
end
