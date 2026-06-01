function test_softstop()
% test_softstop - Soft-stop cancel-token behavior against the emulator.
%
% Drives StreamContourData_to_Galil with a pre-requested CancelToken so
% the streaming loop breaks on its first iteration and the soft-stop
% branch takes over. Asserts:
%   1) exitcond is 'SOFTSTOP'
%   2) the Hann-tapered samples were actually sent (the emulator's
%      final axis-A position is consistent with the taper, not with
%      the full commanded trajectory end)
%   3) no unknown commands were produced

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(root);
addpath(fullfile(root, 'Emulator'));

g = GalilEmulator();
g.GOpen('softstop-test');

% Tiny path: 500-sample axis-A ramp from 0 to 2000 counts over ~2 s.
% A full taper from the start totals roughly the integral of the Hann
% window applied to the ramp, which is well below the full 2000.
N = 500;
hex_path = struct();
hex_path.dt = 4/1024;
hex_path.axis_cts = zeros(N, 6);
hex_path.axis_cts(:,1) = linspace(0, 2000, N);

% Pre-request cancel so the very first iteration breaks out.
token = CancelToken();
token.request();

[exitcond, record] = StreamContourData_to_Galil(g, hex_path, 0, 1, token);

assert(strcmp(exitcond, 'SOFTSTOP'), ...
    'soft-stop should set exitcond = SOFTSTOP (got "%s")', exitcond);

final_A = record.positions(end, 1);
assert(final_A > 0, 'axis A should have moved at all');
assert(final_A < 2000 * 0.6, ...
    'axis A final position %g should be below 60%% of full ramp (2000)', final_A);

snap = g.snapshot();
assert(isempty(snap.unknownCommands), ...
    'no unknown commands expected: %s', strjoin(snap.unknownCommands, '; '));

fprintf('PASS  test_softstop  (exitcond=%s, axis A final=%.1f counts, stopped short)\n', ...
    exitcond, final_A);
end
