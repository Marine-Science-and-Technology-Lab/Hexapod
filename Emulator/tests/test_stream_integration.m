function test_stream_integration()
% test_stream_integration - Drive StreamContourData_to_Galil against the
% GalilEmulator with a small synthetic path and verify:
%   1) the loop completes without error
%   2) record_flag=1 returns a monotonic encoder trace that reaches the
%      commanded endpoint.

% Add the project root to the path so we can call the streaming helper.
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(root);
addpath(fullfile(root, 'Emulator'));

g = GalilEmulator();
g.GOpen('test');

% Tiny synthetic path: 50 samples at DT=2 (256 Hz). Ramp axis A from
% 0 to 1000 counts, others zero.
N = 50;
t = (0:N-1)';
hex_path = struct();
hex_path.dt = 4/1024;                   % DT=2 → 2^2 / 1024 = 3.9 ms
hex_path.axis_cts = zeros(N, 6);
hex_path.axis_cts(:,1) = round(linspace(0, 1000, N));

[exitcond, record] = StreamContourData_to_Galil(g, hex_path, 0, 1);

assert(strcmp(exitcond, 'DONE'), 'streaming should return DONE');
assert(~isempty(record.wall_t), 'record should have samples');
assert(size(record.positions,2) == 8, 'record positions must be 1x8');

% Axis A position should be monotonically non-decreasing and reach
% roughly the commanded end (1000 counts) by the final sample. Allow
% some tolerance because the emulator's lazy clock may lag slightly
% behind the final CD chunk until the drain phase completes.
a_trace = record.positions(:,1);
assert(all(diff(a_trace) >= 0), 'axis A should be non-decreasing');
final = a_trace(end);
assert(final >= 950 && final <= 1050, ...
    sprintf('axis A final position should be near 1000; got %g', final));

% Emulator should have logged a reasonable number of commands
snap = g.snapshot();
assert(isempty(snap.unknownCommands), ...
    'streaming should produce no unknown commands');

fprintf('PASS  test_stream_integration (axis A: 0 -> %g counts)\n', final);
end
