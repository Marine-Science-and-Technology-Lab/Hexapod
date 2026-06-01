function results = test_GalilEmulator()
% test_GalilEmulator - End-to-end smoke tests for the Galil emulator.
%
% Exercises the command surface HexControl and the contour-streaming
% helpers use, and asserts that state evolves correctly.
%
% Run interactively:
%     cd Merged/Emulator/tests
%     test_GalilEmulator();
% Returns a struct-array of {name, ok, msg} results.

tests = { ...
    'open_close',          @test_open_close; ...
    'info_response',       @test_info_response; ...
    'stop_servo_home',     @test_stop_servo_home; ...
    'contour_round_trip',  @test_contour_round_trip; ...
    'buffer_drains',       @test_buffer_drains; ...
    'cd_skipped_axis_map', @test_cd_skipped_axis_map; ...
    'gpio_bits',           @test_gpio_bits; ...
    'program_download',    @test_program_download; ...
    'position_absolute',   @test_position_absolute; ...
    'tell_position',       @test_tell_position; ...
    'stream_end_sentinel', @test_stream_end_sentinel; ...
    'unknown_command',     @test_unknown_command; ...
};

addpath(fileparts(fileparts(mfilename('fullpath'))));  % add Emulator/ to path

results = struct('name', {}, 'ok', {}, 'msg', {});
for k = 1:size(tests, 1)
    name = tests{k,1};
    fn   = tests{k,2};
    try
        fn();
        results(end+1) = struct('name', name, 'ok', true, 'msg', ''); %#ok<AGROW>
        fprintf('  PASS  %s\n', name);
    catch ME
        results(end+1) = struct('name', name, 'ok', false, 'msg', ME.message); %#ok<AGROW>
        fprintf('  FAIL  %s -- %s\n', name, ME.message);
    end
end

nPass = sum([results.ok]);
nTotal = length(results);
fprintf('\n%d / %d passed\n', nPass, nTotal);
end


% ---------------------------------------------------------------- individual tests

function test_open_close()
g = GalilEmulator();
assertEqual(g.isOpen, false, 'should start closed');
g.GOpen('10.0.0.1');
assertEqual(g.isOpen, true, 'should be open after GOpen');
assertEqual(g.hostAddress, '10.0.0.1', 'address recorded');
g.GClose();
assertEqual(g.isOpen, false, 'should be closed after GClose');
end

function test_info_response()
g = newOpen();
resp = g.GInfo();
assert(isfield(resp, 'string'), 'GInfo must return struct with .string');
assert(contains(resp.string, 'GalilEmulator'), 'GInfo string should identify emulator');
end

function test_stop_servo_home()
g = newOpen();
g.GCommand('ST');
g.GCommand('SH ABCEFG');
snap = g.snapshot();
assertTrue(snap.servoActive(1) && snap.servoActive(2) && snap.servoActive(3), 'A,B,C servo on');
assertTrue(~snap.servoActive(4), 'D not in list, should be off');
assertTrue(snap.servoActive(5) && snap.servoActive(6) && snap.servoActive(7), 'E,F,G on');

g.GCommand('HMABCEFG');
snap = g.snapshot();
assertEqual(snap.positions, zeros(1,8), 'home zeros positions');
end

function test_contour_round_trip()
g = newOpen();
g.GCommand('ST');
g.GCommand('SH ABCEFG');
g.GCommand('CMABCEFG');
g.GCommand('DT 2');  % 4/1024 s per sample = 256 Hz

% Buffer free before any CD
free0 = str2num(g.GCommand('CM?').string); %#ok<ST2NM>
assertEqual(free0, 511, 'empty buffer reports full free space (Galil convention: 511)');

% Push a handful of increments
for k = 1:10
    g.GCommand(sprintf('CD %d,%d,%d,,%d,%d,%d', k, 2*k, 3*k, 4*k, 5*k, 6*k));
end
free1 = str2num(g.GCommand('CM?').string); %#ok<ST2NM>
assertTrue(free1 >= 500 && free1 <= 511, 'some samples buffered');
end

function test_buffer_drains()
g = newOpen();
g.GCommand('ST');
g.GCommand('CMABCEFG');
g.GCommand('DT 2');  % 256 Hz

% Load 256 samples (= 1 s at 256 Hz)
for k = 1:256
    g.GCommand('CD 1,0,0,,0,0,0');
end

% Wait ~0.3 s of wall time; expect ~76 samples drained
pause(0.3);
snap = g.snapshot();
assertTrue(snap.contourBufferDepth < 256, 'buffer should be draining over time');
assertTrue(snap.positions(1) >= 50 && snap.positions(1) <= 256, ...
    sprintf('axis A should have advanced; got %g', snap.positions(1)));
end

function test_gpio_bits()
g = newOpen();
g.GCommand('SB 17');
g.GCommand('SB25');
g.GCommand('SB 33');
snap = g.snapshot();
assertTrue(snap.gpio(18), 'bit 17 set (1-indexed 18)');
assertTrue(snap.gpio(26), 'bit 25 set');
assertTrue(snap.gpio(34), 'bit 33 set');

g.GCommand('CB17');
snap = g.snapshot();
assertTrue(~snap.gpio(18), 'bit 17 cleared');
end

function test_program_download()
g = newOpen();
prog = sprintf('#Pulse; \n SB 33; \n #A; \n SB 25; \n SB 17; \n WT16,1; \n CB 25; \n CB 17; \n WT16,1; \n JP #A; \n CB 25; \n CB 17; \n EN');
g.GProgramDownload(prog);
% Should have stored at least the #Pulse label
resp = g.GCommand('XQ #Pulse,2');
assertEqual(resp.string, ':', 'known program XQ should ack');
snap = g.snapshot();
assertTrue(any(strcmp(snap.runningPrograms, 'Pulse')), 'Pulse program should be running');

% Unknown label => '?'
resp = g.GCommand('XQ #Nonexistent,0');
assertEqual(resp.string, '?', 'unknown program XQ should return ?');
end

function test_position_absolute()
g = newOpen();
g.GCommand('PA 100,200,300,,400,500,600');
snap = g.snapshot();
assertEqual(snap.positions(1), 100, 'A set');
assertEqual(snap.positions(2), 200, 'B set');
assertEqual(snap.positions(3), 300, 'C set');
assertTrue(isnan(snap.positions(4)) || snap.positions(4) == 0, 'D not set');
assertEqual(snap.positions(5), 400, 'E set');
end

function test_tell_position()
g = newOpen();
g.GCommand('PA 10,20,30,,40,50,60');
resp = g.GCommand('TPA');
assertEqual(resp.string, '10', 'TPA returns axis A count');
resp = g.GCommand('TP');
assert(contains(resp.string, ','), 'TP returns comma-separated list');
end

function test_stream_end_sentinel()
g = newOpen();
g.GCommand('CMABCEFG');
g.GCommand('DT 2');
g.GCommand('CD 1,2,3,,4,5,6');
g.GCommand('CD 0,0,0,,0,0,0=0');  % end sentinel
snap = g.snapshot();
assertTrue(snap.contourSealed, 'sentinel must seal the contour');
end

function test_unknown_command()
g = newOpen();
warning('off', 'GalilEmulator:UnknownCommand');
cleanup = onCleanup(@() warning('on', 'GalilEmulator:UnknownCommand'));
resp = g.GCommand('TOTALLYBOGUS 42');
assertEqual(resp.string, '', 'unknown command returns empty string');
snap = g.snapshot();
assertTrue(any(strcmp(snap.unknownCommands, 'TOTALLYBOGUS 42')), 'unknown logged');
end

function test_cd_skipped_axis_map()
% Regression test: CD commands on the hexapod's ABCEFG axis layout (D
% and H skipped) must NOT collapse the empty slot after C. If strsplit
% ever regresses to its default CollapseDelimiters=true behavior, axes
% E, F, G shift by one slot and actuator 6 silently reads 0 forever -
% the exact bug that bit HexControl's DROs.
g = newOpen();
g.GCommand('ST');
g.GCommand('SH ABCEFG');
g.GCommand('CMABCEFG');
g.GCommand('DT 2');

% One CD row with a distinct marker value per active axis.
g.GCommand('CD 10,20,30,,40,50,60');
g.GCommand('CD 0,0,0,,0,0,0=0');   % seal the contour
g.GMotionComplete('ABCEFG');       % drain

snap = g.snapshot();
p = snap.positions;

% Axes A B C  (slots 1 2 3) and E F G (slots 5 6 7) must each carry
% their own value. Skipped axes D (slot 4) and H (slot 8) stay zero.
assertEqual(p(1),  10, 'axis A (slot 1) = actuator 1');
assertEqual(p(2),  20, 'axis B (slot 2) = actuator 2');
assertEqual(p(3),  30, 'axis C (slot 3) = actuator 3');
assertEqual(p(4),   0, 'axis D (slot 4) skipped - must stay 0');
assertEqual(p(5),  40, 'axis E (slot 5) = actuator 4');
assertEqual(p(6),  50, 'axis F (slot 6) = actuator 5');
assertEqual(p(7),  60, 'axis G (slot 7) = actuator 6');
assertEqual(p(8),   0, 'axis H (slot 8) skipped - must stay 0');
end


% ---------------------------------------------------------------- helpers

function g = newOpen()
g = GalilEmulator();
g.GOpen('test');
end

function assertTrue(cond, msg)
if ~cond
    error(msg);
end
end

function assertEqual(actual, expected, msg)
if ischar(expected) || isstring(expected)
    if ~strcmp(actual, expected)
        error('%s: expected "%s", got "%s"', msg, char(expected), char(actual));
    end
else
    if ~isequal(actual, expected)
        error('%s: expected %s, got %s', msg, mat2str(expected), mat2str(actual));
    end
end
end
