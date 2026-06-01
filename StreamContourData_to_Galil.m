function [exitcond, record] = StreamContourData_to_Galil(g, hex_path, trigger_flag, record_flag, cancel_token)
% StreamContourData_to_Galil - Synchronous wrapper around
% ContourStreamSession. Kept for backward compatibility with the
% existing tests and script-style callers.
%
% New code (app callbacks, especially anything that should remain
% responsive while streaming runs) should prefer ContourStreamSession
% directly and supply an on_complete handler. The session runs the
% state machine on a MATLAB timer, so the caller returns immediately
% and other timers (encoder DROs, animation) and UI events get
% guaranteed airtime between ticks without needing drawnow sprinkled
% inside a blocking loop.
%
% This wrapper simply creates a session, then polls isDone() on a
% pause(0.05) loop so the MATLAB event loop still services timers and
% cancel-token clicks while we wait. See ContourStreamSession.m for
% full state-machine / soft-stop / buffer-health details.
%
% Inputs / Outputs: identical to the pre-session version. See the
% header of ContourStreamSession for a fuller description of each
% field (cancel_token, record struct, buffer_health, etc.).

if nargin < 3 || isempty(trigger_flag);  trigger_flag = 0; end
if nargin < 4 || isempty(record_flag);   record_flag  = 0; end
if nargin < 5;                           cancel_token = []; end

session = ContourStreamSession(g, hex_path, ...
    trigger_flag, record_flag, cancel_token, []);

cleanup = onCleanup(@() delete(session));

while ~session.isDone()
    pause(0.05);   % yields to MATLAB event loop: timers + UI callbacks fire
end

exitcond = session.exitcond;
record   = session.record;
end
