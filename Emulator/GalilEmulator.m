classdef GalilEmulator < handle
    % GalilEmulator - Software stand-in for a Galil DMC-40x0 via gclib.
    %
    % Implements the subset of the py.gclib.py API that HexControl and
    % the streaming helpers actually call: GOpen, GClose, GCommand,
    % GProgramDownload, GInfo, GMotionComplete. Responses are returned
    % as structs with a .string field, matching the access pattern
    % already used throughout the codebase (e.g. str2num(resp.string)).
    %
    % The emulator maintains a virtual clock that drains the contour
    % buffer lazily on any CM? query: commanded increments accumulate
    % into the internal encoder-count state as simulated time passes.
    % This makes TP responses and CM? buffer-fill readings behave the
    % way the GUI and streaming loop expect.
    %
    % See Merged/Docs/UPGRADE_PLAN.md §5.1 for scope and acceptance.
    %
    % Typical lifecycle:
    %   g = GalilEmulator();
    %   g.GOpen('emulator');
    %   g.GCommand('ST');
    %   g.GCommand('SH ABCEFG');
    %   g.GCommand('CMABCEFG');
    %   g.GCommand('DT 2');
    %   g.GCommand('CD 10,20,30,,40,50,60');
    %   ...
    %   resp = g.GCommand('CM?');
    %   free = str2num(resp.string);  %#ok<ST2NM>
    %   ...
    %   g.GClose();

    properties (Constant)
        ContourBufferSize = 511          % Galil DMC-40x0 contour buffer depth (= CM? reading when empty)
        SampleBaseHz      = 1024         % DT units are 1/1024 s
        NumAxes           = 8            % A..H
        AxisLetters       = 'ABCDEFGH'
    end

    properties (Access = public)
        % Diagnostics / tuning knobs the host can set directly
        CommandLogCapacity double = 1024
        UnknownWarnOnce    logical = true
    end

    properties (SetAccess = private)
        isOpen        logical = false
        hostAddress   char    = ''

        positions     (1,8) double  = zeros(1,8)    % integrated encoder counts per axis
        servoActive   (1,8) logical = false(1,8)
        mode          char          = 'idle'        % 'idle' | 'contour' | 'position' | 'jog'
        stopRequested logical = false

        % Contour-mode state
        contourAxes    (1,8) logical = false(1,8)   % axes active under the last CM
        contourBuffer  (:,8) double  = zeros(0,8)   % pending increments (rows = samples)
        contourHead    double        = 0            % samples already executed (dequeued)
        contourDTExp   double        = 0            % exponent n in DT <n>; period = 2^n / SampleBaseHz
        contourSealed  logical       = false        % true once CD ...=0 sentinel received
        contourClockRef uint64       = uint64(0)    % tic reference for lazy buffer draining
        contourFracSamples double    = 0            % fractional sample carried across advanceVirtualClock calls

        % I/O and programs
        gpio           (1,64) logical = false(1,64)
        gpioOutputMask uint64         = uint64(0)
        programs                                     % containers.Map label -> body text
        runningPrograms                              % containers.Map thread -> label

        % Diagnostics
        commandLog      cell   = cell(1, 1024)     % circular buffer
        commandLogHead  double = 0                 % index of last written slot
        commandLogTotal double = 0                 % lifetime count (never truncated)
        unknownCommands cell   = {}
        responseCount   double = 0
    end

    methods
        function obj = GalilEmulator()
            obj.programs        = containers.Map('KeyType', 'char', 'ValueType', 'char');
            obj.runningPrograms = containers.Map('KeyType', 'double', 'ValueType', 'char');
        end

        % -------------------------------------------------------------- gclib surface

        function GOpen(obj, address)
            if nargin < 2; address = ''; end
            obj.hostAddress = char(address);
            obj.isOpen = true;
            obj.contourClockRef = tic;
        end

        function GClose(obj)
            obj.isOpen = false;
        end

        function resp = GInfo(obj)
            resp = obj.makeResponse(sprintf( ...
                'GalilEmulator, rev 0, %s, hostname', obj.hostAddress));
        end

        function resp = GCommand(obj, cmd)
            if iscell(cmd); cmd = cmd{1}; end
            cmd = strtrim(char(cmd));
            obj.logCommand(cmd);
            obj.advanceVirtualClock();

            % Some scripts send multiple ;-separated commands in one shot.
            % Dispatch each one; return the final response.
            parts = obj.splitCommandList(cmd);
            resp = obj.makeResponse('');
            for k = 1:length(parts)
                piece = strtrim(parts{k});
                if isempty(piece); continue; end
                resp = obj.dispatch(piece);
            end
        end

        function GProgramDownload(obj, programText)
            % Programs take the form:
            %   '#Label1; <body1>; #Label2; <body2>; ... EN'
            % Whitespace and newlines inside the body are ignored by Galil.
            text = char(programText);
            text = regexprep(text, '\r', '');        % strip CR
            tokens = regexp(text, '#(\w+)\s*;?\s*((?:(?!#\w+\s*;).|\n)*)', 'tokens');
            for k = 1:length(tokens)
                label = tokens{k}{1};
                body  = strtrim(tokens{k}{2});
                obj.programs(label) = body;
            end
        end

        function GMotionComplete(obj, ~)
            % For the emulator, motion is completed synchronously by
            % draining the contour buffer at its commanded rate.
            obj.drainContourToEmpty();
        end

        % -------------------------------------------------------------- introspection

        function snap = snapshot(obj)
            % Convenience: full state readout for tests/diagnostics.
            obj.advanceVirtualClock();
            snap = struct( ...
                'positions',    obj.positions, ...
                'servoActive',  obj.servoActive, ...
                'mode',         obj.mode, ...
                'gpio',         obj.gpio(1:64), ...
                'contourBufferDepth', size(obj.contourBuffer,1) - obj.contourHead, ...
                'contourSealed', obj.contourSealed, ...
                'contourRateHz', obj.contourSampleRateHz(), ...
                'runningPrograms', obj.runningPrograms.keys, ...
                'unknownCommands', {obj.unknownCommands}, ...
                'lastCommand', obj.lastLoggedCommand() ...
            );
        end

        function reset(obj)
            obj.positions      = zeros(1,8);
            obj.servoActive    = false(1,8);
            obj.mode           = 'idle';
            obj.stopRequested  = false;
            obj.contourAxes    = false(1,8);
            obj.contourBuffer  = zeros(0,8);
            obj.contourHead    = 0;
            obj.contourDTExp   = 0;
            obj.contourSealed  = false;
            obj.contourFracSamples = 0;
            obj.gpio           = false(1,64);
            obj.commandLog     = cell(1, obj.CommandLogCapacity);
            obj.commandLogHead = 0;
            obj.commandLogTotal= 0;
            obj.unknownCommands= {};
            obj.responseCount  = 0;
            obj.programs       = containers.Map('KeyType', 'char', 'ValueType', 'char');
            obj.runningPrograms= containers.Map('KeyType', 'double', 'ValueType', 'char');
        end
    end

    methods (Access = private)

        % -------------------------------------------------------------- dispatch

        function resp = dispatch(obj, cmd)
            % One command string, routed to a handler. All handlers
            % return a response struct with a .string field.

            % Fast-path queries first (no state change).
            if strcmpi(cmd, 'CM?')
                free = obj.ContourBufferSize - (size(obj.contourBuffer,1) - obj.contourHead);
                free = max(0, min(free, obj.ContourBufferSize));
                resp = obj.makeResponse(sprintf('%d', free));
                return
            end

            % ST - stop
            if strcmpi(cmd, 'ST') || strcmpi(cmd, 'AB')
                obj.mode          = 'idle';
                obj.contourBuffer = zeros(0,8);
                obj.contourHead   = 0;
                obj.contourSealed = false;
                obj.stopRequested = true;
                resp = obj.makeResponse(':');
                return
            end

            % MO - motor off (all or axes)
            tok = regexp(cmd, '^MO\s*([A-H]*)$', 'tokens', 'once');
            if ~isempty(tok)
                mask = obj.axesToMask(tok{1}, true);  % empty => all
                obj.servoActive(mask) = false;
                resp = obj.makeResponse(':');
                return
            end

            % SH <axes> or SHABCEFG (no space)
            tok = regexp(cmd, '^SH\s*([A-H]*)$', 'tokens', 'once');
            if ~isempty(tok)
                mask = obj.axesToMask(tok{1}, true);
                obj.servoActive(mask) = true;
                resp = obj.makeResponse(':');
                return
            end

            % BG <axes>
            tok = regexp(cmd, '^BG\s*([A-H]*)$', 'tokens', 'once');
            if ~isempty(tok)
                obj.mode = 'position';
                resp = obj.makeResponse(':');
                return
            end

            % HM <axes> - home
            tok = regexp(cmd, '^HM\s*([A-H]*)$', 'tokens', 'once');
            if ~isempty(tok)
                mask = obj.axesToMask(tok{1}, true);
                obj.positions(mask) = 0;
                resp = obj.makeResponse(':');
                return
            end

            % CM<axes> - enter contour mode
            tok = regexp(cmd, '^CM\s*([A-H]+)$', 'tokens', 'once');
            if ~isempty(tok)
                obj.contourAxes   = obj.axesToMask(tok{1}, false);
                obj.mode          = 'contour';
                obj.contourBuffer = zeros(0,8);
                obj.contourHead   = 0;
                obj.contourSealed = false;
                obj.contourClockRef = tic;
                resp = obj.makeResponse(':');
                return
            end

            % DT <n> - contour timestep exponent
            tok = regexp(cmd, '^DT\s+(-?\d+)$', 'tokens', 'once');
            if ~isempty(tok)
                obj.contourDTExp = str2double(tok{1});
                resp = obj.makeResponse(':');
                return
            end

            % CD ... (contour data, possibly with =0 sentinel)
            if startsWith(upper(cmd), 'CD ') || startsWith(upper(cmd), 'CD,')
                resp = obj.handleCD(cmd);
                return
            end

            % PA <args> - position absolute
            if startsWith(upper(cmd), 'PA')
                vals = obj.parseAxisList(cmd(3:end));
                present = ~isnan(vals);
                obj.positions(present) = vals(present);
                obj.mode = 'position';
                resp = obj.makeResponse(':');
                return
            end

            % PR <args> - position relative
            if startsWith(upper(cmd), 'PR')
                vals = obj.parseAxisList(cmd(3:end));
                present = ~isnan(vals);
                obj.positions(present) = obj.positions(present) + vals(present);
                resp = obj.makeResponse(':');
                return
            end

            % DP<axis>=<n> - define position for single axis
            tok = regexp(cmd, '^DP([A-H])\s*=\s*(-?\d+(?:\.\d+)?)$', 'tokens', 'once');
            if ~isempty(tok)
                idx = obj.axisLetterToIndex(tok{1});
                obj.positions(idx) = str2double(tok{2});
                resp = obj.makeResponse(':');
                return
            end

            % DP <v1>,<v2>,... - define all-axis positions
            tok = regexp(cmd, '^DP\s+(.+)$', 'tokens', 'once');
            if ~isempty(tok)
                vals = obj.parseAxisList(tok{1});
                present = ~isnan(vals);
                obj.positions(present) = vals(present);
                resp = obj.makeResponse(':');
                return
            end

            % WT<n>  or  WT <n>
            tok = regexp(cmd, '^WT\s*(\d+)$', 'tokens', 'once');
            if ~isempty(tok)
                % Acknowledge but don't actually block - emulator does
                % everything synchronously via the virtual clock.
                resp = obj.makeResponse(':');
                return
            end

            % WT <chan>,<val>  - wait with I/O channel (used inside programs)
            tok = regexp(cmd, '^WT\s*(\d+)\s*,\s*(\d+)$', 'tokens', 'once');
            if ~isempty(tok)
                resp = obj.makeResponse(':');
                return
            end

            % SB <n>  or  SB<n>
            tok = regexp(cmd, '^SB\s*(\d+)$', 'tokens', 'once');
            if ~isempty(tok)
                obj.setBit(str2double(tok{1}), true);
                resp = obj.makeResponse(':');
                return
            end

            % CB <n>  or  CB<n>
            tok = regexp(cmd, '^CB\s*(\d+)$', 'tokens', 'once');
            if ~isempty(tok)
                obj.setBit(str2double(tok{1}), false);
                resp = obj.makeResponse(':');
                return
            end

            % CO <mask> - configure outputs
            tok = regexp(cmd, '^CO\s+(\d+)$', 'tokens', 'once');
            if ~isempty(tok)
                obj.gpioOutputMask = uint64(str2double(tok{1}));
                resp = obj.makeResponse(':');
                return
            end

            % XQ #<label>[,<thread>]
            tok = regexp(cmd, '^XQ\s*#(\w+)\s*(?:,\s*(\d+))?$', 'tokens', 'once');
            if ~isempty(tok)
                label = tok{1};
                if length(tok) >= 2 && ~isempty(tok{2})
                    thread = str2double(tok{2});
                else
                    thread = 0;
                end
                if obj.programs.isKey(label)
                    obj.runningPrograms(thread) = label;
                    resp = obj.makeResponse(':');
                else
                    resp = obj.makeResponse('?');
                end
                return
            end

            % HX<thread> - halt thread
            tok = regexp(cmd, '^HX\s*(\d+)?$', 'tokens', 'once');
            if ~isempty(tok)
                if ~isempty(tok) && ~isempty(tok{1})
                    thread = str2double(tok{1});
                    if obj.runningPrograms.isKey(thread)
                        obj.runningPrograms.remove(thread);
                    end
                else
                    obj.runningPrograms = containers.Map('KeyType', 'double', 'ValueType', 'char');
                end
                resp = obj.makeResponse(':');
                return
            end

            % TP[<axis>][?] - tell position
            tok = regexp(cmd, '^TP([A-H])?\??$', 'tokens', 'once');
            if ~isempty(tok)
                if isempty(tok{1})
                    pos = round(obj.positions);
                    resp = obj.makeResponse(strjoin( ...
                        arrayfun(@(v) sprintf('%d', v), pos, 'UniformOutput', false), ','));
                else
                    idx = obj.axisLetterToIndex(tok{1});
                    resp = obj.makeResponse(sprintf('%d', round(obj.positions(idx))));
                end
                return
            end

            % TS<axis> - tell switches (return 0 as a benign default)
            tok = regexp(cmd, '^TS([A-H])?$', 'tokens', 'once');
            if ~isempty(tok)
                resp = obj.makeResponse('0');
                return
            end

            % MG ... - message (no-op, returns empty)
            if startsWith(upper(cmd), 'MG')
                resp = obj.makeResponse('');
                return
            end

            % EN - end program (harmless at top level)
            if strcmpi(cmd, 'EN')
                resp = obj.makeResponse(':');
                return
            end

            % JP ... - jump (no-op at top level)
            if startsWith(upper(cmd), 'JP')
                resp = obj.makeResponse(':');
                return
            end

            % BN - burn (no-op)
            if strcmpi(cmd, 'BN')
                resp = obj.makeResponse(':');
                return
            end

            % Anything else: log as unknown, return empty string.
            obj.noteUnknown(cmd);
            resp = obj.makeResponse('');
        end

        % -------------------------------------------------------------- CD handler

        function resp = handleCD(obj, cmd)
            % Accept either bare "CD v1,v2,..." or terminator "CD ...=0".
            payload = regexprep(cmd, '^CD\s*', '', 'ignorecase');
            isSentinel = false;
            eqIdx = strfind(payload, '=');
            if ~isempty(eqIdx)
                % e.g. "0,0,0,,0,0,0=0" - strip the =0 suffix
                payload = payload(1:eqIdx(end)-1);
                isSentinel = true;
            end

            vals = obj.parseAxisList(payload);
            row = zeros(1,8);
            present = ~isnan(vals);
            row(present) = vals(present);

            if isSentinel
                obj.contourSealed = true;
                % The sentinel itself is not added to the buffer; any
                % non-zero values supplied with it would be an unusual
                % end-of-contour pad, and we simply drop them.
                resp = obj.makeResponse(':');
                return
            end

            % Append to the pending-buffer tail
            obj.contourBuffer(end+1, :) = row;
            resp = obj.makeResponse(':');
        end

        % -------------------------------------------------------------- virtual clock

        function advanceVirtualClock(obj)
            % Drain the contour buffer in proportion to elapsed wall time
            % since the last update, carrying any fractional sample over
            % into the next call so long-run drift stays zero.
            if ~strcmp(obj.mode, 'contour') || obj.contourClockRef == uint64(0)
                obj.contourClockRef = tic;
                return
            end
            rate = obj.contourSampleRateHz();
            if rate <= 0
                return
            end
            elapsed = toc(obj.contourClockRef);
            obj.contourClockRef = tic;

            samplesFractional = elapsed * rate + obj.contourFracSamples;
            samplesWhole = floor(samplesFractional);
            obj.contourFracSamples = samplesFractional - samplesWhole;

            if samplesWhole <= 0
                return
            end

            pending = size(obj.contourBuffer,1) - obj.contourHead;
            n = min(samplesWhole, pending);
            if n > 0
                idx = (obj.contourHead+1):(obj.contourHead+n);
                delta = sum(obj.contourBuffer(idx, :), 1);
                obj.positions = obj.positions + delta;
                obj.contourHead = obj.contourHead + n;
                if obj.contourHead > 4096
                    obj.contourBuffer = obj.contourBuffer(obj.contourHead+1:end, :);
                    obj.contourHead = 0;
                end
            end
        end

        function drainContourToEmpty(obj)
            pending = size(obj.contourBuffer,1) - obj.contourHead;
            if pending > 0
                idx = (obj.contourHead+1):(obj.contourHead+pending);
                delta = sum(obj.contourBuffer(idx,:), 1);
                obj.positions = obj.positions + delta;
                obj.contourHead = obj.contourHead + pending;
            end
            obj.contourBuffer = zeros(0,8);
            obj.contourHead = 0;
            obj.contourClockRef = tic;
        end

        function rate = contourSampleRateHz(obj)
            % DT <n>: sample period = 2^n / SampleBaseHz  seconds
            % so rate = SampleBaseHz / 2^n samples/sec.
            n = obj.contourDTExp;
            rate = obj.SampleBaseHz / (2.^n);
        end

        % -------------------------------------------------------------- parsing helpers

        function vals = parseAxisList(obj, argString)
            % Parse "1,2,3,,4,5,6" into an 8-vector padded with NaN for
            % absent positions. Commas separate axes A..H in order.
            % CollapseDelimiters MUST be false so that the empty slot
            % for a skipped axis (e.g. D in ABCEFG) is preserved as ''
            % rather than swallowed - otherwise axes past the skip get
            % shifted and the last physical axis silently loses its
            % delta.
            argString = strtrim(argString);
            parts = strsplit(argString, ',', 'CollapseDelimiters', false);
            vals = nan(1, obj.NumAxes);
            n = min(length(parts), obj.NumAxes);
            for k = 1:n
                s = strtrim(parts{k});
                if ~isempty(s)
                    v = str2double(s);
                    if ~isnan(v)
                        vals(k) = v;
                    end
                end
            end
        end

        function mask = axesToMask(obj, letters, allIfEmpty)
            mask = false(1, obj.NumAxes);
            if isempty(letters)
                if allIfEmpty; mask(:) = true; end
                return
            end
            letters = upper(letters);
            for k = 1:length(letters)
                idx = strfind(obj.AxisLetters, letters(k));
                if ~isempty(idx); mask(idx) = true; end
            end
        end

        function idx = axisLetterToIndex(obj, letter)
            idx = strfind(obj.AxisLetters, upper(letter));
            if isempty(idx); idx = 1; end
        end

        function parts = splitCommandList(~, cmd)
            % Split on ';' but preserve semicolons inside program bodies.
            % CollapseDelimiters=false so that an empty slot between two
            % consecutive ';' separators doesn't cause silent shifting;
            % the caller strtrims and skips empty pieces anyway.
            parts = strsplit(cmd, ';', 'CollapseDelimiters', false);
        end

        % -------------------------------------------------------------- bookkeeping

        function resp = makeResponse(obj, text)
            obj.responseCount = obj.responseCount + 1;
            resp = struct('string', char(text));
        end

        function setBit(obj, bit, value)
            if bit >= 0 && bit < length(obj.gpio)
                obj.gpio(bit+1) = logical(value);
            end
        end

        function logCommand(obj, cmd)
            % O(1) circular-buffer write. Avoids the O(n) copy that the
            % prior "grow-then-trim" approach did every time capacity
            % was exceeded, which under sustained 10 Hz + multi-command-
            % per-tick load could stall long enough to trigger phantom
            % Ctrl+C interrupts under memory pressure.
            cap = obj.CommandLogCapacity;
            if numel(obj.commandLog) ~= cap
                % Capacity was changed at runtime; reallocate.
                obj.commandLog = cell(1, cap);
                obj.commandLogHead = 0;
            end
            obj.commandLogHead = mod(obj.commandLogHead, cap) + 1;
            obj.commandLog{obj.commandLogHead} = cmd;
            obj.commandLogTotal = obj.commandLogTotal + 1;
        end

        function noteUnknown(obj, cmd)
            obj.unknownCommands{end+1} = cmd; %#ok<AGROW>
            if obj.UnknownWarnOnce
                % Only warn the first time each unique command appears.
                prior = obj.unknownCommands(1:end-1);
                if ~any(strcmp(prior, cmd))
                    warning('GalilEmulator:UnknownCommand', ...
                        'Emulator received unrecognized command: "%s"', cmd);
                end
            end
        end

        function c = lastLoggedCommand(obj)
            if obj.commandLogHead == 0
                c = '';
            else
                c = obj.commandLog{obj.commandLogHead};
                if isempty(c); c = ''; end
            end
        end
    end
end
