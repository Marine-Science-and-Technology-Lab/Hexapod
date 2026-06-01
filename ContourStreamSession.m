classdef ContourStreamSession < handle
    % ContourStreamSession - Non-blocking contour-mode streaming to Galil.
    %
    % A handle class that runs the streaming state machine on a MATLAB
    % timer so the caller returns immediately. The session fires ticks
    % at TICK_PERIOD, each doing a small piece of work (send a chunk,
    % poll CM?, check the cancel token). Between ticks the MATLAB event
    % loop runs naturally - UI callbacks, other timers (encoder DROs,
    % animation), and the Soft Stop click all get guaranteed airtime
    % without needing drawnow sprinkled inside a blocking loop.
    %
    % Usage (non-blocking, preferred for app callbacks):
    %
    %   session = ContourStreamSession(g, hex_path, ...
    %                                   trigger_flag, record_flag, ...
    %                                   cancel_token, @onComplete);
    %   % function returns immediately; streaming runs on the timer
    %
    %   function onComplete(exitcond, record)
    %       % called when the session finishes (DONE, SOFTSTOP, ERROR)
    %   end
    %
    % For scripts / tests that want synchronous semantics, call
    % StreamContourData_to_Galil - it wraps a session and blocks on the
    % session's isDone() method with a pause(0.05) loop.
    %
    % States:
    %   INIT       - constructed but not yet prepared
    %   STREAMING  - sending pre-planned contour chunks
    %   TAPERING   - soft-stop Hann ramp-down in progress
    %   DRAINING   - waiting for the Galil buffer to empty
    %   COMPLETE   - final commands sent, on_complete invoked
    %
    % Transitions:
    %   INIT       -> STREAMING    (after prepare())
    %   STREAMING  -> TAPERING     (cancel_token.requested)
    %   STREAMING  -> DRAINING     (all chunks sent)
    %   TAPERING   -> DRAINING     (all taper chunks sent)
    %   DRAINING   -> COMPLETE     (buffer reports BUFFER_CAPACITY free)

    properties (Access = public)
        state        char     = 'INIT'     % lifecycle state, see above
        exitcond     char     = ''         % 'DONE' | 'SOFTSTOP' | 'ERROR'
        record       struct   = struct('wall_t', [], 'positions', zeros(0,8))
    end

    properties (Access = private)
        g
        hex_path
        trigger_flag   logical = false
        record_flag    logical = false
        cancel_token   = []
        on_complete    = []

        % Constants
        TargetBuff           double = 250
        BUFFER_CAPACITY      double = 511     % Galil DMC-40x0 contour buffer
        STARVATION_THRESHOLD double = 50
        TICK_PERIOD          double = 0.05    % seconds (20 Hz default)

        % Trajectory
        ydiff
        posStr
        cmdArrays   double = 0

        % Streaming indices
        n double = 1       % chunk counter (1..cmdArrays)
        i double = 1       % starting index into posStr for next chunk
        j double = 0       % committed-sample count

        % Taper state
        N_taper  double = 0
        taperStr
        kt       double = 1

        % Record state
        record_t_start uint64 = uint64(0)
        record_idx     double = 0

        % Buffer health
        buf_min_pending double = Inf

        % Timer
        timer_handle
    end

    methods
        function obj = ContourStreamSession(g, hex_path, trigger_flag, record_flag, cancel_token, on_complete)
            if nargin < 3 || isempty(trigger_flag); trigger_flag = false; end
            if nargin < 4 || isempty(record_flag);  record_flag  = false; end
            if nargin < 5;                           cancel_token = [];    end
            if nargin < 6;                           on_complete  = [];    end

            obj.g            = g;
            obj.hex_path     = hex_path;
            obj.trigger_flag = logical(trigger_flag);
            obj.record_flag  = logical(record_flag);
            obj.cancel_token = cancel_token;
            obj.on_complete  = on_complete;

            obj.record.buffer_health = struct( ...
                'min_pending',  NaN, ...
                'threshold',    obj.STARVATION_THRESHOLD, ...
                'starved_risk', false);

            try
                obj.prepare();
                obj.startTimer();
            catch ME
                obj.exitcond = 'ERROR';
                obj.state    = 'COMPLETE';
                obj.stopTimer();
                warning('ContourStreamSession:PrepareFailed', ...
                    'Stream prepare failed: %s', ME.message);
                obj.fireOnComplete();
                rethrow(ME);
            end
        end

        function d = isDone(obj)
            d = strcmp(obj.state, 'COMPLETE');
        end

        function delete(obj)
            obj.stopTimer();
        end
    end


    methods (Access = private)

        function prepare(obj)
            % -------- Build trajectory deltas + CD strings up front --
            yy = obj.hex_path.axis_cts';
            obj.ydiff = diff(round(yy));
            DT_g = round(log2(obj.hex_path.dt * 1024));
            N = size(obj.ydiff, 1);
            obj.cmdArrays = ceil(N / obj.TargetBuff);

            obj.posStr = "CD " + string(obj.ydiff(:,1)) + "," + ...
                                 string(obj.ydiff(:,2)) + "," + ...
                                 string(obj.ydiff(:,3)) + ",," + ...
                                 string(obj.ydiff(:,4)) + "," + ...
                                 string(obj.ydiff(:,5)) + "," + ...
                                 string(obj.ydiff(:,6)) + ";";

            % -------- Controller setup -------------------------------
            obj.g.GInfo;
            obj.g.GCommand('CO 15');
            obj.g.GCommand('ST');
            obj.g.GCommand('SH ABCEFG');

            if obj.trigger_flag
                CMD2 = sprintf(['#Pulse; \n SB 33; \n #A; \n SB 25; \n SB 17; \n ' ...
                                'WT16,1; \n CB 25; \n CB 17; \n WT16,1; \n JP #A; \n ' ...
                                'CB 25; \n CB 17; \n EN']);
                obj.g.GProgramDownload(CMD2);
                obj.g.GCommand('XQ #Pulse,2');
            end

            obj.g.GCommand('CMABCEFG');
            obj.g.GCommand(['DT ' num2str(DT_g)]);

            if obj.record_flag
                obj.record_t_start   = tic;
                obj.record.wall_t    = [];
                obj.record.positions = zeros(0, 8);
            end

            obj.state    = 'STREAMING';
            obj.exitcond = 'DONE';
        end

        function startTimer(obj)
            obj.timer_handle = timer( ...
                'ExecutionMode', 'fixedSpacing', ...
                'Period',        obj.TICK_PERIOD, ...
                'BusyMode',      'drop', ...
                'TimerFcn',      @(~,~) obj.tick(), ...
                'ErrorFcn',      @(~,evt) obj.onTimerError(evt));
            start(obj.timer_handle);
        end

        function stopTimer(obj)
            if ~isempty(obj.timer_handle) && isvalid(obj.timer_handle)
                try; stop(obj.timer_handle); catch; end
                try; delete(obj.timer_handle); catch; end
            end
            obj.timer_handle = [];
        end

        function onTimerError(obj, evt) %#ok<INUSD>
            obj.exitcond = 'ERROR';
            warning('ContourStreamSession:TimerError', ...
                'Stream timer raised an error; aborting.');
            obj.completeNow();
        end

        function tick(obj)
            try
                switch obj.state
                    case 'STREAMING';  obj.tickStreaming();
                    case 'TAPERING';   obj.tickTapering();
                    case 'DRAINING';   obj.tickDraining();
                    case 'COMPLETE';   obj.stopTimer();    % safety: race with shutdown
                end
            catch ME
                warning('ContourStreamSession:TickFailed', ...
                    'Stream tick failed in state %s: %s', obj.state, ME.message);
                obj.exitcond = 'ERROR';
                obj.completeNow();
            end
        end

        function tickStreaming(obj)
            % Soft-stop?
            if ~isempty(obj.cancel_token) && obj.cancel_token.requested
                obj.exitcond = 'SOFTSTOP';
                obj.beginTaper();
                return
            end

            % All planned chunks sent?
            if obj.n > obj.cmdArrays
                obj.state = 'DRAINING';
                return
            end

            % Buffer health snapshot
            free_val = obj.queryFree();
            if isempty(free_val); return; end
            obj.trackPending(free_val);

            if free_val >= obj.TargetBuff
                obj.sendChunk();
            end
        end

        function sendChunk(obj)
            if length(obj.posStr) < obj.j + obj.TargetBuff
                command = strjoin(obj.posStr(obj.i:end, 1));
            else
                command = strjoin(obj.posStr(obj.i:(obj.j + obj.TargetBuff), 1));
            end
            obj.g.GCommand(command);

            obj.recordIfEnabled();

            obj.n = obj.n + 1;
            obj.i = obj.i + obj.TargetBuff;
            obj.j = obj.j + obj.TargetBuff;
        end

        function beginTaper(obj)
            remaining_start = obj.j + 1;
            if remaining_start > size(obj.ydiff, 1)
                % nothing left to stream; skip taper
                obj.state = 'DRAINING';
                return
            end

            obj.N_taper = min(round(1 / obj.hex_path.dt), ...
                              size(obj.ydiff, 1) - remaining_start + 1);
            if obj.N_taper >= 2
                k = (0:obj.N_taper-1)';
                w = 0.5 * (1 + cos(pi * k / (obj.N_taper - 1)));
            else
                w = zeros(obj.N_taper, 1);
            end
            tapered = round(obj.ydiff(remaining_start : remaining_start + obj.N_taper - 1, :) .* w);

            obj.taperStr = "CD " + string(tapered(:,1)) + "," + string(tapered(:,2)) + "," + ...
                                   string(tapered(:,3)) + ",," + string(tapered(:,4)) + "," + ...
                                   string(tapered(:,5)) + "," + string(tapered(:,6)) + ";";
            obj.kt    = 1;
            obj.state = 'TAPERING';
        end

        function tickTapering(obj)
            if obj.kt > obj.N_taper
                obj.state = 'DRAINING';
                return
            end

            free_val = obj.queryFree();
            if isempty(free_val); return; end
            obj.trackPending(free_val);

            if free_val >= obj.TargetBuff
                chunkEnd = min(obj.kt + obj.TargetBuff - 1, obj.N_taper);
                obj.g.GCommand(strjoin(obj.taperStr(obj.kt:chunkEnd, 1)));
                obj.recordIfEnabled();
                obj.kt = chunkEnd + 1;
            end
        end

        function tickDraining(obj)
            free_val = obj.queryFree();
            if isempty(free_val); return; end

            if free_val ~= obj.BUFFER_CAPACITY
                return      % still draining, wait for next tick
            end

            % One last record sample now that drain is complete
            obj.recordIfEnabled();

            obj.g.GCommand('CD 0,0,0,,0,0,0=0');   % contour end sentinel
            obj.g.GCommand('CB25');
            obj.g.GCommand('CB17');
            obj.g.GCommand('CB33');
            obj.g.GCommand('ST');

            obj.completeNow();
        end

        function completeNow(obj)
            % Finalize buffer-health report.
            if isfinite(obj.buf_min_pending)
                obj.record.buffer_health.min_pending  = obj.buf_min_pending;
                if obj.buf_min_pending < obj.STARVATION_THRESHOLD
                    obj.record.buffer_health.starved_risk = true;
                    warning('ContourStreamSession:BufferLow', ...
                        ['Contour buffer pending dropped to %d samples during stream ', ...
                         '(starvation threshold %d).'], ...
                        obj.buf_min_pending, obj.STARVATION_THRESHOLD);
                end
            end

            obj.state = 'COMPLETE';
            obj.stopTimer();
            obj.fireOnComplete();
        end

        function fireOnComplete(obj)
            if ~isempty(obj.on_complete)
                try
                    obj.on_complete(obj.exitcond, obj.record);
                catch ME
                    warning('ContourStreamSession:OnCompleteFailed', ...
                        'on_complete handler threw: %s', ME.message);
                end
            end
        end

        function free_val = queryFree(obj)
            % Wraps the CM? query with a try that returns [] on failure
            % so the tick can skip gracefully rather than crashing the
            % state machine.
            try
                resp = obj.g.GCommand('CM?');
            catch
                free_val = [];
                return
            end
            free_val = str2num(resp.string); %#ok<ST2NM>
            if isempty(free_val) || ~isfinite(free_val)
                free_val = [];
            end
        end

        function trackPending(obj, free_val)
            pending_val = obj.BUFFER_CAPACITY - free_val;
            if pending_val < obj.buf_min_pending
                obj.buf_min_pending = pending_val;
            end
        end

        function recordIfEnabled(obj)
            if ~obj.record_flag; return; end
            obj.record_idx = obj.record_idx + 1;
            obj.record.wall_t(obj.record_idx, 1)    = toc(obj.record_t_start);
            obj.record.positions(obj.record_idx, :) = obj.sampleTP();
        end

        function pos = sampleTP(obj)
            try
                resp = obj.g.GCommand('TP');
                parts = strsplit(strtrim(resp.string), ',');
                vals  = str2double(parts);
                vals  = vals(~isnan(vals));
                pos   = zeros(1, 8);
                nv    = min(length(vals), 8);
                pos(1:nv) = vals(1:nv);
            catch
                pos = zeros(1, 8);
            end
        end
    end
end
