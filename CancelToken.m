classdef CancelToken < handle
    % CancelToken - Thread-safe-ish flag for cooperative cancellation of
    % blocking operations. One component sets the flag; another polls it.
    %
    % The streaming loop in StreamContourData_to_Galil.m reads the flag
    % on every iteration; the HexControl Soft Stop button callback
    % sets it. Handle semantics mean callers share the same underlying
    % state without having to pass struct updates back and forth.
    %
    % Typical use:
    %     app.softStop = CancelToken();
    %     StreamContourData_to_Galil(g, hex_path, 0, 0, app.softStop);
    %   % ... elsewhere, in the Soft Stop button callback:
    %     app.softStop.request();

    properties (SetAccess = private)
        requested logical = false
        requestedAt double = NaN    % toc() reference at request time
    end

    properties (Access = private)
        ticStart uint64
    end

    methods
        function obj = CancelToken()
            obj.ticStart = tic;
        end

        function request(obj)
            if ~obj.requested
                obj.requested = true;
                obj.requestedAt = toc(obj.ticStart);
            end
        end

        function reset(obj)
            obj.requested = false;
            obj.requestedAt = NaN;
            obj.ticStart = tic;
        end
    end
end
